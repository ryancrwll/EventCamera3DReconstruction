#pragma once

#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class EventRectifier {
private:
    cv::Mat map1_x, map1_y;
    cv::Mat map2_x, map2_y;
    cv::Mat P1, P2;
    cv::Mat K1, D1, K2, D2, rvec, tvec;
    cv::Size imageSize;

    void loadCameraGeometry(const std::string& path, cv::Mat& K, cv::Mat& D) {
        std::ifstream file(path);
        if (!file.is_open()) throw std::runtime_error("Cannot open camera JSON: " + path);
        json j; file >> j;

        K = cv::Mat::zeros(3, 3, CV_64F);
        auto k_vec = j["K"].get<std::vector<double>>();
        for (int i = 0; i < 9; ++i) K.at<double>(i / 3, i % 3) = k_vec[i];

        D = cv::Mat::zeros(1, 5, CV_64F);
        auto d_vec = j["D"].get<std::vector<double>>();
        for (int i = 0; i < 5; ++i) D.at<double>(0, i) = d_vec[i];
    }

    void loadExtrinsics(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) throw std::runtime_error("Cannot open extrinsics JSON: " + path);
        json j; file >> j;

        auto transform = j["T_slave_master"][0];

        rvec = cv::Mat::zeros(3, 1, CV_64F);
        auto r_vec = transform["rvec"].get<std::vector<double>>();
        for (int i = 0; i < 3; ++i) rvec.at<double>(i, 0) = r_vec[i];

        tvec = cv::Mat::zeros(3, 1, CV_64F);
        auto t_vec = transform["tvec"].get<std::vector<double>>();
        for (int i = 0; i < 3; ++i) tvec.at<double>(i, 0) = t_vec[i];

        // Invert matrices to get Master-to-Slave from Slave-to-Master (Ekalibr does it backwards then OpenCV expects)
        cv::Mat R_raw;
        cv::Rodrigues(rvec, R_raw);

        cv::Mat R_inv = R_raw.t(); // Transpose = Inverse for Rotation Matrices
        cv::Mat T_inv = -R_inv * tvec;

        cv::Rodrigues(R_inv, rvec);
        tvec = T_inv;
    }

public:
    EventRectifier(const std::string& master_json, const std::string& slave_json,
                   const std::string& extrinsics_json, int width = 1280, int height = 720)
        : imageSize(width, height) {

        loadCameraGeometry(master_json, K1, D1);
        loadCameraGeometry(slave_json, K2, D2);
        loadExtrinsics(extrinsics_json);

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        cv::Mat R1, R2, Q;
        cv::stereoRectify(K1, D1, K2, D2, imageSize, R, tvec,
                          R1, R2, P1, P2, Q,
                          cv::CALIB_ZERO_DISPARITY, 0, imageSize);

        cv::initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_32FC1, map1_x, map1_y);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_32FC1, map2_x, map2_y);

        std::cout << "[Rectifier] Successfully initialized LUT maps from JSON files.\n";
    }

    cv::Point2f rectify(int x, int y, bool is_master) const {
        if (x < 0 || x >= imageSize.width || y < 0 || y >= imageSize.height) {
            return cv::Point2f(-1.0f, -1.0f);
        }
        if (is_master) return cv::Point2f(map1_x.at<float>(y, x), map1_y.at<float>(y, x));
        else return cv::Point2f(map2_x.at<float>(y, x), map2_y.at<float>(y, x));
    }

    // --- NEW: BATCH TRIANGULATION ---
    std::vector<cv::Point3f> triangulateBatch(const std::vector<cv::Point2f>& pts_master, const std::vector<cv::Point2f>& pts_slave) const {
        if (pts_master.empty() || pts_slave.empty()) return {};

        cv::Mat points4D;
        // Solves the SVD for HUNDREDS of points simultaneously in one function call
        cv::triangulatePoints(P1, P2, pts_master, pts_slave, points4D);
        points4D.convertTo(points4D, CV_32F);

        std::vector<cv::Point3f> results;
        results.reserve(points4D.cols);

        for (int i = 0; i < points4D.cols; ++i) {
            float w = points4D.at<float>(3, i);
            if (std::abs(w) < 1e-6) {
                results.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
                continue;
            }

            float x = points4D.at<float>(0, i) / w;
            float y = points4D.at<float>(1, i) / w;
            float z = points4D.at<float>(2, i) / w;

            if (z < 0) {
                x = -x;
                y = -y;
                z = -z;
            }
            results.push_back(cv::Point3f(x, y, z));
        }
        return results;
    }

    void getCameraFrustums(float max_depth, std::vector<cv::Point3f>& left_rays, std::vector<cv::Point3f>& right_rays) const {
        double fx = P1.at<double>(0, 0);
        double fy = P1.at<double>(1, 1);
        double cx1 = P1.at<double>(0, 2);
        double cx2 = P2.at<double>(0, 2);
        double cy  = P1.at<double>(1, 2);
        double tx  = P2.at<double>(0, 3) / fx;

        std::vector<cv::Point2f> corners = {{0,0}, {1279,0}, {1279,719}, {0,719}};

        for (const auto& corner : corners) {
            for (float z = 0.0f; z <= max_depth; z += 0.01f) {
                float lx = (corner.x - cx1) * z / fx;
                float ly = (corner.y - cy) * z / fy;
                left_rays.push_back(cv::Point3f(lx, ly, z));

                float rx = (corner.x - cx2) * z / fx + tx;
                float ry = (corner.y - cy) * z / fy;
                right_rays.push_back(cv::Point3f(rx, ry, z));
            }
        }
    }
    // --- NEW: Extracts (Focal Length * Baseline) in pixel-meters ---
    double getFocalBaselineProduct() const {
        // P2(0,3) is exactly -fx * Tx (Focal Length * Baseline)
        return std::abs(P2.at<double>(0, 3));
    }
};