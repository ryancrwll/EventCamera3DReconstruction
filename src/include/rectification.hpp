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

    // --- LUT VARIABLES ---
    bool use_lut = false;
    cv::Mat lut_x_m, lut_y_m;
    cv::Mat lut_x_s, lut_y_s;

    // --- THE PUSH MAP INVERTER (Bilinear Splatting) ---
    void invertMap(const cv::Mat& pull_x, const cv::Mat& pull_y, cv::Mat& push_x, cv::Mat& push_y) {
        push_x = cv::Mat::zeros(imageSize, CV_32FC1);
        push_y = cv::Mat::zeros(imageSize, CV_32FC1);
        cv::Mat weight = cv::Mat::zeros(imageSize, CV_32FC1);

        // 1. Bilinear Forward Warping (Splatting)
        // This guarantees a perfectly smooth sub-pixel gradient, preventing jagged laser lines!
        for (int v = 0; v < imageSize.height; v++) {
            for (int u = 0; u < imageSize.width; u++) {
                float px = pull_x.at<float>(v, u);
                float py = pull_y.at<float>(v, u);

                // ADD THIS LINE: Catch the Pinax reflection failures before they corrupt the map!
                if (std::isnan(px) || std::isnan(py)) continue;

                int x0 = std::floor(px);
                int y0 = std::floor(py);
                int x1 = x0 + 1;
                int y1 = y0 + 1;

                float dx = px - x0;
                float dy = py - y0;

                float w00 = (1.0f - dx) * (1.0f - dy);
                float w10 = dx * (1.0f - dy);
                float w01 = (1.0f - dx) * dy;
                float w11 = dx * dy;

                auto add_weight = [&](int x, int y, float w, float val_u, float val_v) {
                    if (x >= 0 && x < imageSize.width && y >= 0 && y < imageSize.height) {
                        push_x.at<float>(y, x) += w * val_u;
                        push_y.at<float>(y, x) += w * val_v;
                        weight.at<float>(y, x) += w;
                    }
                };

                add_weight(x0, y0, w00, u, v);
                add_weight(x1, y0, w10, u, v);
                add_weight(x0, y1, w01, u, v);
                add_weight(x1, y1, w11, u, v);
            }
        }

        // 2. Normalize by the accumulated weights
        for (int y = 0; y < imageSize.height; y++) {
            for (int x = 0; x < imageSize.width; x++) {
                if (weight.at<float>(y, x) > 1e-5f) {
                    push_x.at<float>(y, x) /= weight.at<float>(y, x);
                    push_y.at<float>(y, x) /= weight.at<float>(y, x);
                } else {
                    push_x.at<float>(y, x) = -1.0f; // Mark as a hole
                }
            }
        }

        // 3. Multi-directional Inpainting to fill remaining holes
        // Sweeps L->R, R->L, and T->B to ensure no -1 values break the line tracker
        for (int y = 0; y < imageSize.height; y++) {
            for (int x = 1; x < imageSize.width; x++) {
                if (push_x.at<float>(y, x) < 0 && push_x.at<float>(y, x - 1) >= 0) {
                    push_x.at<float>(y, x) = push_x.at<float>(y, x - 1);
                    push_y.at<float>(y, x) = push_y.at<float>(y, x - 1);
                }
            }
            for (int x = imageSize.width - 2; x >= 0; x--) {
                if (push_x.at<float>(y, x) < 0 && push_x.at<float>(y, x + 1) >= 0) {
                    push_x.at<float>(y, x) = push_x.at<float>(y, x + 1);
                    push_y.at<float>(y, x) = push_y.at<float>(y, x + 1);
                }
            }
        }
        for (int x = 0; x < imageSize.width; x++) {
            for (int y = 1; y < imageSize.height; y++) {
                if (push_x.at<float>(y, x) < 0 && push_x.at<float>(y - 1, x) >= 0) {
                    push_x.at<float>(y, x) = push_x.at<float>(y - 1, x);
                    push_y.at<float>(y, x) = push_y.at<float>(y - 1, x);
                }
            }
        }
    }

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

        // Invert matrices to get Master-to-Slave from Slave-to-Master
        cv::Mat R_raw;
        cv::Rodrigues(rvec, R_raw);

        cv::Mat R_inv = R_raw.t();
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
                          cv::CALIB_ZERO_DISPARITY, -1.0, imageSize);

        // Generate the standard Air "Pull" maps
        cv::Mat pull1_x, pull1_y, pull2_x, pull2_y;
        cv::initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_32FC1, pull1_x, pull1_y);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_32FC1, pull2_x, pull2_y);

        // Convert them to Event "Push" maps!
        invertMap(pull1_x, pull1_y, map1_x, map1_y);
        invertMap(pull2_x, pull2_y, map2_x, map2_y);
    }

    void loadLUT(const std::string& yaml_path) {
        cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "[!] Error: Could not open LUT file: " << yaml_path << std::endl;
            return;
        }

        std::cout << "-> Loading Custom Refraction LUT: " << yaml_path << std::endl;

        // Load the Pinax "Pull" maps
        cv::Mat pull_x_m, pull_y_m, pull_x_s, pull_y_s;
        fs["map_x_left"] >> pull_x_m;
        fs["map_y_left"] >> pull_y_m;
        fs["map_x_right"] >> pull_x_s;
        fs["map_y_right"] >> pull_y_s;
        fs.release();

        // Convert them to Event "Push" maps!
        invertMap(pull_x_m, pull_y_m, lut_x_m, lut_y_m);
        invertMap(pull_x_s, pull_y_s, lut_x_s, lut_y_s);

        use_lut = true;

        std::cout << "   [+] LUT loaded and INVERTED successfully! Resolution: "
                  << lut_x_m.cols << "x" << lut_x_m.rows << std::endl;
    }

    cv::Point2f rectify(int x, int y, bool is_master) const {
        if (x < 0 || x >= imageSize.width || y < 0 || y >= imageSize.height) {
            return cv::Point2f(-1.0f, -1.0f);
        }

        if (use_lut) {
            if (is_master) return cv::Point2f(lut_x_m.at<float>(y, x), lut_y_m.at<float>(y, x));
            else           return cv::Point2f(lut_x_s.at<float>(y, x), lut_y_s.at<float>(y, x));
        }
        else {
            if (is_master) return cv::Point2f(map1_x.at<float>(y, x), map1_y.at<float>(y, x));
            else           return cv::Point2f(map2_x.at<float>(y, x), map2_y.at<float>(y, x));
        }
    }

    std::vector<cv::Point3f> triangulateBatch(const std::vector<cv::Point2f>& pts_master, const std::vector<cv::Point2f>& pts_slave) const {
        if (pts_master.empty() || pts_slave.empty()) return {};

        cv::Mat points4D;
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

    double getFocalBaselineProduct() const {
        // Correct! P2(0,3) is actually -fx * tx, which is the baseline focal product.
        return std::abs(P2.at<double>(0, 3));
    }
};