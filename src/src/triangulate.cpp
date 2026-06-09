#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <fstream>
#include <algorithm>
#include <atomic>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <Eigen/Dense>

#include "pinax_model.hpp"
#include "../include/rectification.hpp"

using namespace std;
using namespace cv;

std::atomic<double> global_first_packet_t{-1.0};

struct Point3D {
    float x, y, z;
    int r = 0, g = 255, b = 100;
};

struct RawEvent {
    int x, y;
    uint8_t p;
    double t;
};

// ==============================================================================
// JSON PARSERS
// ==============================================================================
bool parse_json_intrinsics(const string& filepath, Mat& K, Mat& D) {
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    FileNode k_node = fs["K"]; FileNode d_node = fs["D"];
    K = Mat::zeros(3, 3, CV_64F);
    K.at<double>(0,0) = (double)k_node[0]; K.at<double>(0,1) = (double)k_node[1]; K.at<double>(0,2) = (double)k_node[2];
    K.at<double>(1,0) = (double)k_node[3]; K.at<double>(1,1) = (double)k_node[4]; K.at<double>(1,2) = (double)k_node[5];
    K.at<double>(2,0) = (double)k_node[6]; K.at<double>(2,1) = (double)k_node[7]; K.at<double>(2,2) = (double)k_node[8];
    D = Mat::zeros(1, 5, CV_64F);
    for(int i=0; i<5; i++) D.at<double>(0,i) = (double)d_node[i];
    fs.release();
    return true;
}

bool parse_json_extrinsics(const string& filepath, Mat& R, Mat& T) {
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    FileNode tsm = fs["T_slave_master"];
    if (tsm.empty()) return false;
    FileNode cam = tsm[0];
    Mat rvec = Mat::zeros(3, 1, CV_64F);
    rvec.at<double>(0) = (double)cam["rvec"][0]; rvec.at<double>(1) = (double)cam["rvec"][1]; rvec.at<double>(2) = (double)cam["rvec"][2];
    Rodrigues(rvec, R);
    T = Mat::zeros(3, 1, CV_64F);
    T.at<double>(0) = (double)cam["tvec"][0]; T.at<double>(1) = (double)cam["tvec"][1]; T.at<double>(2) = (double)cam["tvec"][2];
    fs.release();
    return true;
}

// ==============================================================================
// PINAX LUT GENERATOR & INVERTER
// ==============================================================================
void build_refractive_stereo_LUT(
    const Mat& K_virtual, const Mat& K_physical, const Mat& D_physical,
    const Eigen::Matrix3d& R_tilt, const jir_refractive_image_geometry_msgs::PlanarRefractionInfo& glass_info,
    double Z_target, Mat& map_x, Mat& map_y, int W = 1280, int H = 720)
{
    map_x = Mat(H, W, CV_32FC1); map_y = Mat(H, W, CV_32FC1);
    jir_refractive_image_geometry::RefractedPinholeCameraModel pinax_cam;
    sensor_msgs::msg::CameraInfo cam_info;
    cam_info.k.fill(0.0); cam_info.k[8] = 1.0;
    cam_info.k[0] = K_physical.at<double>(0,0); cam_info.k[4] = K_physical.at<double>(1,1);
    cam_info.k[2] = K_physical.at<double>(0,2); cam_info.k[5] = K_physical.at<double>(1,2);
    cam_info.p.fill(0.0); cam_info.p[10] = 1.0;
    cam_info.p[0] = K_physical.at<double>(0,0); cam_info.p[5] = K_physical.at<double>(1,1);
    cam_info.p[2] = K_physical.at<double>(0,2); cam_info.p[6] = K_physical.at<double>(1,2);
    cam_info.d.resize(5); for(int i=0; i<5; i++) cam_info.d[i] = D_physical.at<double>(0,i);
    pinax_cam.fromCameraInfo(cam_info); pinax_cam.fromPlanarRefractionInfo(glass_info);

    double f_virt_x = K_virtual.at<double>(0,0); double f_virt_y = K_virtual.at<double>(1,1);
    double c_virt_x = K_virtual.at<double>(0,2); double c_virt_y = K_virtual.at<double>(1,2);

    for (int v = 0; v < H; v++) {
        for (int u = 0; u < W; u++) {
            double nx = (u - c_virt_x) / f_virt_x; double ny = (v - c_virt_y) / f_virt_y;
            Eigen::Vector3d pt_3d_virtual(nx * Z_target, ny * Z_target, Z_target);
            Eigen::Vector3d pt_3d_physical = R_tilt.transpose() * pt_3d_virtual;
            Eigen::Vector2d distorted_pixel = pinax_cam.project3dToPixel(pt_3d_physical);
            map_x.at<float>(v, u) = (float)distorted_pixel.x(); map_y.at<float>(v, u) = (float)distorted_pixel.y();
        }
    }
}

class LUTInverter {
    cv::flann::Index kdtree; int W, H;
public:
    LUTInverter(const Mat& map_x, const Mat& map_y) {
        H = map_x.rows; W = map_x.cols;
        Mat features(H * W, 2, CV_32F);
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                features.at<float>(y * W + x, 0) = map_x.at<float>(y, x);
                features.at<float>(y * W + x, 1) = map_y.at<float>(y, x);
            }
        }
        kdtree.build(features, cv::flann::KDTreeIndexParams(4), cvflann::FLANN_DIST_L2);
    }

    Point2f invert(Point2f distorted_pt) {
        vector<float> query = {distorted_pt.x, distorted_pt.y};
        vector<int> indices(4); vector<float> dists(4);
        kdtree.knnSearch(query, indices, dists, 4, cv::flann::SearchParams(32));

        if (dists[0] > 100.0f) {
            return Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
        }

        float sum_weight = 0; Point2f rectified_pt(0, 0);
        for (int i = 0; i < 4; i++) {
            float weight = 1.0f / (dists[i] + 1e-6f);
            rectified_pt.x += (indices[i] % W) * weight;
            rectified_pt.y += (indices[i] / W) * weight;
            sum_weight += weight;
        }
        return Point2f(rectified_pt.x / sum_weight, rectified_pt.y / sum_weight);
    }
};

// ==============================================================================
// EVENT PROCESSOR
// ==============================================================================
class TriangulationProcessor : public event_camera_codecs::EventProcessor {
public:
    bool is_master;

    cv::Mat inv_map_x_m, inv_map_y_m;
    cv::Mat inv_map_x_s, inv_map_y_s;

    std::deque<RawEvent> master_raw;
    std::deque<RawEvent> slave_raw;

    double latest_master_time = 0.0;
    double latest_slave_time = 0.0;
    double current_packet_t[2] = {0.0, 0.0};
    double current_offset[2] = {0.0, 0.0};
    bool is_first_event_in_packet[2] = {true, true};

    void setPacketTime(double pkt_t, bool master) {
        int idx = master ? 1 : 0;
        current_packet_t[idx] = pkt_t;
        double expected = -1.0;
        if (global_first_packet_t.load() < 0) {
            global_first_packet_t.compare_exchange_strong(expected, pkt_t);
        }
        is_first_event_in_packet[idx] = true;
    }

    inline void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override {
        int idx = is_master ? 1 : 0;
        double raw_t = sensor_time * 1e-9;
        double expected_t = current_packet_t[idx] - global_first_packet_t.load();

        if (is_first_event_in_packet[idx]) {
            current_offset[idx] = raw_t - expected_t;
            is_first_event_in_packet[idx] = false;
        }

        double final_t = raw_t - current_offset[idx];

        if (std::abs(final_t - expected_t) > 1.0) {
            current_offset[idx] = raw_t - expected_t;
            final_t = raw_t - current_offset[idx];
        }

        if (final_t < 0.0) return;

        if (is_master) latest_master_time = final_t;
        else latest_slave_time = final_t;

        float rx, ry;
        if (is_master) {
            rx = inv_map_x_m.at<float>(ey, ex);
            ry = inv_map_y_m.at<float>(ey, ex);
        } else {
            rx = inv_map_x_s.at<float>(ey, ex);
            ry = inv_map_y_s.at<float>(ey, ex);
        }

        int x = std::round(rx);
        int y = std::round(ry);

        if (x >= 0 && x < 1280 && y >= 0 && y < 720) {
            if (is_master) master_raw.push_back({x, y, polarity, final_t});
            else slave_raw.push_back({x, y, polarity, final_t});
        }
    }

    inline bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return true; }
    inline void finished() override {}
    inline void rawData(const char *, size_t) override {}
};

void writePLY(const string& filename, const vector<Point3D>& cloud) {
    ofstream file(filename);
    file.imbue(std::locale("C"));
    file << "ply\nformat ascii 1.0\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\nproperty float y\nproperty float z\n";
    file << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    file << "end_header\n";
    for (const auto& p : cloud) file << p.x << " " << p.y << " " << p.z << " " << p.r << " " << p.g << " " << p.b << "\n";
    file.close();
    cout << "\nPoint cloud saved to " << filename << " (" << cloud.size() << " points)\n";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ==========================================
    // TERMINAL INTERACTION
    // ==========================================
    const double WINDOW_SEC = 0.001;
    bool USE_UNDERWATER_LUT = false;

    string intrinsics_master = "calib_data/new_meta_int_master.json";
    string intrinsics_slave  = "calib_data/new_meta_int_slave.json";
    string extrinsics_stereo = "calib_data/extrinsics_UW2.json";
    EventRectifier event_rectifier(intrinsics_master, intrinsics_slave, extrinsics_stereo);

    string mcap_file = "/home/ryan/Documents/MIRS/Thesis/3Dreconstruction/datasets/uw_laser/uw_laser_0.mcap";

    cout << "\n============================================\n";
    cout << " SYSTEM CONFIGURATION:\n";
    cout << " 1. Air (Standard Pinhole)\n";
    cout << " 2. Water (Dynamic Pinax Refraction LUT)\n";
    cout << "============================================\n";
    cout << "Enter mode (1 or 2): ";
    string mode_input; getline(cin, mode_input);
    if (mode_input == "2") {
        USE_UNDERWATER_LUT = true;
        cout << "-> Mode: Water (Refractive)\n";
    } else {
        cout << "-> Mode: Air (Pinhole)\n";
    }

    cout << "\nPath to .mcap file (press ENTER for default:\n"
         << mcap_file << ")\n> ";
    string path_input; getline(cin, path_input);
    if (!path_input.empty()) {
        mcap_file = path_input;
    }
    cout << "-> Using .mcap file: " << mcap_file << "\n";

    string master_topic = "/event_cam_0/events";
    string slave_topic = "/event_cam_1/events";

    cout << "Loading Calibration and Building Dynamic LUTs...\n";
    Mat K_L, D_L, K_R, D_R, R_stereo, T_stereo;
    parse_json_intrinsics(intrinsics_master, K_L, D_L);
    parse_json_intrinsics(intrinsics_slave, K_R, D_R);
    parse_json_extrinsics(extrinsics_stereo, R_stereo, T_stereo);

    int W = 1280, H = 720;
    Mat R1, R2, P1_rect, P2_rect, Q;
    stereoRectify(K_L, D_L, K_R, D_R, Size(W, H), R_stereo, T_stereo,
                  R1, R2, P1_rect, P2_rect, Q, CALIB_ZERO_DISPARITY, 0.0);

    TriangulationProcessor processor;
    processor.inv_map_x_m = Mat(H, W, CV_32FC1);
    processor.inv_map_y_m = Mat(H, W, CV_32FC1);
    processor.inv_map_x_s = Mat(H, W, CV_32FC1);
    processor.inv_map_y_s = Mat(H, W, CV_32FC1);

    if (USE_UNDERWATER_LUT) {
        double TARGET_DIST = 0.60;

        jir_refractive_image_geometry_msgs::PlanarRefractionInfo glass_L, glass_R;
        glass_L.normal[0] = 0.0633452; glass_L.normal[1] = 0.0; glass_L.normal[2] = -0.997992;
        glass_L.d_0 = 0.00606132; glass_L.d_1 = 0.0049; glass_L.n_glass = 1.492; glass_L.n_water = 1.333;

        glass_R.normal[0] = -0.143032; glass_R.normal[1] = -0.0034768; glass_R.normal[2] = -0.989712;
        glass_R.d_0 = 0.00594584; glass_R.d_1 = 0.0049; glass_R.n_glass = 1.492; glass_R.n_water = 1.333;

        cv::Mat K_virtual = P1_rect(Rect(0, 0, 3, 3));
        Eigen::Matrix3d R_left_eig, R_right_eig;
        for(int r=0; r<3; r++) {
            for(int c=0; c<3; c++) {
                R_left_eig(r,c) = R1.at<double>(r,c);
                R_right_eig(r,c) = R2.at<double>(r,c);
            }
        }

        cv::Mat pull_x_m, pull_y_m, pull_x_s, pull_y_s;
        build_refractive_stereo_LUT(K_virtual, K_L, D_L, R_left_eig, glass_L, TARGET_DIST, pull_x_m, pull_y_m, W, H);
        build_refractive_stereo_LUT(K_virtual, K_R, D_R, R_right_eig, glass_R, TARGET_DIST, pull_x_s, pull_y_s, W, H);
        std::string temp_lut_file = "temp_dynamic_pinax_lut.yaml";
        cv::FileStorage fs(temp_lut_file, cv::FileStorage::WRITE);
        fs << "map_x_left" << pull_x_m;
        fs << "map_y_left" << pull_y_m;
        fs << "map_x_right" << pull_x_s;
        fs << "map_y_right" << pull_y_s;
        fs.release();

        event_rectifier.loadLUT(temp_lut_file);

        LUTInverter inv_L(pull_x_m, pull_y_m);
        LUTInverter inv_R(pull_x_s, pull_y_s);

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                Point2f pt(x, y);
                Point2f rect_L = inv_L.invert(pt);
                Point2f rect_R = inv_R.invert(pt);
                processor.inv_map_x_m.at<float>(y, x) = rect_L.x;
                processor.inv_map_y_m.at<float>(y, x) = rect_L.y;
                processor.inv_map_x_s.at<float>(y, x) = rect_R.x;
                processor.inv_map_y_s.at<float>(y, x) = rect_R.y;
            }
        }
    } else {
        cv::initUndistortRectifyMap(K_L, D_L, R1, P1_rect, Size(W, H), CV_32FC1, processor.inv_map_x_m, processor.inv_map_y_m);
        cv::initUndistortRectifyMap(K_R, D_R, R2, P2_rect, Size(W, H), CV_32FC1, processor.inv_map_x_s, processor.inv_map_y_s);
    }
    cout << "LUTs Initialized. Ready for Events.\n";

    double f_b = std::abs(P2_rect.at<double>(0, 3));
    float PHYSICAL_MIN_DEPTH = 0.3f;
    float PHYSICAL_MAX_DEPTH = 0.6f;
    double MIN_DISPARITY = f_b / PHYSICAL_MAX_DEPTH;
    double MAX_DISPARITY = f_b / PHYSICAL_MIN_DEPTH;

    int Y_OFFSET = 0;
    double window_start = 0.0;
    double global_sum_z = 0.0, global_sum_y_diff = 0.0;
    uint64_t global_pt_count = 0, global_m_det = 0, global_s_det = 0, global_epi_pass = 0, global_disp_pass = 0;

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = mcap_file;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader.open(storage_options, converter_options);

    event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, TriangulationProcessor> decoderFactory;
    rclcpp::Serialization<event_camera_msgs::msg::EventPacket> serializer;

    std::vector<Point3D> point_cloud;

    while (reader.has_next() && rclcpp::ok()) {
        auto msg = reader.read_next();
        if (msg->topic_name != master_topic && msg->topic_name != slave_topic) continue;

        processor.is_master = (msg->topic_name == master_topic);
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto ros_msg = std::make_shared<event_camera_msgs::msg::EventPacket>();
        serializer.deserialize_message(&serialized_msg, ros_msg.get());

        double pkt_t = ros_msg->header.stamp.sec + ros_msg->header.stamp.nanosec * 1e-9;
        processor.setPacketTime(pkt_t, processor.is_master);

        auto decoder = decoderFactory.getInstance(*ros_msg);
        if (decoder) decoder->decode(*ros_msg, &processor);

        double safe_time = std::min(processor.latest_master_time, processor.latest_slave_time);
        if (window_start == 0.0 && safe_time > 0.0) window_start = safe_time;

        while (safe_time > window_start + WINDOW_SEC) {

            // ==========================================
            // PROBABILISTIC SPATIAL GRADIENT ACCUMULATION
            // ==========================================
            std::vector<std::vector<int>> m_pol(720, std::vector<int>(1280, 0));
            std::vector<std::vector<int>> s_pol(720, std::vector<int>(1280, 0));

            for (const auto& ev : processor.master_raw) {
                if (ev.t < window_start) continue;
                if (ev.t > window_start + WINDOW_SEC) break;
                m_pol[ev.y][ev.x] += (ev.p == 1) ? 1 : -1;
            }

            for (const auto& ev : processor.slave_raw) {
                if (ev.t < window_start) continue;
                if (ev.t > window_start + WINDOW_SEC) break;
                s_pol[ev.y][ev.x] += (ev.p == 1) ? 1 : -1;
            }

            std::vector<cv::Point2f> master_pts, slave_pts;
            const int WINDOW = 5;
            const int MIN_ON_CLUSTER = 3;
            const int MIN_SCORE = 5;

            for (int y = 0; y < 720; ++y) {
                int best_x_m = -1, max_score_m = 0;
                int best_x_s = -1, max_score_s = 0;

                for (int x = WINDOW; x < 1280 - WINDOW; ++x) {
                    int L_m = 0, R_m = 0, L_s = 0, R_s = 0;
                    for (int i = 1; i <= WINDOW; ++i) {
                        L_m += m_pol[y][x - i]; R_m += m_pol[y][x + i];
                        L_s += s_pol[y][x - i]; R_s += s_pol[y][x + i];
                    }
                    if (L_m >= MIN_ON_CLUSTER && (L_m - R_m) > max_score_m) { max_score_m = L_m - R_m; best_x_m = x; }
                    if (L_s >= MIN_ON_CLUSTER && (L_s - R_s) > max_score_s) { max_score_s = L_s - R_s; best_x_s = x; }
                }
                if (max_score_m >= MIN_SCORE) master_pts.push_back(cv::Point2f(best_x_m, y));
                if (max_score_s >= MIN_SCORE) slave_pts.push_back(cv::Point2f(best_x_s, y));
            }

            std::vector<cv::Point2f> batch_m;
            std::vector<cv::Point2f> batch_s;
            std::vector<bool> s_used(slave_pts.size(), false);

            int dbg_m_det = master_pts.size(), dbg_s_det = slave_pts.size();
            int dbg_epi_pass = 0, dbg_disp_pass = 0;
            double dbg_sum_y_diff = 0.0;

            for (const auto& m : master_pts) {
                int best_s_idx = -1;
                float min_y_err = 1e9;

                for (size_t j = 0; j < slave_pts.size(); ++j) {
                    if (s_used[j]) continue;

                    float y_err = std::abs(slave_pts[j].y - (m.y + Y_OFFSET));
                    double epipolar_gate = USE_UNDERWATER_LUT ? 6.0f : 2.0f;

                    if (y_err <= epipolar_gate) {
                        float disparity = std::abs(m.x - slave_pts[j].x);
                        if (disparity >= MIN_DISPARITY && disparity <= MAX_DISPARITY) {
                            if (y_err < min_y_err) {
                                min_y_err = y_err;
                                best_s_idx = j;
                            }
                        }
                    }
                }

                if (best_s_idx != -1) {
                    dbg_epi_pass++;
                    dbg_disp_pass++;
                    s_used[best_s_idx] = true;
                    batch_m.push_back(m);
                    batch_s.push_back(slave_pts[best_s_idx]);
                    dbg_sum_y_diff += (slave_pts[best_s_idx].y - m.y);
                }
            }

            // ==========================================
            // LINEAR TRIANGULATION
            // ==========================================
            if (!batch_m.empty()) {
                cv::Mat p1_mat(2, batch_m.size(), CV_64F);
                cv::Mat p2_mat(2, batch_s.size(), CV_64F);
                for(size_t k = 0; k < batch_m.size(); k++) {
                    p1_mat.at<double>(0, k) = batch_m[k].x; p1_mat.at<double>(1, k) = batch_m[k].y;
                    p2_mat.at<double>(0, k) = batch_s[k].x; p2_mat.at<double>(1, k) = batch_s[k].y;
                }

                cv::Mat pts_4d;
                cv::triangulatePoints(P1_rect, P2_rect, p1_mat, p2_mat, pts_4d);

                cv::Mat pts_4d_64f; pts_4d.convertTo(pts_4d_64f, CV_64F);
                for (int j = 0; j < pts_4d_64f.cols; j++) {
                    double w = pts_4d_64f.at<double>(3, j);
                    double z = pts_4d_64f.at<double>(2, j);

                    if (std::abs(w) > 1e-6 && (z / w) > 0) {
                        float p3d_x = pts_4d_64f.at<double>(0, j) / w;
                        float p3d_y = pts_4d_64f.at<double>(1, j) / w;
                        float p3d_z = z / w;

                        if (p3d_z > PHYSICAL_MIN_DEPTH && p3d_z < PHYSICAL_MAX_DEPTH) {
                            global_sum_z += p3d_z;
                            global_pt_count++;
                            point_cloud.push_back({p3d_x, p3d_y, p3d_z, 255, 255, 255});
                        }
                    }
                }
            }

            window_start += WINDOW_SEC;
            global_m_det += dbg_m_det; global_s_det += dbg_s_det;
            global_disp_pass += dbg_disp_pass; global_epi_pass += dbg_epi_pass;
            global_sum_y_diff += dbg_sum_y_diff;

            double current_global_avg_z = global_pt_count > 0 ? (global_sum_z / global_pt_count) : 0.0;

            if (dbg_m_det > 0 || dbg_s_det > 0) {
                cout << "\33[2K\r[Time: " << std::fixed << std::setprecision(1) << (window_start * 1000.0)
                     << "ms] M:" << global_m_det << " S:" << global_s_det
                     << " Epi:" << global_epi_pass << " Disp:" << global_disp_pass
                     << " AvgZ: " << std::fixed << std::setprecision(3) << current_global_avg_z << "m"
                     << " | Pts: " << global_pt_count << flush;
            }

            while (!processor.master_raw.empty() && processor.master_raw.front().t < window_start) processor.master_raw.pop_front();
            while (!processor.slave_raw.empty() && processor.slave_raw.front().t < window_start) processor.slave_raw.pop_front();
        }
    }

    cout << "\n\n============================================" << endl;
    cout << " FINAL TRIANGULATION SUMMARY" << endl;
    cout << "============================================" << endl;
    cout << " Total Master Detections : " << global_m_det << endl;
    cout << " Total Slave Detections  : " << global_s_det << endl;
    cout << " Passed Epipolar Gate    : " << global_epi_pass << endl;
    cout << " Passed Disparity Gate   : " << global_disp_pass << endl;
    cout << " Final 3D Points Created : " << global_pt_count << endl;
    cout << " ------------------------------------------" << endl;
    cout << " EMPIRICAL Y-OFFSET      : " << std::fixed << std::setprecision(3)
         << (global_pt_count > 0 ? (global_sum_y_diff / global_pt_count) : 0.0) << " pixels" << endl;
    cout << " GLOBAL AVERAGE Z-DEPTH  : " << std::fixed << std::setprecision(4)
         << (global_pt_count > 0 ? (global_sum_z / global_pt_count) : 0.0) << " meters" << endl;
    cout << "============================================\n" << endl;

    // ==================================================
    // ADD CAMERA FRUSTUMS TO PLY
    // ==================================================
    cout << "\nAdding camera frustums to point cloud visualization..." << endl;
    std::vector<cv::Point3f> left_rays, right_rays;

    event_rectifier.getCameraFrustums(1.5f, left_rays, right_rays);

    for (const auto& pt : left_rays) {
        // Use pure white (1.0, 1.0, 1.0) to mark as "system points"
        point_cloud.push_back({pt.x, pt.y, pt.z, 0, 0, 0});
    }
    for (const auto& pt : right_rays) {
        point_cloud.push_back({pt.x, pt.y, pt.z, 0, 0, 0});
    }

    writePLY("laser_scan.ply", point_cloud);

    rclcpp::shutdown();
    return 0;
}