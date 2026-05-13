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
#include "rectification.hpp"

using namespace std;

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

class TriangulationProcessor : public event_camera_codecs::EventProcessor {
public:
    EventRectifier* rectifier;
    bool is_master;

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

        cv::Point2f pt = rectifier->rectify(ex, ey, is_master);
        int x = std::round(pt.x);
        int y = std::round(pt.y);

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

    bool USE_HOUGH_TRANSFORM = true;

    cout << "\n============================================\n";
    cout << " SELECT LASER DETECTION ALGORITHM:\n";
    cout << " 1. Probabilistic Hough Transform (Default)\n";
    cout << " 2. Hybrid Polarity Spatial Gradient\n";
    cout << "============================================\n";
    cout << "Enter choice (1 or 2): ";

    string input;
    getline(cin, input);
    if (input == "2") {
        USE_HOUGH_TRANSFORM = false;
        cout << "-> MODE SELECTED: Polarity Spatial Gradient\n\n";
    } else {
        cout << "-> MODE SELECTED: Probabilistic Hough Transform\n\n";
    }

    string mcap_file = "/home/ryan/Documents/MIRS/Thesis/3Dreconstruction/datasets/laser7/laser6_0.mcap";
    string master_topic = "/event_cam_0/events";
    string slave_topic = "/event_cam_1/events";

    EventRectifier event_rectifier("calib_data/meta_int_master.json",
                                   "calib_data/meta_int_slav.json",
                                   "calib_data/meta2ekal_extrinsics.json");

    TriangulationProcessor processor;
    processor.rectifier = &event_rectifier;

    // --- PRODUCTION PHYSICAL GATES ---
    double f_b = event_rectifier.getFocalBaselineProduct();
    float PHYSICAL_MIN_DEPTH = 0.3f; // 30cm closest valid point
    float PHYSICAL_MAX_DEPTH = 1.2f; // 1.2m furthest valid point
    double MIN_DISPARITY = f_b / PHYSICAL_MAX_DEPTH;
    double MAX_DISPARITY = f_b / PHYSICAL_MIN_DEPTH;

    // As proven by the diagnostic test, epipolar alignment is flawless!
    int Y_OFFSET = 0;

    const double WINDOW_SEC = 0.002;
    double window_start = 0.0;

    double global_sum_z = 0.0;
    double global_sum_y_diff = 0.0;
    uint64_t global_pt_count = 0;
    uint64_t global_m_det = 0;
    uint64_t global_s_det = 0;
    uint64_t global_epi_pass = 0;
    uint64_t global_disp_pass = 0;

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

    cv::Mat bridge_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 9));
    cv::Mat thin_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

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
            cv::Mat m_mask, s_mask;
            std::vector<std::vector<int>> m_pol, s_pol;

            if (USE_HOUGH_TRANSFORM) {
                m_mask = cv::Mat::zeros(720, 1280, CV_8UC1);
                s_mask = cv::Mat::zeros(720, 1280, CV_8UC1);
            } else {
                m_pol.assign(720, std::vector<int>(1280, 0));
                s_pol.assign(720, std::vector<int>(1280, 0));
            }

            for (const auto& ev : processor.master_raw) {
                if (ev.t < window_start) continue;
                if (ev.t > window_start + WINDOW_SEC) break;
                if (USE_HOUGH_TRANSFORM) { if (ev.p == 1) m_mask.at<uchar>(ev.y, ev.x) = 255; }
                else m_pol[ev.y][ev.x] += (ev.p == 1) ? 1 : -1;
            }

            for (const auto& ev : processor.slave_raw) {
                if (ev.t < window_start) continue;
                if (ev.t > window_start + WINDOW_SEC) break;
                if (USE_HOUGH_TRANSFORM) { if (ev.p == 1) s_mask.at<uchar>(ev.y, ev.x) = 255; }
                else s_pol[ev.y][ev.x] += (ev.p == 1) ? 1 : -1;
            }

            std::vector<cv::Point2f> master_pts, slave_pts;

            if (USE_HOUGH_TRANSFORM) {
                cv::morphologyEx(m_mask, m_mask, cv::MORPH_CLOSE, bridge_kernel);
                cv::morphologyEx(s_mask, s_mask, cv::MORPH_CLOSE, bridge_kernel);
                cv::erode(m_mask, m_mask, thin_kernel);
                cv::erode(s_mask, s_mask, thin_kernel);

                std::vector<cv::Vec4i> m_lines, s_lines;
                cv::HoughLinesP(m_mask, m_lines, 3, CV_PI/180, 15, 10, 25);
                cv::HoughLinesP(s_mask, s_lines, 3, CV_PI/180, 15, 10, 25);

                auto interpolate_lines = [&](const std::vector<cv::Vec4i>& lines, std::vector<cv::Point2f>& pts) {
                    for (auto& l : lines) {
                        int x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3];
                        if (y1 > y2) { std::swap(x1, x2); std::swap(y1, y2); }
                        for (int y = y1; y <= y2; ++y) {
                            float x = x1 + (float)(x2 - x1) * (y - y1) / (std::max(1, y2 - y1));
                            pts.push_back(cv::Point2f(x, y));
                        }
                    }
                };
                interpolate_lines(m_lines, master_pts);
                interpolate_lines(s_lines, slave_pts);

            } else {
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
            }

            std::vector<cv::Point2f> batch_m;
            std::vector<cv::Point2f> batch_s;
            std::vector<bool> s_used(slave_pts.size(), false);

            int dbg_m_det = master_pts.size();
            int dbg_s_det = slave_pts.size();
            int dbg_epi_pass = 0;
            int dbg_disp_pass = 0;
            double dbg_sum_y_diff = 0.0;

            for (const auto& m : master_pts) {
                int best_s_idx = -1;
                float min_y_err = 1e9; // Find best Epipolar Y Match

                for (size_t j = 0; j < slave_pts.size(); ++j) {
                    if (s_used[j]) continue;

                    // Tight +/- 2 pixel Epipolar window safely blocks noise
                    float y_err = std::abs(slave_pts[j].y - (m.y + Y_OFFSET));
                    if (y_err <= 2.0f) {

                        // Valid, positive disparity now guaranteed by the Matrix Inversion!
                        float disparity = m.x - slave_pts[j].x;
                        if (disparity >= MIN_DISPARITY && disparity <= MAX_DISPARITY) {

                            // Lock onto the truest geometric alignment
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

            if (!batch_m.empty()) {
                std::vector<cv::Point3f> p3ds = event_rectifier.triangulateBatch(batch_m, batch_s);
                for (const auto& p3d : p3ds) {
                    if (p3d.z > PHYSICAL_MIN_DEPTH && p3d.z < PHYSICAL_MAX_DEPTH) {
                        global_sum_z += p3d.z;
                        global_pt_count++;
                        point_cloud.push_back({p3d.x, p3d.y, p3d.z});
                    }
                }
            }

            window_start += WINDOW_SEC;
            global_m_det += dbg_m_det;
            global_s_det += dbg_s_det;
            global_disp_pass += dbg_disp_pass;
            global_epi_pass += dbg_epi_pass;
            global_sum_y_diff += dbg_sum_y_diff;

            double current_global_avg_z = global_pt_count > 0 ? (global_sum_z / global_pt_count) : 0.0;

            if (dbg_m_det > 0 || dbg_s_det > 0) {
                cout << "\33[2K\r[Time: " << std::fixed << std::setprecision(1) << (window_start * 1000.0)
                     << "ms] M:" << global_m_det
                     << " S:" << global_s_det
                     << " Epi:" << global_epi_pass
                     << " Disp:" << global_disp_pass
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

    std::vector<cv::Point3f> left_rays, right_rays;
    event_rectifier.getCameraFrustums(1.5f, left_rays, right_rays);
    for (const auto& pt : left_rays) point_cloud.push_back({pt.x, pt.y, pt.z, 0, 100, 255});
    for (const auto& pt : right_rays) point_cloud.push_back({pt.x, pt.y, pt.z, 255, 50, 50});

    writePLY("laser_scan.ply", point_cloud);

    rclcpp::shutdown();
    return 0;
}