#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

// Event Camera ROS 2 & Prophesee Decoder Headers
#include <event_camera_msgs/msg/event_packet.hpp>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Custom rectification header
#include "rectification.hpp"

using namespace std;

class TestEventProcessor : public event_camera_codecs::EventProcessor {
public:
    EventRectifier* rectifier;
    bool is_master;

    cv::Mat master_canvas;
    cv::Mat slave_canvas;

    // A fast 1D array for every row to hold our +1 and -1 polarities
    std::vector<int> master_polarities[720];
    std::vector<int> slave_polarities[720];

    TestEventProcessor() {
        master_canvas = cv::Mat::zeros(720, 1280, CV_8UC3);
        slave_canvas = cv::Mat::zeros(720, 1280, CV_8UC3);

        for(int i = 0; i < 720; ++i) {
            master_polarities[i].resize(1280, 0);
            slave_polarities[i].resize(1280, 0);
        }
    }

    inline void eventCD(uint64_t /*sensor_time*/, uint16_t ex, uint16_t ey, uint8_t polarity) override {
        cv::Point2f pt = rectifier->rectify(ex, ey, is_master);

        int x = std::round(pt.x);
        int y = std::round(pt.y);

        if (x >= 0 && x < 1280 && y >= 0 && y < 720) {
            if (is_master) {
                master_polarities[y][x] += (polarity == 1) ? 1 : -1;

                if (polarity == 1) master_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
                else master_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            } else {
                slave_polarities[y][x] += (polarity == 1) ? 1 : -1;

                if (polarity == 1) slave_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
                else slave_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    inline bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return true; }
    inline void finished() override {}
    inline void rawData(const char *, size_t) override {}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    string mcap_file = "/home/ryan/Documents/MIRS/Thesis/3Dreconstruction/datasets/laser7/laser6_0.mcap";
    string master_topic = "/event_cam_0/events";
    string slave_topic = "/event_cam_1/events";

    cout << "Initializing Rectifier from JSON..." << endl;
    EventRectifier event_rectifier("calib_data/meta_int_master.json",
                                   "calib_data/meta_int_slav.json",
                                   "calib_data/meta_extrinsics.json");

    TestEventProcessor processor;
    processor.rectifier = &event_rectifier;

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = mcap_file;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader.open(storage_options, converter_options);

    event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, TestEventProcessor> decoderFactory;
    rclcpp::Serialization<event_camera_msgs::msg::EventPacket> serializer;

    cout << "Reading ROS Bag and visualizing Hybrid Polarity Filter...\n" << endl;
    cout << "==> PRESS [SPACE] TO PAUSE/RESUME <==" << endl;

    cv::namedWindow("Polarity Transition Analyzer", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("Polarity Transition Analyzer", 1920, 540);

    uint64_t last_render_time = 0;
    const uint64_t WINDOW_NS = 2 * 1000000; // 2ms integration window

    while (reader.has_next() && rclcpp::ok()) {
        auto msg = reader.read_next();
        if (msg->topic_name != master_topic && msg->topic_name != slave_topic) continue;

        if (last_render_time == 0) last_render_time = msg->recv_timestamp;

        processor.is_master = (msg->topic_name == master_topic);
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto ros_msg = std::make_shared<event_camera_msgs::msg::EventPacket>();
        serializer.deserialize_message(&serialized_msg, ros_msg.get());

        auto decoder = decoderFactory.getInstance(*ros_msg);
        if (decoder) decoder->decode(*ros_msg, &processor);

        // --- VISUALIZATION TRIGGER ---
        if (msg->recv_timestamp - last_render_time > WINDOW_NS) {

            cv::Mat combined;
            cv::hconcat(processor.master_canvas, processor.slave_canvas, combined);

            // --- THE HYBRID SPATIAL GRADIENT FILTER ---
            const int WINDOW = 5;          // Look 5 pixels left and right
            const int MIN_ON_CLUSTER = 3;  // YOUR IDEA: Guarantee a dense block of leading ON events
            const int MIN_SCORE = 5;       // Minimum overall transition severity

            for (int y = 0; y < 720; ++y) {

                // --- MASTER LOGIC ---
                int best_x_m = -1;
                int max_score_m = 0;

                for (int x = WINDOW; x < 1280 - WINDOW; ++x) {
                    int left_sum = 0, right_sum = 0;
                    for (int i = 1; i <= WINDOW; ++i) {
                        left_sum += processor.master_polarities[y][x - i];
                        right_sum += processor.master_polarities[y][x + i];
                    }

                    // HYBRID CHECK: Ensure the left side has a guaranteed minimum mass of ON events!
                    if (left_sum >= MIN_ON_CLUSTER) {
                        int score = left_sum - right_sum;
                        if (score > max_score_m) {
                            max_score_m = score;
                            best_x_m = x;
                        }
                    }
                }

                if (max_score_m >= MIN_SCORE) {
                    cv::circle(combined, cv::Point(best_x_m, y), 1, cv::Scalar(255, 255, 255), -1);
                }

                // --- SLAVE LOGIC ---
                int best_x_s = -1;
                int max_score_s = 0;

                for (int x = WINDOW; x < 1280 - WINDOW; ++x) {
                    int left_sum = 0, right_sum = 0;
                    for (int i = 1; i <= WINDOW; ++i) {
                        left_sum += processor.slave_polarities[y][x - i];
                        right_sum += processor.slave_polarities[y][x + i];
                    }

                    if (left_sum >= MIN_ON_CLUSTER) {
                        int score = left_sum - right_sum;
                        if (score > max_score_s) {
                            max_score_s = score;
                            best_x_s = x;
                        }
                    }
                }

                if (max_score_s >= MIN_SCORE) {
                    cv::circle(combined, cv::Point(best_x_s + 1280, y), 1, cv::Scalar(255, 255, 255), -1);
                }

                // Wipe arrays clean for the next 2ms window
                std::fill(processor.master_polarities[y].begin(), processor.master_polarities[y].end(), 0);
                std::fill(processor.slave_polarities[y].begin(), processor.slave_polarities[y].end(), 0);
            }

            // Draw Epipolar Grid
            for(int y = 50; y < 720; y += 50) {
                cv::line(combined, cv::Point(0, y), cv::Point(2560, y), cv::Scalar(30, 30, 30), 1);
            }

            cv::putText(combined, "HYBRID GRADIENT FILTER ACTIVE | Press [SPACE] to pause",
                        cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

            cv::Mat display;
            cv::resize(combined, display, cv::Size(), 0.75, 0.75);
            cv::imshow("Polarity Transition Analyzer", display);

            char key = (char)cv::waitKey(1);
            if (key == 'q' || key == 27) break;
            if (key == ' ' || key == 'p') {
                cout << "-> Paused. Press [SPACE] to resume." << endl;
                while (true) {
                    char pause_key = (char)cv::waitKey(0); // Wait indefinitely for input
                    if (pause_key == ' ' || pause_key == 'p') {
                        cout << "-> Resuming..." << endl;
                        break;
                    }
                    if (pause_key == 'q' || pause_key == 27) {
                        cout << "\nVerification aborted." << endl;
                        cv::destroyAllWindows();
                        rclcpp::shutdown();
                        return 0;
                    }
                }
            }

            processor.master_canvas.setTo(cv::Scalar(0,0,0));
            processor.slave_canvas.setTo(cv::Scalar(0,0,0));
            last_render_time = msg->recv_timestamp;
        }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////


// #include <iostream>
// #include <string>
// #include <cmath>
// #include <vector>
// #include <algorithm>

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/serialization.hpp>
// #include <rclcpp/serialized_message.hpp>
// #include <rosbag2_cpp/reader.hpp>
// #include <rosbag2_cpp/readers/sequential_reader.hpp>
// #include <rosbag2_storage/storage_options.hpp>

// // Event Camera ROS 2 & Prophesee Decoder Headers
// #include <event_camera_msgs/msg/event_packet.hpp>
// #include <event_camera_codecs/decoder.h>
// #include <event_camera_codecs/decoder_factory.h>
// #include <event_camera_codecs/event_processor.h>

// // OpenCV
// #include <opencv2/opencv.hpp>

// // Custom rectification header
// #include "rectification.hpp"

// using namespace std;

// class TestEventProcessor : public event_camera_codecs::EventProcessor {
// public:
//     EventRectifier* rectifier;
//     bool is_master;

//     cv::Mat master_canvas;
//     cv::Mat slave_canvas;

//     // Binary masks used specifically for Edge/Line Detection
//     cv::Mat master_mask;
//     cv::Mat slave_mask;

//     TestEventProcessor() {
//         master_canvas = cv::Mat::zeros(720, 1280, CV_8UC3);
//         slave_canvas = cv::Mat::zeros(720, 1280, CV_8UC3);

//         // 8-bit single channel matrices (pure black and white) for the Hough Transform
//         master_mask = cv::Mat::zeros(720, 1280, CV_8UC1);
//         slave_mask = cv::Mat::zeros(720, 1280, CV_8UC1);
//     }

//     inline void eventCD(uint64_t /*sensor_time*/, uint16_t ex, uint16_t ey, uint8_t polarity) override {
//         // We only use ON events (leading edge) to keep the initial geometry as thin as possible
//         if (polarity == 0) return;

//         cv::Point2f pt = rectifier->rectify(ex, ey, is_master);

//         int x = std::round(pt.x);
//         int y = std::round(pt.y);

//         if (x >= 0 && x < 1280 && y >= 0 && y < 720) {
//             if (is_master) {
//                 master_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(200, 200, 200); // Faint white for visual reference
//                 master_mask.at<uchar>(y, x) = 255; // Solid white for the computer vision mask
//             } else {
//                 slave_canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(200, 200, 200);
//                 slave_mask.at<uchar>(y, x) = 255;
//             }
//         }
//     }

//     inline bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return true; }
//     inline void finished() override {}
//     inline void rawData(const char *, size_t) override {}
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);

//     string mcap_file = "/home/ryan/Documents/MIRS/Thesis/3Dreconstruction/laser2/laser2_0.mcap";
//     string master_topic = "/event_cam_0/events";
//     string slave_topic = "/event_cam_1/events";

//     cout << "Initializing Rectifier from JSON..." << endl;
//     EventRectifier event_rectifier("calib_data/meta_int_master.json",
//                                    "calib_data/meta_int_slav.json",
//                                    "calib_data/meta_extrinsics.json");

//     TestEventProcessor processor;
//     processor.rectifier = &event_rectifier;

//     rosbag2_cpp::Reader reader;
//     rosbag2_storage::StorageOptions storage_options;
//     storage_options.uri = mcap_file;
//     storage_options.storage_id = "mcap";

//     rosbag2_cpp::ConverterOptions converter_options;
//     converter_options.input_serialization_format = "cdr";
//     converter_options.output_serialization_format = "cdr";
//     reader.open(storage_options, converter_options);

//     event_camera_codecs::DecoderFactory<event_camera_msgs::msg::EventPacket, TestEventProcessor> decoderFactory;
//     rclcpp::Serialization<event_camera_msgs::msg::EventPacket> serializer;

//     cout << "Reading ROS Bag and extracting Distinct Line Segments...\n" << endl;

//     cv::namedWindow("Event-Based Edge & Line Detection", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
//     cv::resizeWindow("Event-Based Edge & Line Detection", 1920, 540);

//     uint64_t last_render_time = 0;
//     const uint64_t WINDOW_NS = 2 * 1000000; // 2ms integration

//     // Morphological Kernel for manipulating the event clouds
//     cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

//     while (reader.has_next() && rclcpp::ok()) {
//         auto msg = reader.read_next();
//         if (msg->topic_name != master_topic && msg->topic_name != slave_topic) continue;

//         if (last_render_time == 0) last_render_time = msg->recv_timestamp;

//         processor.is_master = (msg->topic_name == master_topic);
//         rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
//         auto ros_msg = std::make_shared<event_camera_msgs::msg::EventPacket>();
//         serializer.deserialize_message(&serialized_msg, ros_msg.get());

//         auto decoder = decoderFactory.getInstance(*ros_msg);
//         if (decoder) decoder->decode(*ros_msg, &processor);

//         if (msg->recv_timestamp - last_render_time > WINDOW_NS) {

//             // 1. MORPHOLOGICAL BRIDGING & THINNING
//             // First, "close" the image to bridge tiny microscopic gaps in the laser line
//             cv::morphologyEx(processor.master_mask, processor.master_mask, cv::MORPH_CLOSE, morph_kernel);
//             cv::morphologyEx(processor.slave_mask, processor.slave_mask, cv::MORPH_CLOSE, morph_kernel);

//             // Then, "erode" the image to thin the thick laser blobs into a 1-pixel skeleton
//             cv::erode(processor.master_mask, processor.master_mask, morph_kernel);
//             cv::erode(processor.slave_mask, processor.slave_mask, morph_kernel);

//             // 2. PROBABILISTIC HOUGH TRANSFORM (Line Segment Extraction)
//             std::vector<cv::Vec4i> master_lines, slave_lines;

//             // Parameters: (Image, Lines Out, Rho Res, Theta Res, Minimum Intersections, Min Line Length, Max Line Gap)
//             int Rho_res = 3;        // Thickness of line in pixels
//             int threshold = 10;      // Lowered: Catch fainter lines with fewer total events
//             int minLineLength = 10;  // Lowered: Allow slightly shorter physical segments to survive
//             int maxLineGap = 15;     // Massive increase: Allow the algorithm to jump over 25-pixel empty gaps!

//             cv::HoughLinesP(processor.master_mask, master_lines, Rho_res, CV_PI/180, threshold, minLineLength, maxLineGap);
//             cv::HoughLinesP(processor.slave_mask, slave_lines, Rho_res, CV_PI/180, threshold, minLineLength, maxLineGap);

//             // 3. VISUALIZATION
//             cv::Mat combined;
//             cv::hconcat(processor.master_canvas, processor.slave_canvas, combined);

//             // Draw the extracted vector segments as distinct, brightly colored geometric lines
//             for (size_t i = 0; i < master_lines.size(); i++) {
//                 cv::Vec4i l = master_lines[i];
//                 // Use a distinct color for every segment to prove it identifies breaks in depth!
//                 cv::Scalar color(50 * (i % 5) + 50, 255 - (30 * (i % 8)), 255);
//                 cv::line(combined, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 3, cv::LINE_AA);
//             }

//             for (size_t i = 0; i < slave_lines.size(); i++) {
//                 cv::Vec4i l = slave_lines[i];
//                 cv::Scalar color(255, 255 - (30 * (i % 8)), 50 * (i % 5) + 50);
//                 cv::line(combined, cv::Point(l[0] + 1280, l[1]), cv::Point(l[2] + 1280, l[3]), color, 3, cv::LINE_AA);
//             }

//             // Draw Epipolar Grid
//             for(int y = 50; y < 720; y += 50) {
//                 cv::line(combined, cv::Point(0, y), cv::Point(2560, y), cv::Scalar(30, 30, 30), 1);
//             }

//             cv::putText(combined, "PROBABILISTIC HOUGH TRANSFORM | Press [SPACE] to pause",
//                         cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

//             cv::Mat display;
//             cv::resize(combined, display, cv::Size(), 0.75, 0.75);
//             cv::imshow("Event-Based Edge & Line Detection", display);

//             char key = (char)cv::waitKey(1);
//             if (key == 'q' || key == 27) break;
//             if (key == ' ' || key == 'p') {
//                 while (true) {
//                     char pause_key = (char)cv::waitKey(0);
//                     if (pause_key == ' ' || pause_key == 'p' || pause_key == 'q' || pause_key == 27) break;
//                 }
//             }

//             // Wipe arrays clean for the next 2ms window
//             processor.master_canvas.setTo(cv::Scalar(0,0,0));
//             processor.slave_canvas.setTo(cv::Scalar(0,0,0));
//             processor.master_mask.setTo(cv::Scalar(0));
//             processor.slave_mask.setTo(cv::Scalar(0));
//             last_render_time = msg->recv_timestamp;
//         }
//     }

//     cv::destroyAllWindows();
//     rclcpp::shutdown();
//     return 0;
// }