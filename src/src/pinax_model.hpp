#ifndef PINAX_MODEL_HPP
#define PINAX_MODEL_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

#include <jir_refractive_image_geometry/refracted_pinhole_camera_model.hpp>
#include <jir_refractive_image_geometry_msgs/PlanarRefractionInfo.h>

using namespace std;
using namespace cv;

struct PinaxReprojectionResidual {
    PinaxReprojectionResidual(double p2d_x, double p2d_y, double p3d_x, double p3d_y, double p3d_z, 
                              double n_air, double n_glass, double n_water)
        : p2d_x_(p2d_x), p2d_y_(p2d_y), p3d_x_(p3d_x), p3d_y_(p3d_y), p3d_z_(p3d_z),
          n_air_(n_air), n_glass_(n_glass), n_water_(n_water) {}

    bool operator()(const double* const intrinsics,
                    const double* const glass_plane, 
                    const double* const board_rot,
                    const double* const board_trans,
                    double* residual) const {
        
        double p_board[3] = {p3d_x_, p3d_y_, p3d_z_};
        double p_cam[3];
        ceres::AngleAxisRotatePoint(board_rot, p_board, p_cam);
        p_cam[0] += board_trans[0];
        p_cam[1] += board_trans[1];
        p_cam[2] += board_trans[2];

        Eigen::Matrix<double, 3, 1> xyz_target(p_cam[0], p_cam[1], p_cam[2]);

        jir_refractive_image_geometry_msgs::PlanarRefractionInfo info;
        info.normal[0] = glass_plane[0]; 
        info.normal[1] = glass_plane[1]; 
        info.normal[2] = glass_plane[2];
        info.d_0 = glass_plane[3];  // Distance to glass along the plane normal. If the normal is renormalized, d must be scaled the same way.
        info.d_1 = 0.01;            // Assume 1cm thick glass port
        info.n_glass = n_glass_;
        info.n_water = n_water_;

        jir_refractive_image_geometry::RefractedPinholeCameraModel pinax_cam;
        
        // [CHANGED: ROS 2 specific message format and lowercase 'k']
        sensor_msgs::msg::CameraInfo cam_info;
        cam_info.k[0] = intrinsics[0]; // fx
        cam_info.k[4] = intrinsics[1]; // fy
        cam_info.k[2] = intrinsics[2]; // cx
        cam_info.k[5] = intrinsics[3]; // cy
        
        pinax_cam.fromCameraInfo(cam_info);
        pinax_cam.fromPlanarRefractionInfo(info);

        Eigen::Matrix<double, 2, 1> projected_pixel = pinax_cam.project3dToPixel(xyz_target);

        residual[0] = projected_pixel(0) - p2d_x_;
        residual[1] = projected_pixel(1) - p2d_y_;

        return true;
    }

private:
    double p2d_x_, p2d_y_, p3d_x_, p3d_y_, p3d_z_;
    double n_air_, n_glass_, n_water_;
};

#endif // PINAX_MODEL_HPP
