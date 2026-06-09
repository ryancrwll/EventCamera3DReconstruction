#ifndef MOCK_PLANAR_REFRACTION_INFO_H
#define MOCK_PLANAR_REFRACTION_INFO_H

#include <string>
#include <memory>

namespace jir_refractive_image_geometry_msgs {
    struct PlanarRefractionInfo {
        struct { std::string frame_id; } header;
        double normal[3];
        double d_0;     // Distance from camera to glass
        double d_1;     // Thickness of the glass
        double n_glass; // Refractive index of glass
        double n_water; // Refractive index of water
    };
    typedef std::shared_ptr<const PlanarRefractionInfo> PlanarRefractionInfoConstPtr;
}

#endif
