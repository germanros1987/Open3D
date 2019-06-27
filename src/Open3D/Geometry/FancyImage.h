// Anonymous author
//
//-------------------------

#pragma once

#include "Open3D/Geometry/Geometry2D.h"
#include "Open3D/Geometry/Image.h"

namespace open3d {

class FancyImage;


// I will comment this class later...maybe...probably not...
class FancyImage {
public:
    FancyImage() {}
    FancyImage(Image &ultra_violet, Image &depth) : depth_(depth) {}

    ~FancyImage() {
        ultra_violet_.Clear();
    };

     FancyImage* CreateFromUltraAndDepth(
            const Image &ultra_violet,
            const Image &depth,
            double depth_scale = 1000.0,
            double depth_trunc,
            bool convert_ultra_violet_to_intensity = true);

     std::shared_ptr<FancyImage> CreateFromStandardFormat(
            const Image &ultra_violet,
            const Image &depth,
            bool convert_rgb_to_intensity = true);

     std::shared_ptr<FancyImage> CreateFromTUMFormat(
            const Image &ultra_violet,
            const Image &depth,
            bool convert_rgb_to_intensity = true);


public:
    Image depth_;
    Image ultra_violet_;
};

}  // namespace open3d
