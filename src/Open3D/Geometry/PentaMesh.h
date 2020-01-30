
#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <memory>
#include <vector>

#include "Open3D/Geometry/MeshBase.h"
#include "Open3D/Utility/Eigen.h"
#include "Open3D/Utility/Helper.h"

namespace open3d {
namespace geometry {

class PointCloud;
class TriangleMesh;

class PentaMesh : public MeshBase {
public:
    PentaMesh() : MeshBase(Geometry::GeometryType::PentaMesh) {}
    PentaMesh(const std::vector<Eigen::Vector3d> &vertices,
              const std::vector<Eigen::Vector5i, utility::Vector5i_allocator>
                      &pentas)
        : MeshBase(Geometry::GeometryType::PentaMesh, vertices),
          pentas_(pentas) {}
    ~PentaMesh() override {}

public:
    PentaMesh &Clear() override;

public:
    PentaMesh &operator+=(PentaMesh mesh);
    PentaMesh operator+(PentaMesh mesh);

    PentaMesh &RemoveDuplicatedVertices();

    PentaMesh &RemoveDuplicatedTetras();

    PentaMesh &RemoveUnreferencedVertices();


    bool HasPentas() const {
        return vertices_.size() > 0 & pentas_.size() > 0;
    }

    std::shared_ptr<TriangleMesh> ExtractTriangleMesh(
            const std::vector<double> &values, double level);


    static std::tuple<std::shared_ptr<PentaMesh>, std::vector<size_t>>
    CreateFromPointCloud(const PointCloud &point_cloud);

protected:
    PentaMesh(Geometry::GeometryType type) : MeshBase(type) {}

public:
    std::vector<Eigen::Vector5i, utility::Vector5i_allocator> pentas_;
};

}  // namespace geometry
}  // namespace open3d
