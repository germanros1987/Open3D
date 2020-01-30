
#include "Open3D/Geometry/PentaMesh.h"
#include "Open3D/Geometry/BoundingVolume.h"
#include "Open3D/Geometry/PointCloud.h"
#include "Open3D/Geometry/TriangleMesh.h"

#include <Eigen/Dense>
#include <array>
#include <numeric>
#include <tuple>

#include "Open3D/Utility/Console.h"

namespace open3d {
namespace geometry {

PentaMesh &PentaMesh::Clear() {
    MeshBase::Clear();
    pentas_.clear();
    return *this;
}

PentaMesh &PentaMesh::operator+=(const PentaMesh &mesh) {
    typedef decltype(pentas_)::value_type Vector5i;
    if (mesh.IsEmpty()) return (*this);
    size_t old_vert_num = vertices_.size();
    size_t old_penta_num = pentas_.size();
    size_t add_penta_num = mesh.pentas_.size();
    MeshBase::operator+=(mesh);
    pentas_.resize(pentas_.size() + mesh.pentas_.size());
    Vector5i index_shift = Vector5i::Constant((int64_t)old_vert_num);
    for (size_t i = 0; i < add_penta_num; i++) {
        pentas_[old_penta_num + i] = mesh.pentas_[i] + index_shift;
    }
    return (*this);
}

PentaMesh PentaMesh::operator+(PentaMesh mesh) {
    return (PentaMesh(*this) += mesh);
}

PentaMesh &PentaMesh::RemoveDuplicatedVertices() {
    typedef decltype(pentas_)::value_type::Scalar Index;
    typedef std::tuple<double, double, double> Coordinate3;
    std::unordered_map<Coordinate3, size_t,
                       utility::hash_tuple::hash<Coordinate3>>
            point_to_old_index;
    std::vector<Index> index_old_to_new(vertices_.size());
    size_t old_vertex_num = vertices_.size();
    size_t k = 0;                                  // new index
    for (size_t i = 0; i < old_vertex_num; i++) {  // old index
        Coordinate3 coord = std::make_tuple(vertices_[i](0), vertices_[i](1),
                                            vertices_[i](2));
        if (point_to_old_index.find(coord) == point_to_old_index.end()) {
            point_to_old_index[coord] = i;
            vertices_[k] = vertices_[i];
            index_old_to_new[i] = (Index)k;
            k++;
        } else {
            index_old_to_new[i] = index_old_to_new[point_to_old_index[coord]];
        }
    }
    vertices_.resize(k);
    if (k < old_vertex_num) {
        for (auto &penta : pentas_) {
            penta(0) = index_old_to_new[penta(0)];
            penta(1) = index_old_to_new[penta(1)];
            penta(2) = index_old_to_new[penta(2)];
            penta(3) = index_old_to_new[penta(3)];
            penta(4) = index_old_to_new[penta(4)];

        }
    }
    utility::LogDebug(
            "[RemoveDuplicatedVertices] {:d} vertices have been removed.",
            (int)(old_vertex_num - k));

    return this;
}

PentaMesh &PentaMesh::RemoveDuplicatedTetras() {
    typedef decltype(pentas_)::value_type::Scalar Index;
    typedef std::tuple<Index, Index, Index, Index, Index> Index5;
    std::unordered_map<Index5, size_t, utility::hash_tuple::hash<Index5>>
            penta_to_old_index;
    size_t old_tetra_num = pentas_.size();
    size_t k = 0;
    for (size_t i = 0; i < old_tetra_num; i++) {
        Index5 index;
        std::array<Index, 5> p{pentas_[i](0), pentas_[i](1), pentas_[i](2),
                               pentas_[i](3), pentas_[i](4)};

        std::sort(p.begin(), p.end());
        index = std::make_tuple(p[0], p[1], p[2], p[3], p[4]);

        if (penta_to_old_index.find(index) == penta_to_old_index.end()) {
            penta_to_old_index[index] = i;
            pentas_[k] = pentas_[i];
            k++;
        }
    }
    pentas_.resize(k);
    utility::LogDebug("[RemoveDuplicatedPentas] {:d} pentas have been removed.",
                      (int)(old_tetra_num - k));

    return *this;
}


}  // namespace geometry
}  // namespace open3d
