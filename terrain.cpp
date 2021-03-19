//
// Created by cryscan on 3/17/21.
//

#include "terrain.h"


using SharedLock = std::shared_lock<std::shared_mutex>;
using UniqueLock = std::unique_lock<std::shared_mutex>;

Terrain::Terrain(Vector3d pos, uint x, uint y, double unit_size)
        : pos(std::move(pos)),
          size(x, y),
          unit_size(unit_size),
          data(x + 1, y + 1) {
    friction_coeff_ = 1.0;
}

void Terrain::SetHeight(uint x, uint y, double height) {
    auto lock = UniqueLock(mutex);
    data(x, y) = height;
}

double Terrain::GetHeight(double x, double y) const {
    return GetValue(x, y);
}

double Terrain::GetHeightDerivWrtX(double x, double y) const {
    return GetValue(x, y, Dim3D::X);
}

double Terrain::GetHeightDerivWrtY(double x, double y) const {
    return GetValue(x, y, Dim3D::Y);
}

double Terrain::BarycentricInterpolation(Vector3d p1, Vector3d p2, Vector3d p3, Vector2d pos, Dim3D deriv) {
    using Dim3D::X;
    using Dim3D::Y;
    using Dim3D::Z;

    // transform pos to the frame whose origin is p3
    pos.x() -= p3.x();
    pos.y() -= p3.y();

    auto v31 = p1 - p3, v32 = p2 - p3;
    auto det = v31.x() * v32.y() - v32.x() * v31.y();

    Vector3d p;
    p.x() = (pos.x() * v32.y() - v32.x() * pos.y()) / det;
    p.y() = (v31.x() * pos.y() - pos.x() * v31.y()) / det;
    p.z() = 1.0 - p.x() - p.y();

    auto d_h_px = v31.z(), d_h_py = v32.z();
    double d_px_pos, d_py_pos;

    switch (deriv) {
        case X:
            d_px_pos = v32.y() / det;
            d_py_pos = -v31.y() / det;
            break;
        case Y:
            d_px_pos = -v32.x() / det;
            d_py_pos = v31.x() / det;
            break;
        case Z:
            return p.x() * p1.z() + p.y() * p2.z() + p.z() * p3.z();
    }

    return d_h_px * d_px_pos + d_h_py * d_py_pos;
}

double Terrain::GetValue(double x, double y, Dim3D deriv) const {
    auto lock = SharedLock(mutex);

    x -= pos.x();
    y -= pos.y();

    auto id_x = (int) (x / unit_size), id_y = (int) (y / unit_size);
    if (x < 0 || x >= size.x() || y < 0 || y >= size.y()) return 0;

    Vector2d p;
    p.x() = (x - id_x * unit_size) / unit_size;
    p.y() = (y - id_y * unit_size) / unit_size;

    Vector3d p1, p2, p3;
    if (x + y < 1) {
        p1 << 1, 0, data(id_x + 1, id_y);
        p2 << 0, 1, data(id_x, id_y + 1);
        p3 << 0, 0, data(id_x, id_y);
    } else {
        p1 << 0, 1, data(id_x, id_y + 1);
        p2 << 1, 0, data(id_x + 1, id_y);
        p3 << 1, 1, data(id_x + 1, id_y + 1);
    }

    auto result = BarycentricInterpolation(p1, p2, p3, p, deriv);
    if (deriv != Dim3D::Z) result /= unit_size;
    return result;
}