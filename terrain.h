//
// Created by cryscan on 3/17/21.
//

#ifndef PLAYGROUND_TERRAIN_H
#define PLAYGROUND_TERRAIN_H

#include <towr/terrain/height_map.h>


class Terrain : public towr::HeightMap {
public:
    using Vector2i = Eigen::Vector2i;
    using Vector2d = Eigen::Vector2d;
    using MatrixXd = Eigen::MatrixXd;
    using Dim3D = towr::Dim3D;

    Terrain(Vector3d origin, int x, int y, double unit_size);

    void SetHeight(int x, int y, double height);

    [[nodiscard]] double GetHeight(double x, double y) const override;
    [[nodiscard]] double GetHeightDerivWrtX(double x, double y) const override;
    [[nodiscard]] double GetHeightDerivWrtY(double x, double y) const override;

private:
    Vector3d origin;
    MatrixXd data;
    double unit_size;

    /**
     * @brief performs barycentric interpolation given the three vertices of a triangle and a planar coordinate.
     * @param p1 The first vertex of the triangle.
     * @param p2 The second vertex of the triangle.
     * @param p3 The third vertex of the triangle.
     * @param pos The planar position for which to solve.
     * @param deriv If Z, calculate value, else derivative.
     * @return The z axis value or derivative at the given position.
     */
    static double BarycentricInterpolation(Vector3d p1,
                                           Vector3d p2,
                                           Vector3d p3,
                                           Vector2d pos,
                                           Dim3D deriv = Dim3D::Z);

    [[nodiscard]] double GetValue(double x, double y, Dim3D deriv = Dim3D::Z) const;
};

#endif //PLAYGROUND_TERRAIN_H
