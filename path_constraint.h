//
// Created by cryscan on 3/15/21.
//

#ifndef PLAYGROUND_PATH_CONSTRAINT_H
#define PLAYGROUND_PATH_CONSTRAINT_H

#include <towr/constraints/time_discretization_constraint.h>


class PathConstraint : public towr::TimeDiscretizationConstraint {
public:
    PathConstraint(const VecTimes& dts,
                   std::vector<VectorXd> path_linear,
                   std::vector<VectorXd> path_angular,
                   std::vector<uint8_t> path_bounds,
                   const towr::SplineHolder& spline_holder);

private:
    std::vector<VectorXd> path_linear;
    std::vector<VectorXd> path_angular;
    std::vector<uint8_t> path_bounds;

    VecBound node_bounds;

    towr::NodeSpline::Ptr base_linear;
    towr::NodeSpline::Ptr base_angular;

    void UpdateConstraintAtInstance(double t, int k, VectorXd&) const override;
    void UpdateBoundsAtInstance(double t, int k, VecBound&) const override;
    void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;

    [[nodiscard]] static int GetRow(int node, int dim);
};

#endif //PLAYGROUND_PATH_CONSTRAINT_H
