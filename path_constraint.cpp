//
// Created by cryscan on 3/15/21.
//

#include <iostream>
#include <cassert>
#include <bitset>

#include <towr/nlp_formulation.h>
#include <towr/variables/variable_names.h>

#include "path_constraint.h"
#include "utils.h"


using namespace towr;

PathConstraint::PathConstraint(const VecTimes& dts,
                               std::vector<VectorXd> path_linear,
                               std::vector<VectorXd> path_angular,
                               std::vector<uint8_t> path_bounds,
                               const SplineHolder& spline_holder)
        : TimeDiscretizationConstraint(dts, "path"),
          path_linear(std::move(path_linear)),
          path_angular(std::move(path_angular)),
          path_bounds(std::move(path_bounds)) {
    assert(dts.size() == path_linear.size());
    assert(dts.size() == path_angular.size());

    base_linear = spline_holder.base_linear_;
    base_angular = spline_holder.base_angular_;

    double dev_rad = 0.05;
    double dev = 0.1;
    node_bounds.resize(k6D);
    node_bounds.at(AX) = Bounds(-dev_rad, dev_rad);
    node_bounds.at(AY) = Bounds(-dev_rad, dev_rad);
    node_bounds.at(AZ) = Bounds(-dev_rad, dev_rad);
    node_bounds.at(LX) = Bounds(-dev, dev);
    node_bounds.at(LY) = Bounds(-dev, dev);
    node_bounds.at(LZ) = Bounds(-dev, dev);

    SetRows(GetNumberOfNodes() * k6D);
}

int PathConstraint::GetRow(int node, int dim) {
    return node * k6D + dim;
}

void PathConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const {
    g.middleRows(GetRow(k, LX), k3D) = base_linear->GetPoint(t).p() - path_linear.at(k);
    g.middleRows(GetRow(k, AX), k3D) = base_angular->GetPoint(t).p() - path_angular.at(k);
}

void PathConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& b) const {
    for (auto dim : {AX, AY, AZ, LX, LY, LZ})
        b.at(GetRow(k, dim)) = ifopt::NoBound;

    auto bounds = convert_bound_dims(path_bounds.at(k));
    for (auto dim : bounds)
        b.at(GetRow(k, dim)) = node_bounds.at(dim);
}

void PathConstraint::UpdateJacobianAtInstance(double t, int k, std::string var_set, Jacobian& jac) const {
    if (var_set == id::base_ang_nodes)
        jac.middleRows(GetRow(k, AX), k3D) = base_angular->GetJacobianWrtNodes(t, kPos);

    if (var_set == id::base_lin_nodes)
        jac.middleRows(GetRow(k, LX), k3D) = base_linear->GetJacobianWrtNodes(t, kPos);
}
