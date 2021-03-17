//
// Created by cryscan on 3/15/21.
//

#include <cassert>

#include <towr/nlp_formulation.h>
#include <towr/variables/variable_names.h>

#include "path_constraint.h"


using namespace towr;

PathConstraint::PathConstraint(const VecTimes& dts,
                               const std::vector<VectorXd>& path_linear,
                               const std::vector<VectorXd>& path_angular,
                               const SplineHolder& spline_holder)
        : TimeDiscretizationConstraint(dts, "path"),
          path_linear(path_linear),
          path_angular(path_angular) {
    assert(dts.size() == path_linear.size());
    assert(dts.size() == path_angular.size());

    base_linear = spline_holder.base_linear_;
    base_angular = spline_holder.base_angular_;

    double dev = 0.05;
    node_bounds.resize(k6D);
    node_bounds.at(AX) = ifopt::NoBound;
    node_bounds.at(AY) = ifopt::NoBound;
    node_bounds.at(AZ) = ifopt::NoBound;
    node_bounds.at(LX) = Bounds(-dev, dev);
    node_bounds.at(LY) = Bounds(-dev, dev);
    node_bounds.at(LZ) = ifopt::NoBound;

    int n_constraints_per_node = node_bounds.size();
    SetRows(GetNumberOfNodes() * n_constraints_per_node);
}

int PathConstraint::GetRow(int node, int dim) const {
    return node * (int) node_bounds.size() + dim;
}

void PathConstraint::UpdateConstraintAtInstance(double t, int k, VectorXd& g) const {
    g.middleRows(GetRow(k, LX), k3D) = base_linear->GetPoint(t).p() - path_linear.at(k);
    g.middleRows(GetRow(k, AX), k3D) = base_angular->GetPoint(t).p() - path_angular.at(k);
}

void PathConstraint::UpdateBoundsAtInstance(double t, int k, VecBound& b) const {
    for (int dim = 0; dim < node_bounds.size(); ++dim)
        b.at(GetRow(k, dim)) = node_bounds.at(dim);
}

void PathConstraint::UpdateJacobianAtInstance(double t, int k, std::string var_set, Jacobian& jac) const {
    if (var_set == id::base_ang_nodes)
        jac.middleRows(GetRow(k, AX), k3D) = base_angular->GetJacobianWrtNodes(t, kPos);

    if (var_set == id::base_lin_nodes)
        jac.middleRows(GetRow(k, LX), k3D) = base_linear->GetJacobianWrtNodes(t, kPos);
}
