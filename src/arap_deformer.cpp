#include "arap_deformer.h"

void ArapDeformer::precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    // TODO: Feature 02
    V_orig_ = V;
    F_ = F;
    n_verts_ = V.rows();
    precomputed_ = true;
}

void ArapDeformer::set_constraints(const Eigen::VectorXi& constraint_indices) {
    // TODO: Feature 03
    constraint_indices_ = constraint_indices;
    prefactored_ = false;
}

Eigen::MatrixXd ArapDeformer::solve_laplacian(const Eigen::MatrixXd& constraint_positions) {
    // TODO: Feature 03
    return V_orig_;
}

Eigen::MatrixXd ArapDeformer::solve_arap(const Eigen::MatrixXd& constraint_positions,
                                          int num_iterations) {
    // TODO: Feature 04
    return V_orig_;
}

void ArapDeformer::compute_rotations(const Eigen::MatrixXd& V_deformed,
                                      std::vector<Eigen::Matrix3d>& rotations) {
    // TODO: Feature 04
}

Eigen::MatrixXd ArapDeformer::build_arap_rhs(const std::vector<Eigen::Matrix3d>& rotations) {
    // TODO: Feature 04
    return deltas_;
}
