#include "arap.h"
#include <igl/adjacency_list.h>
#include <Eigen/SVD>
#include <cmath>

void ArapSolver::precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    LaplacianSolver::precompute(V, F);
    igl::adjacency_list(F, neighbors_);
}

Eigen::MatrixXd ArapSolver::solve(const Eigen::MatrixXd& constraint_positions,
                                   int num_iterations) {
    if (!prefactored_) return V_orig_;

    // Initialize with Laplacian result
    Eigen::MatrixXd V_prime = LaplacianSolver::solve(constraint_positions);

    int num_constraints = constraint_indices_.size();
    std::vector<Eigen::Matrix3d> rotations;

    for (int iter = 0; iter < num_iterations; ++iter) {
        compute_rotations(V_prime, rotations);

        Eigen::MatrixXd arap_b = build_rhs(rotations);

        Eigen::MatrixXd rhs(n_verts_ + num_constraints, 3);
        rhs.topRows(n_verts_) = arap_b;
        for (int k = 0; k < num_constraints; ++k)
            rhs.row(n_verts_ + k) = constraint_weight_ * constraint_positions.row(k);

        Eigen::MatrixXd Atb = A_.transpose() * rhs;
        for (int d = 0; d < 3; ++d)
            V_prime.col(d) = solver_.solve(Atb.col(d));
    }

    return V_prime;
}

void ArapSolver::compute_rotations(const Eigen::MatrixXd& V_deformed,
                                    std::vector<Eigen::Matrix3d>& rotations) {
    rotations.resize(n_verts_);
    for (int i = 0; i < n_verts_; ++i) {
        Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
        for (int j : neighbors_[i]) {
            double w = std::abs(L_.coeff(i, j));
            Eigen::Vector3d e_ij  = V_orig_.row(j) - V_orig_.row(i);
            Eigen::Vector3d ep_ij = V_deformed.row(j) - V_deformed.row(i);
            S += w * e_ij * ep_ij.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d Vt = svd.matrixV();
        Eigen::Matrix3d R = Vt * U.transpose();

        if (R.determinant() < 0) {
            int min_idx;
            svd.singularValues().minCoeff(&min_idx);
            U.col(min_idx) *= -1;
            R = Vt * U.transpose();
        }
        rotations[i] = R;
    }
}

Eigen::MatrixXd ArapSolver::build_rhs(const std::vector<Eigen::Matrix3d>& rotations) {
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n_verts_, 3);
    for (int i = 0; i < n_verts_; ++i) {
        for (int j : neighbors_[i]) {
            double w = std::abs(L_.coeff(i, j));
            Eigen::Vector3d e_ij = V_orig_.row(j) - V_orig_.row(i);
            b.row(i) += 0.5 * w * ((rotations[i] + rotations[j]) * e_ij).transpose();
        }
    }
    return b;
}
