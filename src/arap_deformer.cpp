#include "arap_deformer.h"
#include <igl/cotmatrix.h>
#include <igl/adjacency_list.h>
#include <Eigen/SVD>
#include <cmath>

void ArapDeformer::precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    V_orig_ = V;
    F_ = F;
    n_verts_ = V.rows();

    // 1. Cotangent Laplacian (n x n sparse matrix)
    igl::cotmatrix(V, F, L_);

    // 2. Differential coordinates (n x 3)
    deltas_ = L_ * V;

    // 3. Per-vertex adjacency (1-ring neighbors)
    igl::adjacency_list(F, neighbors_);

    precomputed_ = true;
}

void ArapDeformer::set_constraints(const Eigen::VectorXi& constraint_indices) {
    constraint_indices_ = constraint_indices;
    int num_constraints = constraint_indices_.size();

    // Build augmented matrix A = [L; W] of size (n + |C|) x n
    std::vector<Eigen::Triplet<double>> triplets;

    // Copy L entries
    for (int k = 0; k < L_.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_, k); it; ++it)
            triplets.emplace_back(it.row(), it.col(), it.value());

    // Constraint rows
    for (int k = 0; k < num_constraints; ++k)
        triplets.emplace_back(n_verts_ + k, constraint_indices_[k], constraint_weight_);

    A_.resize(n_verts_ + num_constraints, n_verts_);
    A_.setFromTriplets(triplets.begin(), triplets.end());

    // Prefactor A^T * A
    AtA_ = A_.transpose() * A_;
    solver_.compute(AtA_);
    prefactored_ = (solver_.info() == Eigen::Success);
}

Eigen::MatrixXd ArapDeformer::solve_laplacian(const Eigen::MatrixXd& constraint_positions) {
    if (!prefactored_) return V_orig_;

    int num_constraints = constraint_indices_.size();

    // Build RHS: b = [deltas; W * target_positions]
    Eigen::MatrixXd b(n_verts_ + num_constraints, 3);
    b.topRows(n_verts_) = deltas_;
    for (int k = 0; k < num_constraints; ++k)
        b.row(n_verts_ + k) = constraint_weight_ * constraint_positions.row(k);

    Eigen::MatrixXd Atb = A_.transpose() * b;

    // Solve each coordinate independently
    Eigen::MatrixXd V_prime(n_verts_, 3);
    for (int d = 0; d < 3; ++d)
        V_prime.col(d) = solver_.solve(Atb.col(d));

    return V_prime;
}

Eigen::MatrixXd ArapDeformer::solve_arap(const Eigen::MatrixXd& constraint_positions,
                                          int num_iterations) {
    if (!prefactored_) return V_orig_;

    // Initialize with Laplacian deformation result
    Eigen::MatrixXd V_prime = solve_laplacian(constraint_positions);

    int num_constraints = constraint_indices_.size();
    std::vector<Eigen::Matrix3d> rotations;

    for (int iter = 0; iter < num_iterations; ++iter) {
        // Local step: compute per-vertex rotations from current V'
        compute_rotations(V_prime, rotations);

        // Global step: build rotation-aware RHS and solve
        Eigen::MatrixXd arap_b = build_arap_rhs(rotations);

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

void ArapDeformer::compute_rotations(const Eigen::MatrixXd& V_deformed,
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

        // Reflection fix: flip column of U with smallest singular value
        if (R.determinant() < 0) {
            int min_idx;
            svd.singularValues().minCoeff(&min_idx);
            U.col(min_idx) *= -1;
            R = Vt * U.transpose();
        }
        rotations[i] = R;
    }
}

Eigen::MatrixXd ArapDeformer::build_arap_rhs(const std::vector<Eigen::Matrix3d>& rotations) {
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
