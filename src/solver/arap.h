#pragma once

#include "laplacian.h"
#include <vector>

class ArapSolver : public LaplacianSolver {
public:
    using LaplacianSolver::solve;

    void precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    Eigen::MatrixXd solve(const Eigen::MatrixXd& constraint_positions,
                          int num_iterations);

private:
    void compute_rotations(const Eigen::MatrixXd& V_deformed,
                           std::vector<Eigen::Matrix3d>& rotations);
    Eigen::MatrixXd build_rhs(const std::vector<Eigen::Matrix3d>& rotations);

    std::vector<std::vector<int>> neighbors_;
};
