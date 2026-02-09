#pragma once

#include "arap_deformer.h"
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <set>
#include <string>

class ViewerApp {
public:
    // Load mesh and initialize the viewer
    bool load_mesh(const std::string& path);

    // Launch the viewer window (blocks until closed)
    void launch();

    int num_vertices() const { return V_.rows(); }
    int num_faces() const { return F_.rows(); }

private:
    // Mesh data
    Eigen::MatrixXd V_;           // original vertex positions
    Eigen::MatrixXd V_current_;   // current (deformed) vertex positions
    Eigen::MatrixXi F_;           // face indices

    // Viewer
    igl::opengl::glfw::Viewer viewer_;

    // Deformer back-end
    ArapDeformer deformer_;

    // Interaction state (for features 05-07)
    std::set<int> selected_vertices_;
    int dragged_vertex_ = -1;
    bool is_dragging_ = false;
    int interaction_mode_ = 0;   // 0 = Select, 1 = Drag
    int solver_mode_ = 0;        // 0 = Laplacian, 1 = ARAP
    int arap_iterations_ = 5;
    double last_solve_time_ms_ = 0.0;
};
