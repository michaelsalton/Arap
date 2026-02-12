#pragma once

#include "solver/arap_deformer.h"
#include "ui/control_panel.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
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
    Eigen::MatrixXi E_;           // #E x 2 unique edges (precomputed)

    // Viewer + ImGui
    igl::opengl::glfw::Viewer viewer_;
    igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin_;
    igl::opengl::glfw::imgui::ImGuiMenu imgui_menu_;
    ControlPanel panel_;

    // Deformer back-end
    ArapDeformer deformer_;

    // Interaction state (for features 05-07)
    std::set<int> selected_vertices_;
    int dragged_vertex_ = -1;
    bool is_dragging_ = false;
    int interaction_mode_ = 0;       // 0 = Select, 1 = Drag
    int solver_mode_ = 0;            // 0 = Laplacian, 1 = ARAP
    int selection_element_mode_ = 0; // 0 = Vertex, 1 = Edge, 2 = Face
    int arap_iterations_ = 5;
    double last_solve_time_ms_ = 0.0;

    // Drag state
    float drag_depth_ = 0.0f;  // screen-space depth at drag start

    // Constraint positions (indexed same order as selected_vertices_)
    Eigen::MatrixXd constraint_positions_;

    // Helpers
    void update_overlay();
    void sync_constraints();
    void solve_and_update();
    void reset_mesh();
    void clear_selection();
};
