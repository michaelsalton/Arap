#pragma once

#include "solver/arap.h"
#include "ui/control_panel.h"
#include "app/input_handler.h"
#include "app/overlay_renderer.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <Eigen/Core>
#include <set>
#include <string>

class ViewerApp {
public:
    bool load_mesh(const std::string& path);
    void launch();

    int num_vertices() const { return V_.rows(); }
    int num_faces() const { return F_.rows(); }

private:
    // Mesh data
    Eigen::MatrixXd V_;
    Eigen::MatrixXd V_current_;
    Eigen::MatrixXi F_;
    Eigen::MatrixXi E_;

    // Viewer + ImGui
    igl::opengl::glfw::Viewer viewer_;
    igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin_;
    igl::opengl::glfw::imgui::ImGuiMenu imgui_menu_;
    ControlPanel panel_;

    // Subsystems
    ArapSolver deformer_;
    InputHandler input_;
    OverlayRenderer overlay_;

    // Interaction state
    std::set<int> selected_vertices_;
    int interaction_mode_ = 0;
    int solver_mode_ = 0;
    int selection_element_mode_ = 0;
    int arap_iterations_ = 5;
    double last_solve_time_ms_ = 0.0;

    // Constraint positions (indexed same order as selected_vertices_)
    Eigen::MatrixXd constraint_positions_;

    // Helpers
    void update_overlay();
    void sync_constraints();
    void solve_and_update();
    void reset_mesh();
    void clear_selection();
};
