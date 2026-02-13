#include "app/viewer_app.h"
#include "types/modes.h"
#include <igl/read_triangle_mesh.h>
#include <igl/edges.h>
#include <iostream>
#include <chrono>

bool ViewerApp::load_mesh(const std::string& path) {
    if (!igl::read_triangle_mesh(path, V_, F_)) {
        std::cerr << "Error: could not read mesh file: " << path << std::endl;
        return false;
    }

    if (V_.rows() == 0 || F_.rows() == 0) {
        std::cerr << "Warning: mesh is empty (" << V_.rows() << " vertices, "
                  << F_.rows() << " faces)" << std::endl;
        return false;
    }

    V_current_ = V_;

    std::cout << "Loaded mesh: " << V_.rows() << " vertices, "
              << F_.rows() << " faces" << std::endl;

    deformer_.precompute(V_, F_);
    igl::edges(F_, E_);

    return true;
}

void ViewerApp::launch() {
    viewer_.core().background_color = Eigen::Vector4f(0.9f, 0.9f, 0.9f, 1.0f);
    viewer_.data().set_mesh(V_current_, F_);
    viewer_.data().compute_normals();
    viewer_.data().point_size = 10.0f;
    viewer_.data().line_width = 3.0f;
    viewer_.data().show_overlay_depth = true;

    // --- ImGui setup ---
    viewer_.plugins.push_back(&imgui_plugin_);
    imgui_plugin_.widgets.push_back(&imgui_menu_);
    imgui_menu_.callback_draw_viewer_window = []() {};

    panel_.interaction_mode = &interaction_mode_;
    panel_.solver_mode = &solver_mode_;
    panel_.arap_iterations = &arap_iterations_;
    panel_.selection_element_mode = &selection_element_mode_;
    panel_.on_reset_mesh = [this]() { reset_mesh(); };
    panel_.on_clear_selection = [this]() { clear_selection(); };

    imgui_menu_.callback_draw_custom_window = [this]() {
        int prev_sel_mode = selection_element_mode_;
        panel_.num_vertices = V_.rows();
        panel_.num_faces = F_.rows();
        panel_.num_constraints = selected_vertices_.size();
        panel_.last_solve_time_ms = last_solve_time_ms_;
        panel_.draw();
        if (selection_element_mode_ != prev_sel_mode)
            update_overlay();
    };

    // --- Wire up input handler ---
    input_.V_current = &V_current_;
    input_.F = &F_;
    input_.E = &E_;
    input_.selected_vertices = &selected_vertices_;
    input_.constraint_positions = &constraint_positions_;
    input_.interaction_mode = &interaction_mode_;
    input_.solver_mode = &solver_mode_;
    input_.selection_element_mode = &selection_element_mode_;
    input_.arap_iterations = &arap_iterations_;

    input_.on_selection_changed = [this]() {
        sync_constraints();
        update_overlay();
    };
    input_.on_drag_moved = [this]() {
        solve_and_update();
    };
    input_.on_overlay_needs_update = [this]() {
        update_overlay();
    };

    input_.bind(viewer_);

    // Override key handler to include R shortcut (needs reset_mesh)
    viewer_.callback_key_pressed =
        [this](igl::opengl::glfw::Viewer& v, unsigned int key, int mod) -> bool {
        if (key == 'r' || key == 'R') {
            reset_mesh();
            return true;
        }
        return input_.on_key_pressed(v, key, mod);
    };

    update_overlay();
    viewer_.launch();
}

void ViewerApp::update_overlay() {
    overlay_.update(viewer_.data(), V_current_, F_, E_,
                    selected_vertices_, selection_element_mode_,
                    input_.dragged_vertex);
}

void ViewerApp::sync_constraints() {
    if (!deformer_.is_precomputed()) return;

    int n = selected_vertices_.size();
    Eigen::VectorXi indices(n);
    constraint_positions_.resize(n, 3);

    int k = 0;
    for (int vid : selected_vertices_) {
        indices(k) = vid;
        constraint_positions_.row(k) = V_current_.row(vid);
        ++k;
    }

    deformer_.set_constraints(indices);
}

void ViewerApp::reset_mesh() {
    input_.is_dragging = false;
    input_.dragged_vertex = -1;
    V_current_ = V_;
    viewer_.data().set_vertices(V_current_);
    viewer_.data().compute_normals();
    int k = 0;
    for (int vid : selected_vertices_)
        constraint_positions_.row(k++) = V_.row(vid);
    update_overlay();
}

void ViewerApp::clear_selection() {
    input_.is_dragging = false;
    input_.dragged_vertex = -1;
    selected_vertices_.clear();
    constraint_positions_.resize(0, 3);
    sync_constraints();
    update_overlay();
}

void ViewerApp::solve_and_update() {
    if (!deformer_.is_prefactored()) return;

    auto t0 = std::chrono::high_resolution_clock::now();

    if (solver_mode_ == ARAP)
        V_current_ = deformer_.solve(constraint_positions_, arap_iterations_);
    else
        V_current_ = deformer_.solve(constraint_positions_);

    auto t1 = std::chrono::high_resolution_clock::now();
    last_solve_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();

    viewer_.data().set_vertices(V_current_);
    viewer_.data().compute_normals();
    update_overlay();
}
