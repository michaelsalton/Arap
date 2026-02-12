#pragma once

#include <imgui.h>
#include <functional>

struct ControlPanel {
    // State (owned by ViewerApp, bound via pointers)
    int* interaction_mode = nullptr;
    int* solver_mode = nullptr;
    int* selection_element_mode = nullptr;
    int* arap_iterations = nullptr;
    int num_vertices = 0;
    int num_faces = 0;
    int num_constraints = 0;
    double last_solve_time_ms = 0.0;

    // Actions (set by ViewerApp)
    std::function<void()> on_reset_mesh;
    std::function<void()> on_clear_selection;

    void draw();
};
