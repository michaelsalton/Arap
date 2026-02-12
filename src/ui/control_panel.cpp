#include "control_panel.h"

void ControlPanel::draw() {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("ARAP Deformer");

    ImGui::Text("Mode");
    ImGui::RadioButton("Select", interaction_mode, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Drag", interaction_mode, 1);

    ImGui::Spacing();
    ImGui::Text("Selection");
    ImGui::RadioButton("Vertex", selection_element_mode, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Edge", selection_element_mode, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Face", selection_element_mode, 2);

    ImGui::Spacing();
    ImGui::Text("Solver");
    ImGui::RadioButton("Laplacian", solver_mode, 0);
    ImGui::SameLine();
    ImGui::RadioButton("ARAP", solver_mode, 1);

    ImGui::Spacing();
    ImGui::SliderInt("ARAP Iterations", arap_iterations, 1, 20);

    ImGui::Spacing();
    if (ImGui::Button("Reset Mesh") && on_reset_mesh)
        on_reset_mesh();
    ImGui::SameLine();
    if (ImGui::Button("Clear Selection") && on_clear_selection)
        on_clear_selection();

    ImGui::Separator();
    ImGui::Text("Vertices: %d", num_vertices);
    ImGui::Text("Faces: %d", num_faces);
    ImGui::Text("Constraints: %d", num_constraints);
    ImGui::Text("Last solve: %.1f ms", last_solve_time_ms);

    ImGui::End();
}
