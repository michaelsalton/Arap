#include "viewer_app.h"
#include <igl/read_triangle_mesh.h>
#include <iostream>

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

    // Precompute deformer data
    deformer_.precompute(V_, F_);

    return true;
}

void ViewerApp::launch() {
    viewer_.data().set_mesh(V_current_, F_);
    viewer_.data().compute_normals();
    viewer_.launch();
}
