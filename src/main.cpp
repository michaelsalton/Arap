#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
    std::string mesh_path;
    if (argc > 1) {
        mesh_path = argv[1];
    } else {
        mesh_path = DATA_DIR "/spot.obj";
        std::cout << "Usage: " << argv[0] << " <mesh.(obj|off|ply|stl)>" << std::endl;
        std::cout << "No mesh specified, loading default: " << mesh_path << std::endl;
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    if (!igl::read_triangle_mesh(mesh_path, V, F)) {
        std::cerr << "Error: could not read mesh file: " << mesh_path << std::endl;
        return 1;
    }

    if (V.rows() == 0 || F.rows() == 0) {
        std::cerr << "Warning: mesh is empty (" << V.rows() << " vertices, "
                  << F.rows() << " faces)" << std::endl;
        return 1;
    }

    std::cout << "Loaded mesh: " << V.rows() << " vertices, "
              << F.rows() << " faces" << std::endl;

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().compute_normals();
    viewer.launch();

    return 0;
}
