#include <iostream>
#include <chrono>

#include <geo/io/IO.h>
#include <geo/field/CotanLaplacian.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " mesh.vtk output.vtk"
                  << std::endl;
        return 0;
    }

    Mesh mesh;
    loadMeshFromVtk(argv[1], mesh);

    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();
    Eigen::SparseMatrix<double> L;
    cotanLaplacian(mesh, L);
    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    std::cout << "Assembled Laplacian in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                     .count()
              << " ms" << std::endl;

    Eigen::MatrixX3d pos(mesh.nV(), 3);
    for (size_t i = 0; i < mesh.nV(); ++i)
        pos.row(i) = mesh.vertices[i].position;

    Eigen::MatrixX3d H = L.selfadjointView<Eigen::Lower>() * pos;

    mesh.require(Mesh::VertexNormals);
    mesh.require(Mesh::VertexAreas);
    Eigen::VectorXd h(mesh.nV());
    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        h(i) = -0.5 * H.row(i).dot(mesh.vertices[i].normal) /
               mesh.vertices[i].area;
    }
    writeMeshVertexScalarFieldToVtk(argv[2], mesh, h);
}
