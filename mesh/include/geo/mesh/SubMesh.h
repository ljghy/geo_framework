#include <geo/mesh/Mesh.h>

void getSubMesh(const Mesh &mesh, const std::vector<size_t> &selectedFaces,
                Mesh &subMesh, std::vector<int> *vertexIndicesMapping = nullptr,
                std::vector<int> *faceIndicesMapping = nullptr);
