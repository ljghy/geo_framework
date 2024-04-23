#include <geo/mesh/Mesh.h>
#include <geo/mesh/IndexBasedHalfEdge.h>

void cutEdge(Mesh &mesh, IndexBasedHalfEdgeStructure &heStruct, int heIndex,
             double t, int *updatedFaces)
{

#define HE(i) heStruct.halfEdges[i]

    int heTwinIndex = heStruct.halfEdges[heIndex]->twin;

    auto *he = HE(heIndex).get();
    auto *heTwin = HE(heTwinIndex).get();

    int f0 = he->face;
    int f1 = heTwin->face;

    if (f0 == -1)
    {
        cutEdge(mesh, heStruct, heTwinIndex, 1.0 - t, updatedFaces);
        return;
    }

    int v0 = he->origin;
    int v1 = heTwin->origin;

    int nHe = heStruct.halfEdges.size();
    int nF = mesh.nF();

    Eigen::Vector3d newPos =
        (1.0 - t) * mesh.V(v0).position + t * mesh.V(v1).position;

    int newVertIndex = mesh.vertices.size();
    mesh.vertices.emplace_back(newPos);
    heStruct.vertHalfEdge.push_back(nHe);

    if (f1 != -1)
    {
        IndexBasedHalfEdge *newHe[6];
        for (int i = 0; i < 6; ++i)
            newHe[i] = heStruct.halfEdges
                           .emplace_back(std::make_unique<IndexBasedHalfEdge>())
                           .get();

        newHe[0]->origin = newVertIndex;
        newHe[0]->twin = heTwinIndex;
        newHe[0]->prev = nHe + 1;
        newHe[0]->next = he->next;
        newHe[0]->face = nF;

        newHe[1]->origin = HE(he->prev)->origin;
        newHe[1]->twin = nHe + 2;
        newHe[1]->prev = he->next;
        newHe[1]->next = nHe;
        newHe[1]->face = nF;

        newHe[2]->origin = newVertIndex;
        newHe[2]->twin = nHe + 1;
        newHe[2]->prev = heIndex;
        newHe[2]->next = he->prev;
        newHe[2]->face = f0;

        HE(he->next)->prev = nHe;
        HE(he->next)->next = nHe + 1;
        HE(he->next)->face = nF;
        he->next = nHe + 2;
        he->twin = nHe + 3;
        HE(he->prev)->prev = nHe + 2;

        newHe[3]->origin = newVertIndex;
        newHe[3]->twin = heIndex;
        newHe[3]->prev = nHe + 4;
        newHe[3]->next = heTwin->next;
        newHe[3]->face = nF + 1;

        newHe[4]->origin = HE(heTwin->prev)->origin;
        newHe[4]->twin = nHe + 5;
        newHe[4]->prev = heTwin->next;
        newHe[4]->next = nHe + 3;
        newHe[4]->face = nF + 1;

        newHe[5]->origin = newVertIndex;
        newHe[5]->twin = nHe + 4;
        newHe[5]->prev = heTwinIndex;
        newHe[5]->next = heTwin->prev;
        newHe[5]->face = f1;

        HE(heTwin->next)->prev = nHe + 3;
        HE(heTwin->next)->next = nHe + 4;
        HE(heTwin->next)->face = nF + 1;
        heTwin->next = nHe + 5;
        heTwin->twin = nHe;
        HE(heTwin->prev)->prev = nHe + 5;

        int i = v1 == mesh.F(f0, 0) ? 0 : v1 == mesh.F(f0, 1) ? 1 : 2;
        mesh.F(f0, i) = newVertIndex;
        if (i == 0)
            heStruct.faceHalfEdge[f0] = nHe + 2;

        i = v0 == mesh.F(f1, 0) ? 0 : v0 == mesh.F(f1, 1) ? 1 : 2;
        mesh.F(f1, i) = newVertIndex;
        if (i == 0)
            heStruct.faceHalfEdge[f1] = nHe + 5;

        mesh.faces.emplace_back(
            Eigen::Vector3i(newVertIndex, v1, HE(nHe + 1)->origin));
        heStruct.faceHalfEdge.push_back(nHe);
        mesh.faces.emplace_back(
            Eigen::Vector3i(newVertIndex, v0, HE(nHe + 4)->origin));
        heStruct.faceHalfEdge.push_back(nHe + 3);

        if (updatedFaces)
        {
            updatedFaces[0] = f0;
            updatedFaces[1] = f1;
            updatedFaces[2] = nF;
            updatedFaces[3] = nF + 1;
        }
    }
    else // f1 == -1
    {
        IndexBasedHalfEdge *newHe[4];
        for (int i = 0; i < 4; ++i)
            newHe[i] = heStruct.halfEdges
                           .emplace_back(std::make_unique<IndexBasedHalfEdge>())
                           .get();

        newHe[0]->origin = newVertIndex;
        newHe[0]->twin = heTwinIndex;
        newHe[0]->prev = nHe + 1;
        newHe[0]->next = he->next;
        newHe[0]->face = nF;

        newHe[1]->origin = HE(he->prev)->origin;
        newHe[1]->twin = nHe + 2;
        newHe[1]->prev = he->next;
        newHe[1]->next = nHe;
        newHe[1]->face = nF;

        newHe[2]->origin = newVertIndex;
        newHe[2]->twin = nHe + 1;
        newHe[2]->prev = heIndex;
        newHe[2]->next = he->prev;
        newHe[2]->face = f0;

        HE(he->next)->prev = nHe;
        HE(he->next)->next = nHe + 1;
        HE(he->next)->face = nF;
        he->next = nHe + 2;
        he->twin = nHe + 3;
        HE(he->prev)->prev = nHe + 2;

        newHe[3]->origin = newVertIndex;
        newHe[3]->twin = heIndex;
        newHe[3]->prev = -1;
        newHe[3]->next = -1;
        newHe[3]->face = -1;

        int i = v1 == mesh.F(f0, 0) ? 0 : v0 == mesh.F(f0, 1) ? 1 : 2;
        mesh.F(f0, i) = newVertIndex;
        if (i == 0)
            heStruct.faceHalfEdge[f0] = nHe + 2;
        mesh.faces.emplace_back(
            Eigen::Vector3i(newVertIndex, v1, HE(nHe + 1)->origin));
        heStruct.faceHalfEdge.push_back(nHe);

        if (updatedFaces)
        {
            updatedFaces[0] = f0;
            updatedFaces[1] = nF;
            updatedFaces[2] = -1;
        }
    }
}