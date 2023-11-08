#ifndef GEO_IO_H_
#define GEO_IO_H_

#include <string>

#include <Eigen/Core>

#include <geo/mesh/Mesh.h>

void loadMeshFromVtk(const std::string &filename, Mesh &mesh);

void writeMeshToVtk(const std::string &filename, const Mesh &mesh);

void loadMeshVertexScalarFieldFromVtk(const std::string &filename, Mesh &mesh,
                                      Eigen::VectorXd &phi);

void writeMeshVertexScalarFieldToVtk(const std::string &filename,
                                     const Mesh &mesh,
                                     const Eigen::VectorXd &phi);

void writeMeshVertexVectorFieldToVtk(const std::string &filename,
                                     const Mesh &mesh,
                                     const Eigen::Matrix3Xd &phi);

void writeMeshFaceVectorFieldToVtk(const std::string &filename,
                                   const Mesh &mesh,
                                   const Eigen::Matrix3Xd &phi);

void writeCurveToVtk(const std::string &filename,
                     const std::vector<Eigen::Vector3d> &vertices, bool isLoop);

void writeVerticesToVtk(const std::string &filename,
                        const std::vector<Eigen::Vector3d> &vertices);

#endif
