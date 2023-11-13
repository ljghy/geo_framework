#ifndef GEO_IO_H_
#define GEO_IO_H_

#include <string>

#include <Eigen/Core>

#include <geo/mesh/Mesh.h>

#include <geo/io/tiny_obj_loader.h>

void loadMeshFromVtk(const std::string &filename, Mesh &mesh);

void loadMeshFromObj(const std::string &filename, Mesh &mesh);

void loadVerticesFromObj(const std::string &filename,
                         Eigen::Matrix3Xd &vertices);

void writeMeshToVtk(const std::string &filename, const Mesh &mesh);

void loadMeshVertexScalarFieldFromVtk(const std::string &filename, Mesh &mesh,
                                      Eigen::VectorXd &phi);

void writeMeshVertexScalarFieldToVtk(const std::string &filename,
                                     const Mesh &mesh,
                                     const Eigen::VectorXd &phi);

void writeMeshVertexVectorFieldToVtk(const std::string &filename,
                                     const Mesh &mesh,
                                     const Eigen::Matrix3Xd &phi);

void writeMeshFaceScalarFieldToVtk(const std::string &filename,
                                   const Mesh &mesh,
                                   const Eigen::VectorXd &phi);

void writeMeshFaceVectorFieldToVtk(const std::string &filename,
                                   const Mesh &mesh,
                                   const Eigen::Matrix3Xd &phi);

void writeCurveToVtk(const std::string &filename,
                     const std::vector<Eigen::Vector3d> &vertices, bool isLoop);

void writeVerticesToVtk(const std::string &filename,
                        const std::vector<Eigen::Vector3d> &vertices);

#endif
