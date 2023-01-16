#include <Eigen/Dense>
#include <Eigen/Geometry>

//quaternion as a rotation matrix
Eigen::Matrix3f getqRot(Eigen::Vector4f q);

//differential q matrix
Eigen::Matrix<float,4,3> B(Eigen::Vector4f q);

//q as matrix
Eigen::Matrix4f getqMat(Eigen::Vector4f q);

//q invers
Eigen::Vector4f getqInv(Eigen::Vector4f q);

//skew symmetric
Eigen::Matrix3f getss(Eigen::Vector3f p);

// rotate a vector
Eigen::Vector3f rotatevec(Eigen::Vector3f v, Eigen::Vector4f q);

// rotate a vector
Eigen::Vector3f rotatevec(Eigen::Vector3f v, Eigen::Vector4f q, bool normalize);

// distance derivative wrt time
float fd(Eigen::Vector3f u, Eigen::Vector3f vc);

// unit vector derivative wrt time
Eigen::Vector3f fu(Eigen::Vector3f u, float d, Eigen::Vector3f vc, Eigen::Vector3f wc);

// quaternion derivative wrt time
Eigen::Vector4f fq(Eigen::Vector4f q, Eigen::Vector3f w);

// position derivative wrt time
Eigen::Vector3f fp(Eigen::Vector4f q, Eigen::Vector3f v);

// guic
Eigen::Matrix3f guic(Eigen::Vector3f u, Eigen::Vector3f v);

// gpck
Eigen::Matrix<float,3,4> gpck(Eigen::Vector4f q, Eigen::Vector3f v);

//gqck
Eigen::Matrix4f gqck(Eigen::Vector3f wc);

// gcuic
Eigen::Matrix3f gcuic(Eigen::Vector4f q);

//gcqck
Eigen::Matrix<float,3,4> gcqck(Eigen::Vector4f q, Eigen::Vector3f p);
