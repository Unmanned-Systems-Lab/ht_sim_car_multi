#include <helper_functions.h>

Eigen::Matrix3f getqRot(Eigen::Vector4f q)
{
	Eigen::Matrix3f R;
	R << 1.0-2.0*(q(2)*q(2)+q(3)*q(3)),     2.0*(q(1)*q(2)-q(3)*q(0)),     2.0*(q(1)*q(3)+q(2)*q(0)),
	         2.0*(q(1)*q(2)+q(3)*q(0)), 1.0-2.0*(q(1)*q(1)+q(3)*q(3)),     2.0*(q(2)*q(3)-q(1)*q(0)),
			     2.0*(q(0)*q(3)-q(2)*q(0)),     2.0*(q(2)*q(3)+q(1)*q(0)), 1.0-2.0*(q(1)*q(1)+q(2)*q(2));
  return R;
}

//differential q matrix
Eigen::Matrix<float,4,3> B(Eigen::Vector4f q)
{
	Eigen::Matrix<float,4,3> qDiff;
	qDiff << -q(1), -q(2), -q(3),
			  q(0), -q(3),  q(2),
			  q(3),  q(0), -q(1),
			 -q(2),  q(1),  q(0);
	return qDiff;
}

//q as matrix
Eigen::Matrix4f getqMat(Eigen::Vector4f q)
{
	Eigen::Matrix4f qMat;
	qMat << q(0), -q(1), -q(2), -q(3),
					q(1),  q(0), -q(3),  q(2),
					q(2),  q(3),  q(0), -q(1),
					q(3), -q(2),  q(1),  q(0);
	return qMat;
}

//q invers
Eigen::Vector4f getqInv(Eigen::Vector4f q)
{
	Eigen::Vector4f qInv;
	qInv << q(0), -q(1), -q(2), -q(3);
	return qInv;
}

//skew symmetric
Eigen::Matrix3f getss(Eigen::Vector3f p)
{
	Eigen::Matrix3f pss;
	pss <<   0.0, -p(2),  p(1),
	        p(2),   0.0, -p(0),
		   -p(1),  p(0),   0.0;
	return pss;
}

// rotate a vector
Eigen::Vector3f rotatevec(Eigen::Vector3f v, Eigen::Vector4f q)
{
	Eigen::Vector4f rotatedvec = getqMat(getqMat(q)*Eigen::Vector4f(0.0,v(0),v(1),v(2)))*getqInv(q);
	return rotatedvec.segment(1,3);
}

// rotate a vector
Eigen::Vector3f rotatevec(Eigen::Vector3f v, Eigen::Vector4f q, bool normalize)
{
	Eigen::Vector4f rotatedvec = getqMat(getqMat(q)*Eigen::Vector4f(0.0,v(0),v(1),v(2)))*getqInv(q);

	return rotatedvec.segment(1,3)/rotatedvec.segment(1,3).norm();
}

// distance derivative wrt time
float fd(Eigen::Vector3f u, Eigen::Vector3f vc)
{
	return -1.0*u.transpose()*vc;
}

// unit vector derivative wrt time
Eigen::Vector3f fu(Eigen::Vector3f u, float d, Eigen::Vector3f vc, Eigen::Vector3f wc)
{
	return (1.0/d*(u*u.transpose() - Eigen::Matrix3f::Identity())*vc - getss(wc)*u);
}

// quaternion derivative wrt time
Eigen::Vector4f fq(Eigen::Vector4f q, Eigen::Vector3f w)
{
	return 0.5*B(q)*w;
}

// position derivative wrt time
Eigen::Vector3f fp(Eigen::Vector4f q, Eigen::Vector3f v)
{
	return rotatevec(v,q);
}

// guic
Eigen::Matrix3f guic(Eigen::Vector3f u, Eigen::Vector3f v)
{
	float u1 = u(0);
	float u2 = u(1);
	float u3 = u(2);
	float v1 = v(0);
	float v2 = v(1);
	float v3 = v(2);
	Eigen::Matrix3f gu;
	gu << 2.0*u1*v1+u2*v2+u3*v3,                 u1*v2,                 u1*v3,
						  u2*v1, u1*v1+2.0*u2*v2+u3*v3,                 u2*v3,
						  u3*v1,                 u3*v2, u1*v1+u2*v2+2.0*u3*v3;
	return gu;
}

// gpck
Eigen::Matrix<float,3,4> gpck(Eigen::Vector4f q, Eigen::Vector3f v)
{
	float qw = q(0);
	float qx = q(1);
	float qy = q(2);
	float qz = q(3);
	float vx = v(0);
	float vy = v(1);
	float vz = v(2);
	Eigen::Matrix<float,3,4> gp;
	gp << 2.0*qw*vx+2.0*qy*vz-2.0*qz*vy, 2.0*qx*vx+2.0*qy*vy+2.0*qz*vz, 2.0*qw*vz+2.0*qx*vy-2.0*qy*vx, 2.0*qx*vz-2.0*qw*vy-2.0*qz*vx,
		  2.0*qw*vy-2.0*qx*vz+2.0*qz*vx, 2.0*qy*vx-2.0*qx*vy-2.0*qw*vz, 2.0*qx*vx+2.0*qy*vy+2.0*qz*vz, 2.0*qw*vx+2.0*qy*vz-2.0*qz*vy,
		  2.0*qw*vz+2.0*qx*vy-2.0*qy*vx, 2.0*qw*vy-2.0*qx*vz+2.0*qz*vx, 2.0*qz*vy-2.0*qy*vz-2.0*qw*vx, 2.0*qx*vx+2.0*qy*vy+2.0*qz*vz;
	return gp;
}

//gqck
Eigen::Matrix4f gqck(Eigen::Vector3f wc)
{
	float wx = wc(0);
	float wy = wc(1);
	float wz = wc(2);
	Eigen::Matrix4f gq;
	gq <<       0.0, -wx/2.0, -wy/2.0, -wz/2.0,
			 wx/2.0,     0.0,  wz/2.0, -wy/2.0,
			 wy/2.0, -wz/2.0,     0.0,  wx/2.0,
			 wz/2.0,  wy/2.0, -wx/2.0,     0.0;
	return gq;
}

// gcuic
Eigen::Matrix3f gcuic(Eigen::Vector4f q)
{
	float qw = q(0);
	float qx = q(1);
	float qy = q(2);
	float qz = q(3);
	float gcu11 = std::pow(qw,2.0)+std::pow(qx,2.0)-std::pow(qy,2.0)-std::pow(qz,2.0);
	float gcu22 = std::pow(qw,2.0)-std::pow(qx,2.0)+std::pow(qy,2.0)-std::pow(qz,2.0);
	float gcu33 = std::pow(qw,2.0)-std::pow(qx,2.0)-std::pow(qy,2.0)+std::pow(qz,2.0);
	Eigen::Matrix3f gcu;
	gcu <<	              gcu11, 2.0*qx*qy-2.0*qw*qz, 2.0*qw*qy+2.0*qx*qz,
			2.0*qw*qz+2.0*qx*qy,               gcu22, 2.0*qy*qz-2.0*qw*qx,
			2.0*qx*qz-2.0*qw*qy, 2.0*qw*qx+2.0*qy*qz,               gcu33;
	return gcu;
}

//gcqck
Eigen::Matrix<float,3,4> gcqck(Eigen::Vector4f q, Eigen::Vector3f p)
{
	return gpck(q,p);
}
