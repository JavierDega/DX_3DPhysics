#ifndef NARROWPHASE_H_
#define NARROWPHASE_H_

#include "RigidbodyComponent.h"
#include "OrientedBoundingBox.h"
#include "ContactSolver.h"
class NarrowPhase
{
public:
	NarrowPhase();
	~NarrowPhase();

	void ApplyImpulse(DirectX::SimpleMath::Vector3 impulse, DirectX::SimpleMath::Vector3 contactVector);
	///@Test intersection queries
	bool SphereToSphere(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt);
	bool SphereToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt);
	bool OBBToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt);
	bool OBBToOBBSAT(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt);
	//@Helpful queries
	DirectX::SimpleMath::Vector3 FindIntersectionWithPlaneFromDistances(DirectX::SimpleMath::Vector3 start, DirectX::SimpleMath::Vector3 end, float d1, float d2);
	//Closest point on plane to point
	DirectX::SimpleMath::Vector3 ClosestPtPointPlane(DirectX::SimpleMath::Vector3 point, DirectX::SimpleMath::Plane plane);
	//Dist to plane
	float DistPointPlane(DirectX::SimpleMath::Vector3 q, DirectX::SimpleMath::Plane p);
	// Given point p, return point q on (or in) OBB b, closest to p 
	DirectX::SimpleMath::Vector3 ClosestPtPointOBB(DirectX::SimpleMath::Vector3 p, OrientedBoundingBox * b, DirectX::SimpleMath::Vector3 bc, DirectX::SimpleMath::Quaternion bRot);
	//Get closest point between two line segments, returns LengthSq between both
	float ClosestPtSegmentSegment(DirectX::SimpleMath::Vector3 a1, DirectX::SimpleMath::Vector3 a2, DirectX::SimpleMath::Vector3 b1, DirectX::SimpleMath::Vector3 b2,
		DirectX::SimpleMath::Vector3 & p1, DirectX::SimpleMath::Vector3 & p2, float &f1, float &f2);
	//Get closest point p to a triangle
	DirectX::SimpleMath::Vector3 ClosestPtPointTriangle(DirectX::SimpleMath::Vector3 p, DirectX::SimpleMath::Vector3 a, DirectX::SimpleMath::Vector3 b, DirectX::SimpleMath::Vector3 c);
	//Get closest point p to a tetrahedron defined by four points
	DirectX::SimpleMath::Vector3 ClosestPtPointTetrahedron(DirectX::SimpleMath::Vector3 p, DirectX::SimpleMath::Vector3 a, DirectX::SimpleMath::Vector3 b, DirectX::SimpleMath::Vector3 c,
		DirectX::SimpleMath::Vector3 d);
	//@SUtherland-Hodgmann clipping algorithm
	std::vector<DirectX::SimpleMath::Vector3> OBBClip(DirectX::SimpleMath::Vector3 faceVertices[4], DirectX::SimpleMath::Plane supportPlanes[4], DirectX::SimpleMath::Plane referencePlane);

	//Variables
	//@Solver
	ContactSolver m_solver;
};
#endif /*NARROWPHASE_H_*/