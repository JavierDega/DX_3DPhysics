#ifndef PHYSICSYSTEM_H_
#define PHYSICSYSTEM_H_
#include "System.h"
#include "ContactSolver.h"
#include "RigidbodyComponent.h"
#include "OrientedBoundingBox.h"

typedef struct SettingStr {
	std::wstring log;
	bool isEnabled;
}Setting;
/*Iterates on all rigidbodies
Computing timestep, then running:
Integration update (Semi-euler - Verlet)
Space subdivision (Uniform - Octree)
Broad Phase Solver (AABB - Sphere culling)
Medium Phase Solver (GJK)
Narrow phase Solver on final colliding pairs ( OBBs - Cylinders - Capsules)
*/
class PhysicSystem :
	public System
{
private:
	/*Here will be the instance stored*/
	static PhysicSystem* m_instance;
	/*Private constructor to prevent instancing*/
	PhysicSystem();
public:
	~PhysicSystem();
	//Singleton
	static PhysicSystem* GetInstance();

	///Functions
	//Events
	void Initialize(ID3D11Device1* device = nullptr, ID3D11DeviceContext1 * deviceContext = nullptr);
	void InitWindow(D3D11_VIEWPORT screenViewport = D3D11_VIEWPORT{ 0, 0, 0, 0 });
	virtual void Update(float dt);
	virtual void Reset();
	//Utility
	//@Timestep
	void UpdatePhysics(float dt);
	//@BroadPhase
	bool BroadPhase(RigidbodyComponent * rb1, RigidbodyComponent *rb2);
	AABB ComputeAABB(RigidbodyComponent * rb);
	//@NarrowPhase
	bool NarrowPhase(RigidbodyComponent * rb1, RigidbodyComponent *rb2, float dt );
	void ApplyImpulse(DirectX::SimpleMath::Vector3 impulse, DirectX::SimpleMath::Vector3 contactVector);
	///@Test intersection queries
	bool SphereToSphere(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt);
	bool SphereToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt );
	bool OBBToOBB(RigidbodyComponent * rb1, RigidbodyComponent * rb2, float dt );

	//@Helpful queries
	DirectX::SimpleMath::Vector3 QueryOBBEdgeContact(RigidbodyComponent * rb1, RigidbodyComponent * rb2, DirectX::SimpleMath::Vector3 edge1Dir, DirectX::SimpleMath::Vector3 edge2Dir,
		DirectX::SimpleMath::Vector3 axisOfMinimumPenetration, float penetrationDepth);
	//Closest point on plane to point
	DirectX::SimpleMath::Vector3 ClosestPtPointPlane(DirectX::SimpleMath::Vector3 point, DirectX::SimpleMath::Plane plane);
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
	//@Timestep
	float m_minDt;
	float m_accumulator;
	//@Simulation settings
	DirectX::SimpleMath::Vector3 m_gravity;
	float m_airViscosity;
	std::wstring m_fps;
	//Space subdivision
	//BroadPhase
	Setting m_AABBCulling;
	Setting m_sphereCulling;
	//Medium phase

	//@Debug
	bool m_stepMode;
	bool m_stepOnce;

};
#endif /*PHYSICSYSTEM_H_*/