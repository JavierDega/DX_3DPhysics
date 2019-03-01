#ifndef PHYSICSYSTEM_H_
#define PHYSICSYSTEM_H_
#include "System.h"
#include "NarrowPhase.h"
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
	bool ComputeNarrowPhase(RigidbodyComponent * rb1, RigidbodyComponent *rb2, float dt );

	NarrowPhase m_narrowPhase;
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