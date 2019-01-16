#ifndef PHYSICSYSTEM_H_
#define PHYSICSYSTEM_H_
#include "System.h"

//@Iterates on all rigidbodies
//Computing timestep, then running:
//Kinematics update
//Space subdivision (Binning algorithm)
//Broad Phase Solver
//Medium Phase Solver
//Narrow phase Solver on final colliding pairs
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
	//@Events
	void Initialize(ID3D11Device1* device = nullptr, ID3D11DeviceContext1 * deviceContext = nullptr);
	void InitWindow(D3D11_VIEWPORT screenViewport = D3D11_VIEWPORT{ 0, 0, 0, 0 });
	virtual void Update(float dt);
	virtual void Reset();
	//Utility
	void UpdatePhysics(float dt);

	//Variables
	DirectX::SimpleMath::Vector3 m_gravity;
	//Timestep
	float m_minDt;
	float m_accumulator;
};
#endif /*PHYSICSYSTEM_H_*/