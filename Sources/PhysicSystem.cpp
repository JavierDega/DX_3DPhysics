#include "pch.h"
#include "..\Headers\PhysicSystem.h"
#include "ObjectSystem.h"

using namespace DirectX;
using namespace SimpleMath;



//Instance
PhysicSystem * PhysicSystem::m_instance = NULL;
PhysicSystem * PhysicSystem::GetInstance()
{
	//Singleton
	if (m_instance == NULL)
	{
		m_instance = new PhysicSystem();
	}
	return m_instance;
}
//Constructor
PhysicSystem::PhysicSystem()
{
	m_gravity = Vector3 ( 0, -9.8f, 0 );
	m_minDt = 1.0f / 60.0f;
	m_accumulator = 0;
}
//Destructor
PhysicSystem::~PhysicSystem()
{
}

void PhysicSystem::Initialize(ID3D11Device1 * device, ID3D11DeviceContext1 * deviceContext)
{
	//@What do here?
}

void PhysicSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
	//@What do here?
}

void PhysicSystem::Update(float dt)
{
	//https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-core-engine--gamedev-7493
	//@Timestep, advance physics, run all algorithm classes
	m_accumulator += dt;

	// Avoid spiral of death and clamp dt, thus clamping
	// how many times the UpdatePhysics can be called in
	// a single game loop.
	if (m_accumulator > 0.2f) m_accumulator = 0.2f;

	while (m_accumulator > m_minDt) {
		UpdatePhysics(m_minDt);
		m_accumulator -= m_minDt;
	}

	//To create a lerp between this frame and the next, interact with the graphic system.
	//ApproxTransform.position = transform.position + m_velocity*m_accumulator ?
	float alpha = m_accumulator / m_minDt;

}

void PhysicSystem::Reset()
{
	//@What do here?
}
//Physics Update
void PhysicSystem::UpdatePhysics(float dt) {
	std::vector<RigidbodyComponent*> m_rigidbodies = ObjectSystem::GetInstance()->GetRigidbodyComponentList();
	for (unsigned int i = 0; i < m_rigidbodies.size(); i++) {
		
		//Kinematics
		RigidbodyComponent* currentRb = m_rigidbodies[i];
		//Forces are computed every frame
		currentRb->m_force = Vector3( 0, 0, 0 );
		currentRb->m_force += m_gravity;

		//SSScheme
		//BroadPhase
		//Medium Phase
		//Narrow Phase

		//Apply forces
		currentRb->m_acceleration = currentRb->m_force / currentRb->m_mass;
		currentRb->m_velocity += currentRb->m_acceleration;
		currentRb->m_owner->m_transform.m_position += currentRb->m_velocity*dt;

	}

}

