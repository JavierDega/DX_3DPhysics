#include "pch.h"
#include "..\Headers\PhysicSystem.h"
#include "ObjectSystem.h"
#include "Sphere.h"

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
		currentRb->m_force += m_gravity;

		//SSScheme
		//BroadPhase
		//Medium Phase
		//Narrow Phase
		//@To avoid double checks, we only check upwards
		for (unsigned int j = i + 1; j < m_rigidbodies.size(); j++) {
			NarrowPhase(currentRb, m_rigidbodies[j]);
		}
		//Apply forces
		currentRb->m_acceleration = currentRb->m_force / currentRb->m_mass;
		currentRb->m_velocity += currentRb->m_acceleration;
		currentRb->m_owner->m_transform.m_position += currentRb->m_velocity*dt;

		//Forces are computed every frame
		currentRb->m_force = Vector3(0, 0, 0);

	}

}
bool PhysicSystem::NarrowPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2) {

	Sphere * sphere1 = dynamic_cast<Sphere*>(rb1->m_shape);
	Sphere * sphere2 = dynamic_cast<Sphere*>(rb2->m_shape);
	TransformComponent * t1 = &rb1->m_owner->m_transform;
	TransformComponent * t2 = &rb2->m_owner->m_transform;
	//@Cases?
	//Resting contact, moving contact, contact vs kinematic
	//@Impulse based collision response

	//@Are they colliding?
	float distSq = Vector3::DistanceSquared(t1->m_position, t2->m_position);

	// Calculate the sum of the radii, then square it
	float sumRadiiSq = sphere1->m_radius + sphere2->m_radius;
	sumRadiiSq *= sumRadiiSq;

	if (distSq <= sumRadiiSq) {
		// A and B are touching
		//@Impulse based collision resolution

		//Displacement
		//Cases? Both moving, one of them kinematic/sleeping and the other one moving
		//@Static collision resolution based on speed?
		//float ratio = rb1->m_velocity.Length / rb2->m_velocity.Length;
		float dist = sqrtf(distSq);
		float overlap = 0.5f * (dist - sphere1->m_radius - sphere2->m_radius);

		t1->m_position -= overlap * (t1->m_position - t2->m_position) / dist;
		t2->m_position -= overlap * (t1->m_position - t2->m_position) / dist;
		
		//Dynamic resolution
		//http://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php?page=3
		// First, find the normalized vector n from the center of 
		// circle1 to the center of circle2
		Vector3 normal = t1->m_position - t2->m_position;
		normal.Normalize();

		// Find the length of the component of each of the movement
		// vectors along n. 
		// a1 = v1 . n
		// a2 = v2 . n
		float a1 = rb1->m_velocity.Dot(normal);
		float a2 = rb2->m_velocity.Dot(normal);

		// Using the optimized version, 
		// optimizedP =  2(a1 - a2)
		//              -----------
		//                m1 + m2
		float optimizedP = (2.0 * (a1 - a2)) / (rb1->m_mass + rb2->m_mass);

		// Calculate v1', the new movement vector of circle1
		// v1' = v1 - optimizedP * m2 * n
		rb1->m_velocity  = rb1->m_velocity - optimizedP * rb2->m_mass * normal;

		// Calculate v1', the new movement vector of circle1
		// v2' = v2 + optimizedP * m1 * n
		rb2->m_velocity = rb2->m_velocity + optimizedP * rb1->m_mass * normal;

	}
	return true;
}
