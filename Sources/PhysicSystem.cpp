#include "pch.h"
#include "..\Headers\PhysicSystem.h"
#include "ObjectSystem.h"
#include "Sphere.h"

using namespace DirectX;
using namespace SimpleMath;
using namespace std;


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
	//Viscosity for earth's air @  0'Celsius = 1.33*10^-5 kg/ms^2
	m_airViscosity = 0.133f;
	m_minDt = 1.0f / 60.0f;
	m_accumulator = 0;

	m_AABBCulling.isEnabled = true;
}
//Destructor
PhysicSystem::~PhysicSystem()
{

}
///Init
void PhysicSystem::Initialize(ID3D11Device1 * device, ID3D11DeviceContext1 * deviceContext)
{
	//@What do here?
}
///Init window
void PhysicSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
	//@What do here?
}
///Timestep and run physics
void PhysicSystem::Update(float dt)
{
	//@Update settings text
	m_fps = to_wstring(1 / dt);
	m_AABBCulling.log = m_AABBCulling.isEnabled ? L"Press F1 to disable AABB culling" : L"Press F1 to enable AABB culling";

	//@Timestep
	//https://gamedevelopment.tutsplus.com/tutorials/how-to-create-a-custom-2d-physics-engine-the-core-engine--gamedev-7493
	m_accumulator += dt;

	// @Avoid spiral of death and clamp dt, thus clamping
	// how many times the UpdatePhysics can be called in
	// a single game loop.
	if (m_accumulator > 0.2f) m_accumulator = 0.2f;

	while (m_accumulator > m_minDt) {
		UpdatePhysics(m_minDt);
		m_accumulator -= m_minDt;
	}

	//@To create a lerp between this frame and the next, interact with the graphic system.
	//ApproxTransform.position = transform.position + m_velocity*m_accumulator ?
	float alpha = m_accumulator / m_minDt;

}
///Reset
void PhysicSystem::Reset()
{
	//@What do here?
}
///Utility
///Physics loop
void PhysicSystem::UpdatePhysics(float dt) {
	vector<RigidbodyComponent*> m_rigidbodies = ObjectSystem::GetInstance()->GetRigidbodyComponentList();
	vector<pair<RigidbodyComponent*, RigidbodyComponent*>> m_collidingPairs;

	//@Clear debug colors
	for (RigidbodyComponent* rb : m_rigidbodies){
		rb->m_shape->m_AABBColor = Colors::Red;
	}	

	//@First loop: Integration + First culling algorithm
	for (unsigned int i = 0; i < m_rigidbodies.size(); i++) {
		RigidbodyComponent* currentRb = m_rigidbodies[i];
		//@Integration
		if (currentRb->m_isKinematic) {
			currentRb->m_acceleration = Vector3::Zero;
			currentRb->m_velocity = Vector3::Zero;
		}
		else
		{
			currentRb->m_force -= m_airViscosity * currentRb->m_velocity;
			currentRb->m_force += m_gravity;
			currentRb->m_acceleration = currentRb->m_force / currentRb->m_mass;
			currentRb->m_velocity += currentRb->m_acceleration*dt;
			currentRb->m_owner->m_transform.m_position += currentRb->m_velocity*dt;
		}
		//Forces are computed every frame
		currentRb->m_force = Vector3(0, 0, 0);

		//@SSScheme

		//@BroadPhase
		for (unsigned int j = i + 1; j < m_rigidbodies.size(); j++) {
			//@To avoid double checks, we only check upwards
			if(BroadPhase(currentRb, m_rigidbodies[j])) m_collidingPairs.push_back(make_pair(currentRb, m_rigidbodies[j]));
		}
	}

	//@Start nulling out collider pairs
	
	//@Medium Phase

	///@Surviving pairs MUST be colliding.
	
	//@Narrow Phase
	for (unsigned int i = 0; i < m_collidingPairs.size(); i++) {
		NarrowPhase(m_collidingPairs[i].first, m_collidingPairs[i].second);
	}
}
///Broad phase component
bool PhysicSystem::BroadPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2) {

	if (rb1->m_isKinematic && rb2->m_isKinematic) return false;
	//@Compute AABBs
	if (m_AABBCulling.isEnabled) {
		//@We assume shapes as spheres
		Sphere * sphere1 = dynamic_cast<Sphere*>(rb1->m_shape);
		Sphere * sphere2 = dynamic_cast<Sphere*>(rb2->m_shape);
		TransformComponent * t1 = &rb1->m_owner->m_transform;
		TransformComponent * t2 = &rb2->m_owner->m_transform;

		AABB box1 = sphere1->ComputeAABB();
		box1.m_minExtent += t1->m_position;
		box1.m_maxExtent += t1->m_position;

		AABB box2 = sphere2->ComputeAABB();
		box2.m_minExtent += t2->m_position;
		box2.m_maxExtent += t2->m_position;

		//Define bounds
		float thisRight = box1.m_maxExtent.x; float otherRight = box2.m_maxExtent.x;
		float thisLeft = box1.m_minExtent.x; float otherLeft = box2.m_minExtent.x;
		float thisTop = box1.m_maxExtent.y; float otherTop = box2.m_maxExtent.y;
		float thisBottom = box1.m_minExtent.y; float otherBottom = box2.m_minExtent.y;
		float thisFront = box1.m_maxExtent.z; float otherFront = box2.m_maxExtent.z;
		float thisBack = box1.m_minExtent.z; float otherBack = box2.m_minExtent.z;

		if (!(
			thisRight < otherLeft
			|| thisLeft > otherRight
			|| thisTop < otherBottom
			|| thisBottom > otherTop
			|| thisFront < otherBack
			|| thisBack > otherFront
			)
			) {
			//@Set AABB colors
			sphere1->m_AABBColor = Colors::Yellow;
			sphere2->m_AABBColor = Colors::Yellow;
			return true;
		}
		else {
			return false;
		}
	}

	//Default: if BroadPhase is disabled
	return true;
}
///Narrow phase component
bool PhysicSystem::NarrowPhase(RigidbodyComponent * rb1, RigidbodyComponent * rb2) {

	if (rb1->m_isKinematic && rb2->m_isKinematic) return false;

	Sphere * sphere1 = dynamic_cast<Sphere*>(rb1->m_shape);
	Sphere * sphere2 = dynamic_cast<Sphere*>(rb2->m_shape);
	TransformComponent * t1 = &rb1->m_owner->m_transform;
	TransformComponent * t2 = &rb2->m_owner->m_transform;
	//@Cases?
	//Resting contact, moving contact, contact vs kinematic
	//@Impulse based collision response

	//@1:Are they colliding?
	float distSq = Vector3::DistanceSquared(t1->m_position, t2->m_position);
	// Calculate the sum of the radii, then square it
	float sumRadiiSq = sphere1->m_radius + sphere2->m_radius;
	sumRadiiSq *= sumRadiiSq;
	if (distSq <= sumRadiiSq) {
		// A and B are touching
		//@Impulse based collision resolution
		///1:Displacement
		//Calculate overlap
		float dist = sqrtf(distSq);
		float overlap = (dist - sphere1->m_radius - sphere2->m_radius);
		//@Static collision resolution based on speed
		float v1Length = rb1->m_velocity.Length();
		float v2Length = rb2->m_velocity.Length();
		//@What if two objects with no velocity just collided?
		float v1Ratio = v1Length / (v1Length + v2Length);
		float v2Ratio = v2Length / (v1Length + v2Length);

		//@We want to keep values for accurate displacement
		Vector3 pos1Prev = t1->m_position;
		Vector3 pos2Prev = t2->m_position;
		t1->m_position -= v1Ratio * overlap * (pos1Prev - pos2Prev) / dist;
		t2->m_position += v2Ratio * overlap * (pos1Prev - pos2Prev) / dist;

		///2:Dynamic resolution
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
		float optimizedP = (2.0f * (a1 - a2)) / (rb1->m_mass + rb2->m_mass);

		// Calculate v1', the new movement vector of circle1
		// v1' = v1 - optimizedP * m2 * n
		rb1->m_velocity = rb1->m_velocity - optimizedP * rb2->m_mass * normal;

		// Calculate v1', the new movement vector of circle1
		// v2' = v2 + optimizedP * m1 * n
		rb2->m_velocity = rb2->m_velocity + optimizedP * rb1->m_mass * normal;

		return true;
	}
	return false;
}