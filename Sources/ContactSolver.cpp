#include "pch.h"
#include "..\Headers\ContactSolver.h"
#include "GameObject.h"

using namespace DirectX;
using namespace SimpleMath;

ContactSolver::ContactSolver()
{
	m_frictionCoefficient = .01f;
}


ContactSolver::~ContactSolver()
{
}

void ContactSolver::Solve(float dt)
{
	//Iterate through all m_collidingPairs, solve them
	for (ContactManifold manifold : m_collidingPairs) {
		//Displace along the normal by maxPenetration amount
		//@CONVENTION: Normal always points to first rigidbody pair.
		//@Overlap is always positive
		//@Impulse based collision resolution
		RigidbodyComponent * rb1 = manifold.m_rigidbodies.first;
		RigidbodyComponent * rb2 = manifold.m_rigidbodies.second;
		TransformComponent * t1 = &rb1->m_owner->m_transform;
		TransformComponent * t2 = &rb2->m_owner->m_transform;
		///1:Displacement
#pragma region Displacement

		//@Static collision resolution based on speed
		float v1Length = rb1->m_velocity.Length();
		float v2Length = rb2->m_velocity.Length();
		//@What if two objects with no velocity just collided?
		float v1Ratio = v1Length / (v1Length + v2Length);
		float v2Ratio = v2Length / (v1Length + v2Length);
			
		float maxOverlap = 0.f;
		for (ContactPoint cp : manifold.m_points) {
			if (cp.m_penetration > maxOverlap) maxOverlap = cp.m_penetration;
		}
		if (!rb1->m_isKinematic)t1->m_position += v1Ratio * maxOverlap * manifold.m_normal;
		if (!rb2->m_isKinematic)t2->m_position -= v2Ratio * maxOverlap * manifold.m_normal;
#pragma endregion

		///2:Dynamic resolution
#pragma region Linear Resolution

		//http://www.gamasutra.com/view/feature/131424/pool_hall_lessons_fast_accurate_.php?page=3
		// First, find the normalized vector n from the center of
		// circle1 to the center of circle2

		// Find the length of the component of each of the movement
		// vectors along n.
		// a1 = v1 . n
		// a2 = v2 . n

		float a1 = rb1->m_velocity.Dot(manifold.m_normal);
		float a2 = rb2->m_velocity.Dot(manifold.m_normal);

		// Using the optimized version,
		// optimizedP =  2(a1 - a2)
		//              -----------
		//                m1 + m2
		float optimizedP = (2.0f * (a1 - a2)) / (rb1->m_mass + rb2->m_mass);

		//@Store previous velocity
		Vector3 prevVel = rb1->m_velocity;
		Vector3 prevVel2 = rb2->m_velocity;

		// Calculate v1', the new movement vector of circle1
		// v1' = v1 - optimizedP * m2 * n
		rb1->m_velocity = rb1->m_velocity - optimizedP * rb2->m_mass * manifold.m_normal;
		rb2->m_velocity = rb2->m_velocity + optimizedP * rb1->m_mass * manifold.m_normal;
		 
#pragma endregion

		//@Derive force applied from our impulse resolution
		Vector3 velDelta = rb1->m_velocity - prevVel;
		Vector3 velDelta2 = rb2->m_velocity - prevVel2;
		//Calculate normal force from impulse
		//f = m*a
		//f = mdv/dt
		Vector3 rb1Force = rb1->m_mass * velDelta / dt;
		Vector3 rb2Force = rb2->m_mass * velDelta2 / dt;

#pragma region Angular resolution
		//@Now we calculate our angular resolution
		// Flinear = F
		//Ftorque = F x(p - x)
		rb1->m_torque = (manifold.m_points[0].m_position - t1->m_position).Cross(rb1Force);//@NOTE: WE'RE ASSUMING SINGLE CONTACT POINTS HERE
		rb2->m_torque = (manifold.m_points[0].m_position - t2->m_position).Cross(rb2Force);

#pragma endregion

		//3: Friction::Because of our Impulse based resolution, we need to calculate the normal force 'After the fact'
#pragma region Kinetic friction

		//@Friction:
		//Find component of force along normal
		float normalForce = rb1Force.Dot(manifold.m_normal);//@DX11's Dot is unnornalized so already multiplied by length
		float normal2Force = rb2Force.Dot(manifold.m_normal);

		Vector3 vel1Norm = rb1->m_velocity;
		vel1Norm.Normalize();
		Vector3 vel2Norm = rb2->m_velocity;
		vel2Norm.Normalize();

		Vector3 friction1 = -vel1Norm * m_frictionCoefficient * abs(normalForce);//normalForce should be positive here
		rb1->m_force += friction1;

		Vector3 friction2 = -vel2Norm * m_frictionCoefficient * abs(normal2Force);//normal2Force should be negative here
		rb2->m_force += friction2;
		//Put to rest?

#pragma endregion
		

	}
	m_collidingPairs.clear();
}
