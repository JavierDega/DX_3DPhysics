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
	for (ContactManifold manifold : m_contactManifolds) {
		//Displace along the normal by maxPenetration amount
		//@CONVENTION: Normal always points to first rigidbody pair.
		//@Overlap is always positive
		//@Impulse based collision resolution
		RigidbodyComponent * rb1 = manifold.m_rigidbodies.first;
		RigidbodyComponent * rb2 = manifold.m_rigidbodies.second;
		TransformComponent * t1 = &rb1->m_owner->m_transform;
		TransformComponent * t2 = &rb2->m_owner->m_transform;

		Vector3 oneToTwo = t2->m_position - t1->m_position;
		if (oneToTwo.Dot(manifold.m_normal) >= 0) manifold.m_normal *= -1;//@Normal should point to t1 instead of to t2

		//@If both their velocities are separating they're likely duplicates and have already been processed
		if (rb1->m_velocity.Dot(manifold.m_normal) > 0) {
			if (rb2->m_velocity.Dot(manifold.m_normal) < 0)
			{
				continue;//@We don't process them (Manifold set is cleared every frame)
			}
		}

		///1:Displacement
#pragma region Displacement

		//@Static collision resolution based on speed
		float v1Length = rb1->m_velocity.Length();
		float v2Length = rb2->m_velocity.Length();
		//@What if two objects with no velocity just collided?
		float v1Ratio = v1Length / (v1Length + v2Length);
		float v2Ratio = v2Length / (v1Length + v2Length);
			
		if (!rb1->m_isKinematic)t1->m_position += v1Ratio * manifold.m_maxPenetration * manifold.m_normal;
		if (!rb2->m_isKinematic)t2->m_position -= v2Ratio * manifold.m_maxPenetration * manifold.m_normal;
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

			Vector3 prevTorque = rb1->m_torque;
			Vector3 prevTorque2 = rb2->m_torque;
			for (int i = 0; i < manifold.m_points.size(); i++) {
#pragma region Angular resolution
				//@Now we calculate our angular resolution
				// Flinear = F
				//Ftorque = F x(p - x)
				Vector3 f1 = rb1Force / manifold.m_points.size();
				Vector3 f2 = rb2Force / manifold.m_points.size();
				rb1->m_torque = (manifold.m_points[i] - t1->m_position).Cross(f1);//@MUTLIPLE CONTACT POINTS
				rb2->m_torque = (manifold.m_points[i] - t2->m_position).Cross(f2);

#pragma endregion
			}
			Vector3 appliedTorque = rb1->m_torque - prevTorque;
			Vector3 appliedTorque2 = rb2->m_torque - prevTorque2;

			//3: Friction::Because of our Impulse based resolution, we need to calculate the normal force 'After the fact'
#pragma region Static/Kinetic friction

		//@Friction:
		//Find component of force along normal
		float normalForce = rb1Force.Dot(manifold.m_normal);//@DX11's Dot is unnornalized so already multiplied by length
		float normal2Force = rb2Force.Dot(manifold.m_normal);

		Vector3 vel1Norm = rb1->m_velocity;
		vel1Norm.Normalize();
		Vector3 vel2Norm = rb2->m_velocity;
		vel2Norm.Normalize();
		
		Vector3 angForce1Norm = rb1->m_torque;
		angForce1Norm.Normalize();
		Vector3 angForce2Norm = rb2->m_torque;
		angForce2Norm.Normalize();

		Vector3 friction1;
		Vector3 rotFriction1;
		if (rb1->m_isSleeping) {
			//Static friction model:
			//Friction is twice the m_frictionCoefficient
			friction1 = -vel1Norm * m_frictionCoefficient * 3 * abs(normalForce);
			rotFriction1 = -angForce1Norm * m_frictionCoefficient * 3 * abs(normalForce);
			if (friction1.LengthSquared() >= rb1Force.LengthSquared()) {
				//Scale friction to be equal to rb1Force, on the opposite direction
				friction1 = -rb1Force;
			}
			if (rotFriction1.LengthSquared() >= appliedTorque.LengthSquared()) {
				//@Rescale
				rotFriction1 = -appliedTorque;
			}
		}
		else {
			//Kinetic friction
			friction1 = -vel1Norm * m_frictionCoefficient * abs(normalForce);
			rotFriction1 = -angForce1Norm * m_frictionCoefficient *abs(normalForce);
		}
		rb1->m_force += friction1;
		rb1->m_torque += rotFriction1;

		Vector3 friction2;//normal2Force should be negative here
		Vector3 rotFriction2;
		if (rb2->m_isSleeping) {
			//Static friction model:
			//Friction is twice the m_frictionCoefficient
			friction2 = -vel2Norm * m_frictionCoefficient * 3 * abs(normal2Force);
			rotFriction2 = -angForce2Norm * m_frictionCoefficient * 3 * abs(normal2Force);
			if (friction2.LengthSquared() >= rb2Force.LengthSquared()) {
				//Scale friction to be equal to rb1Force, on the opposite direction
				friction2 = -rb2Force;
			}
			if (rotFriction2.LengthSquared() >= appliedTorque2.LengthSquared()) {
				//Scale rotFriction to be equal to appliedTorque2, on the opposite direction
				rotFriction2 = -appliedTorque2;
			}
		}
		else {
			//Kinetic friction
			friction2 = -vel2Norm * m_frictionCoefficient * abs(normal2Force);
			rotFriction2 = -angForce2Norm * m_frictionCoefficient * abs(normal2Force);
		}
		rb2->m_force += friction2;
		rb2->m_torque += rotFriction2;
		
#pragma endregion

	}
}
