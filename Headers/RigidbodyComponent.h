#ifndef RIGIDBODYCOMPONENT_H_
#define RIGIDBODYCOMPONENT_H_

#include "Component.h"
#include "Shape.h"
//@Handles physics, and indirectly, holds all graphic/render related data
class RigidbodyComponent :
	public Component
{
public:
	//Funcs

	//@@@CONVENTION FOR CONSTRUCTORS:
	//SHAPE PROPERTIES->MASS->IsKinematic->Color
	//@TRY AND DIFFERENTIATE DIFFERENT COLLIDER SHAPES FROM DIFFERENT SHAPE PROPERTIES, DATA TYPE ORDER

	//@Sphere constructor
	RigidbodyComponent( float radius = 0.5f, float mass = 10.0f, bool isKinematic = false, DirectX::XMVECTOR color = DirectX::Colors::White);
	//@OBB constructor
	RigidbodyComponent( DirectX::SimpleMath::Vector3 halfExtents = DirectX::SimpleMath::Vector3::One, float mass = 10.0f, bool isKinematic = false, DirectX::XMVECTOR color = DirectX::Colors::White );
	virtual ~RigidbodyComponent();
	//Messaging
	virtual bool Receive(ComponentMessage msg);

	//Variables
	Shape * m_shape;
	float m_mass;
	bool m_isKinematic;
	//Semi euler
	DirectX::SimpleMath::Vector3 m_force, m_acceleration, m_velocity;
	DirectX::SimpleMath::Vector3 m_torque;//@Angular force
	DirectX::SimpleMath::Vector3 m_angularAcceleration;//@Ang accel
	DirectX::SimpleMath::Vector3 m_angularVelocity;//Radians/dt. Direction represents axis of rotation, magnitude is amount of rotation

};
#endif /*RIGIDBODYCOMPONENT_H_*/