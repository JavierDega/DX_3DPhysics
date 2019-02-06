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
	//Sphere constructor
	RigidbodyComponent(float radius = 0.5f, float mass = 10.0f, bool m_isKinematic = false, DirectX::XMVECTOR color = DirectX::Colors::White);
	virtual ~RigidbodyComponent();
	//Messaging
	virtual bool Receive(ComponentMessage msg);

	//Variables
	Shape * m_shape;
	float m_mass;
	bool m_isKinematic;
	//Semi euler
	DirectX::SimpleMath::Vector3 m_force, m_acceleration, m_velocity;

};
#endif /*RIGIDBODYCOMPONENT_H_*/