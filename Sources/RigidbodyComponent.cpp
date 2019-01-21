#include "pch.h"
#include "..\Headers\RigidbodyComponent.h"
#include "Sphere.h"

using namespace DirectX;
using namespace SimpleMath;

RigidbodyComponent::RigidbodyComponent(float radius, float mass, bool isKinematic, XMVECTOR color)
	: m_mass(mass), m_isKinematic(isKinematic)
{
	m_shape = new Sphere(radius, color);
	m_force = Vector3( 0, 0, 0 );
	m_acceleration = Vector3( 0, 0, 0 );
	m_velocity = Vector3( 0, 0, 0 );
}


RigidbodyComponent::~RigidbodyComponent()
{
	//Destroy pointer
	delete m_shape;
	m_shape = nullptr;
}

bool RigidbodyComponent::Receive(ComponentMessage msg)
{
	return false;
}
