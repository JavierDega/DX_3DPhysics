#include "pch.h"
#include "..\Headers\RigidbodyComponent.h"
#include "Sphere.h"
#include "OrientedBoundingBox.h"
using namespace DirectX;
using namespace SimpleMath;


//Sphere constructor
RigidbodyComponent::RigidbodyComponent( float radius, float mass, bool isKinematic, XMVECTOR color)
	: m_mass(mass), m_isKinematic(isKinematic), m_force(Vector3::Zero), m_acceleration(Vector3::Zero), m_velocity(Vector3::Zero)
{
	m_shape = new Sphere(radius, color);
}

//OBB Constructor
RigidbodyComponent::RigidbodyComponent( Vector3 halfExtents, float mass, bool isKinematic, XMVECTOR color)
	: m_mass(mass), m_isKinematic(isKinematic), m_force(Vector3::Zero), m_acceleration(Vector3::Zero), m_velocity(Vector3::Zero)
{
	m_shape = new OrientedBoundingBox(halfExtents, color);
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
