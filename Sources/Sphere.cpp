#include "pch.h"
#include "..\Headers\Sphere.h"


using namespace DirectX;
using namespace SimpleMath;

Sphere::Sphere(float radius, XMVECTOR color)
	: m_radius(radius), Shape(color)
{

}


Sphere::~Sphere()
{
}

AABB Sphere::ComputeAABB()
{
	//minExtent: (-radius, -radius, -radius )
	//maxExtent: ( radius, radius, radius )
	return AABB{ Vector3(-m_radius, -m_radius, -m_radius), Vector3(m_radius, m_radius, m_radius) };
}
