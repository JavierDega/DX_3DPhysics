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
	//@Update AABB extents and return it (In case of Spheres, extents never change)
	//minExtent: (-radius, -radius, -radius )
	//maxExtent: ( radius, radius, radius )
	m_AABB = AABB{
		Vector3(-m_radius, -m_radius, -m_radius),
		Vector3(m_radius, m_radius, m_radius)
	};
	return m_AABB;
}
