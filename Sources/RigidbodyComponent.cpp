#include "pch.h"
#include "..\Headers\RigidbodyComponent.h"

using namespace DirectX;

RigidbodyComponent::RigidbodyComponent(float radius)
	: m_radius(radius)
{
}


RigidbodyComponent::~RigidbodyComponent()
{
}

bool RigidbodyComponent::Receive(ComponentMessage msg)
{
	return false;
}
