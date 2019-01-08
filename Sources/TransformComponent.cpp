#include "pch.h"
#include "TransformComponent.h"

//Constructor
TransformComponent::TransformComponent(DirectX::SimpleMath::Vector3 position = DirectX::SimpleMath::Vector3(0, 0, 0),
	DirectX::SimpleMath::Quaternion rotation = DirectX::SimpleMath::Quaternion::Identity,
	DirectX::SimpleMath::Vector3  scale = DirectX::SimpleMath::Vector3(1, 1, 1))
	: m_position(position), m_rotation(rotation), m_scale(scale)
{

}

//Destructor
TransformComponent::~TransformComponent() {
	//Empty vars
	m_position.x = 0;
	m_position.y = 0;
	m_position.z = 0;

}