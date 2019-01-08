#include "pch.h"
#include "TransformComponent.h"

using namespace DirectX;
using namespace SimpleMath;
//Constructor
TransformComponent::TransformComponent(Vector3 position, Quaternion rotation, Vector3  scale)
	: m_position(position), m_rotation(rotation), m_scale(scale)
{

}

//Destructor
TransformComponent::~TransformComponent() {
	//Empty vars


}