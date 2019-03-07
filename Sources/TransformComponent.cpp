#include "pch.h"
#include "TransformComponent.h"

using namespace DirectX;
using namespace SimpleMath;
//Constructor
TransformComponent::TransformComponent(Vector3 position, Quaternion rotation)
	: m_position(position), m_rotation(rotation)
{

}

//Destructor
TransformComponent::~TransformComponent() {
	//Empty vars


}

bool TransformComponent::Receive(ComponentMessage msg)
{
	return false;
}
