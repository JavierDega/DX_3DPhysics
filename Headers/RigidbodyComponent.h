#include "Component.h"

//@Handles physics, and indirectly, holds all graphic/render related data
class RigidbodyComponent :
	public Component
{
public:
	RigidbodyComponent(float radius = 0.5f);
	virtual ~RigidbodyComponent();
	//Messaging
	virtual bool Receive(ComponentMessage msg);
	//Variables
	std::unique_ptr<DirectX::GeometricPrimitive> m_shape;
	float m_radius;
};