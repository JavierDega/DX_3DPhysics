#ifndef TRANSFORMCOMPONENT_H_
#define TRANSFORMCOMPONENT_H_

#include "Component.h"

class TransformComponent : public Component {
public:
	//Functions
	TransformComponent(DirectX::SimpleMath::Vector3 position = DirectX::SimpleMath::Vector3::Zero,
		DirectX::SimpleMath::Quaternion rotation = DirectX::SimpleMath::Quaternion::Identity);//@Scale only affects the graphical component?
	~TransformComponent();
	//Messaging
	virtual bool Receive(ComponentMessage msg);
	//Variables
	DirectX::SimpleMath::Vector3 m_position;
	DirectX::SimpleMath::Quaternion m_rotation;
};
#endif /*TRANSFORMCOMPONENT_H_*/