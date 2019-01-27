#ifndef GAMEOBJECT_H_
#define GAMEOBJECT_H_


#include "Component.h"
#include "TransformComponent.h"
#include <string>
#include <vector>

///Stores components, has transform and ID
class GameObject {
public:
	GameObject(std::string name = "Default name", DirectX::SimpleMath::Vector3 position = DirectX::SimpleMath::Vector3( 0, 0, 0 ));
	~GameObject();

	//Functions
	void AddComponent(Component * component);
	//Messaging
	virtual void Send(ComponentMessage msg);
	void RefreshComponentAddresses();

	//Variables
	std::string m_name; //@HashIds?SlotMaps?Handles?
	TransformComponent m_transform;
	std::vector<Component *> m_components;
};
#endif /*GAMEOBJECT_H_*/