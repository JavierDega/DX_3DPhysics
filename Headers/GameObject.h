#ifndef GAMEOBJECT_H_
#define GAMEOBJECT_H_


#include "Component.h"
#include "TransformComponent.h"
#include <string>
#include <vector>

//@Stores components, has string id
class GameObject {
public:
	GameObject(std::string name = "Default name", DirectX::SimpleMath::Vector3 position = DirectX::SimpleMath::Vector3( 0, 0, 0 ));
	~GameObject();
	//Functions
	void AddComponent(Component * component);
	//Messages
	//virtual void Send(ComponentMessage * msg);
	//virtual Component * FindComponent(ComponentType type);
	//Variables
	std::string m_name;
	//All gameobjects have a transform component
	TransformComponent m_transform;
	//Cache coherency avoiding 'new'
	std::vector<Component *> m_components;
};
#endif /*GAMEOBJECT_H_*/