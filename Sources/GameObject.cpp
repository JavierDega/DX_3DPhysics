#include "pch.h"
#include "GameObject.h"

using namespace std;
using namespace DirectX;
using namespace SimpleMath;

//Constructor
GameObject::GameObject(std::string name, Vector3 position, Quaternion rotation, Vector3 scale)
	: m_name(name), m_transform(position, rotation, scale)
{
}
//Destructor
GameObject::~GameObject()
{
	while (!m_components.empty()) {
		Component * curComp = m_components.back();
		m_components.pop_back();
		delete curComp;
		curComp = nullptr;
	}
	bool debug;
}
///We add the component to the vector, and we set reference
void GameObject::AddComponent(Component * component) {
	component->m_owner = this;
	m_components.push_back(component);
}
///Broadcast message to all children components
void GameObject::Send(ComponentMessage msg)
{
	for (int i = 0; i < m_components.size(); i++)
	{
		m_components[i]->Receive(msg);
	}
}
///Refresh pointer to parent gameObject in all component children
void GameObject::RefreshComponentAddresses()
{
	for (unsigned int i = 0; i < m_components.size(); i++) {
		m_components[i]->m_owner = this;
	}
}
