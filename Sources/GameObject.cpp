#include "pch.h"
#include "GameObject.h"

using namespace std;
using namespace DirectX;
using namespace SimpleMath;

//Constructor
GameObject::GameObject(std::string name, Vector3 position)
	: m_name(name), m_transform(position)
{

}
//Destructor
GameObject::~GameObject()
{

}
//AddComp
void GameObject::AddComponent(Component * component) {
	//We add the component to the vector, and we set reference (Just in case)
	component->m_owner = this;
	m_components.push_back(component);
}

void GameObject::Send(ComponentMessage msg)
{
	for (int i = 0; i < m_components.size(); i++)
	{
		m_components[i]->Receive(msg);
	}
}

void GameObject::RefreshComponentAddresses()
{
	//@Redefine pointer in component->m_owner
	for (unsigned int i = 0; i < m_components.size(); i++) {
		m_components[i]->m_owner = this;
	}
}
