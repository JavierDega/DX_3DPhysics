#include "pch.h"
#include "Component.h"

//Constructor
Component::Component()
	: m_owner(nullptr)
{

}
//Destructor
Component::~Component()
{
	m_owner = nullptr;
	delete m_owner;
}