#include "pch.h"
#include "ObjectSystem.h"
#include "GraphicSystem.h"

using namespace std;
using namespace DirectX;
using namespace SimpleMath;

//Instance
ObjectSystem * ObjectSystem::m_instance = NULL;
ObjectSystem * ObjectSystem::GetInstance()
{
	//Singleton
	if (m_instance == NULL)
	{
		m_instance = new ObjectSystem();
	}
	return m_instance;
}

//Constructor
ObjectSystem::ObjectSystem() {
	//m_objectList.reserve(10);
}
//Destructor @Singleton so?
ObjectSystem::~ObjectSystem() {

}
//Init
void ObjectSystem::Initialize(ID3D11Device1* device, ID3D11DeviceContext1 * deviceContext)
{
	//@What do here? LoadSceneOne
	LoadScene1();
}
void ObjectSystem::InitWindow(D3D11_VIEWPORT screenViewport)
{
	//@What do here?
}
//Update
void ObjectSystem::Update(float dt)
{
	//@What do here?
}
void ObjectSystem::Reset()
{
	//@What do here?
}
//Add Obj
GameObject * ObjectSystem::AddObject(string name, Vector3 position) {
	//Add to array
	bool m_reallocate = false;
	//Check if capacity is greater than size
	if (m_objectList.size() == m_objectList.capacity()) {
		//This means its going to reallocate
		m_reallocate = true;
	}
	m_objectList.push_back(GameObject(name, position));

	//Has just reallocated
	if (m_reallocate) {
		for (unsigned int i = 0; i < m_objectList.size(); i++) {
			m_objectList[i].RefreshComponentAddresses();
		}
	}
	//Return address
	return &m_objectList.back();
}
//Remove Objs
void ObjectSystem::RemoveAllObjects() {
	//Safety check
}
void ObjectSystem::RemoveObject(GameObject * object)
{
	//Remove specific object. ideally called from Component itself.
	//i.e. Sphere collider requests removing attached gameObject when it is put on a hole.
	//We give an index? a pointer to the gameObject?
	for (unsigned int i = 0; i < m_objectList.size(); i++) {
		//Compare addresses
		if (object == &(m_objectList[i])) {
			//Remove
			//Swap and pop approach
			if (i < m_objectList.size() - 1) {
				swap(m_objectList[i], m_objectList.back());
				m_objectList[i].RefreshComponentAddresses();
			}
			m_objectList.pop_back();
		}
	}
}
//Get component lists
vector< RigidbodyComponent * > ObjectSystem::GetRigidbodyComponentList() {
	//Iterate through gameobjects, find components through i.e dynamic_casts
	//Add pointers to the vector, return such vector
	vector< RigidbodyComponent * > RbCompList;
	//@Beware of vectors dynamically moving instances in memory
	for (unsigned int i = 0; i < m_objectList.size(); i++) {
		GameObject curObj = m_objectList[i];
		//Find mesh components
		for (unsigned int j = 0; j < curObj.m_components.size(); j++) {
			//Dynamic casting to identify type;
			RigidbodyComponent * RbComp = dynamic_cast<RigidbodyComponent *>(curObj.m_components[j]);
			if (RbComp) {
				RbCompList.push_back(RbComp);
			}
		}
	}
	return RbCompList;
}

void ObjectSystem::LoadScene1()
{
	//Create some spheres
	GameObject * mySphere = AddObject( "Sphere", Vector3( 0, 0, -10.0f));
	mySphere->AddComponent(new RigidbodyComponent());

	//@Same bug as Wii, dynamic allocation
	GameObject * mySphere2 = AddObject("Sphere2", Vector3(0, 2, -10.0f));
	mySphere2->AddComponent(new RigidbodyComponent());

	RemoveObject(mySphere2);
}
