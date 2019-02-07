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
	LoadScene(1);
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
int ObjectSystem::AddObject(GameObject * obj) {
	//@Not cache coherent at all
	m_objectList.push_back(obj);
	return 0;
}
//Remove Objs
void ObjectSystem::RemoveAllObjects() {
	//Safety check
	while (!m_objectList.empty()) {
		GameObject * curObj = m_objectList.back();
		delete curObj;
		m_objectList.pop_back();
	}
}
void ObjectSystem::RemoveObject(GameObject * object)
{
	//Remove specific object. ideally called from Component itself.
	//i.e. Sphere collider requests removing attached gameObject when it is put on a hole.
	//We give an index? a pointer to the gameObject?
	for (unsigned int i = 0; i < m_objectList.size(); i++) {
		//Compare addresses
		if (object == (m_objectList[i])) {
			//Remove
			//Swap and pop approach (NOT Needed)
			if (i < m_objectList.size() - 1) {
				swap(m_objectList[i], m_objectList.back());
			}
			GameObject * curObj = m_objectList.back();
			delete curObj;
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
		GameObject * curObj = m_objectList[i];
		//Find mesh components
		for (unsigned int j = 0; j < curObj->m_components.size(); j++) {
			//Dynamic casting to identify type;
			RigidbodyComponent * RbComp = dynamic_cast<RigidbodyComponent *>(curObj->m_components[j]);
			if (RbComp) {
				RbCompList.push_back(RbComp);
			}
		}
	}
	return RbCompList;
}

void ObjectSystem::LoadScene( unsigned int index)
{
	RemoveAllObjects();
	switch (index) {
		case 0:
		{
			//Create some spheres
			GameObject * mySphere = new GameObject("Sphere", Vector3(-1.5, 2.0f, 6.f));
			RigidbodyComponent * rb1 = new RigidbodyComponent(0.5f, 1.0f, false, Colors::Red);
			rb1->m_force = Vector3(100.f, 0.f, 0.f);
			mySphere->AddComponent(rb1);
			AddObject(mySphere);

			GameObject * mySphere2 = new GameObject("Sphere2", Vector3(1.5, 2.0f, 6.f));
			RigidbodyComponent * rb2 = new RigidbodyComponent(0.5f, 1.0f, false, Colors::Red);
			rb2->m_force = Vector3(-100.f, 0.f, 0.f);
			mySphere2->AddComponent(rb2);
			AddObject(mySphere2);


			GameObject * mySphere3 = new GameObject("Sphere3", Vector3(-0.5, 10.0f, 10.f));
			mySphere3->AddComponent(new RigidbodyComponent(0.5f, 1.0f, false, Colors::Red));
			AddObject(mySphere3);

			GameObject * mySphere4 = new GameObject("Sphere4", Vector3(0.5, 10.0f, 10.f));
			mySphere4->AddComponent(new RigidbodyComponent(0.5f, 1.f, false, Colors::Red));
			AddObject(mySphere4);

			GameObject * mySphere5 = new GameObject("Sphere5", Vector3(-0.5, 10.0f, 15.f));
			mySphere5->AddComponent(new RigidbodyComponent(0.5f, 1.0f, false, Colors::Red));
			AddObject(mySphere5);

			GameObject * mySphere6 = new GameObject("Sphere6", Vector3(0.5, 10.0f, 15.f));
			mySphere6->AddComponent(new RigidbodyComponent(0.5f, 1.f, false, Colors::Red));
			AddObject(mySphere6);

			GameObject * mySphere7 = new GameObject("Sphere7", Vector3(-0.5, 10.0f, 0.f));
			mySphere7->AddComponent(new RigidbodyComponent(0.5f, 1.0f, false, Colors::Red));
			AddObject(mySphere7);

			GameObject * mySphere8 = new GameObject("Sphere8", Vector3(0.5, 10.0f, 0.f));
			mySphere8->AddComponent(new RigidbodyComponent(0.5f, 1.f, false, Colors::Red));
			AddObject(mySphere8);

			for (int i = 0; i < 100; i++) {
				GameObject * mySphere = new GameObject("Sphere" + i, Vector3(0.5, 15.0f + 2 * i, 0.f));
				mySphere->AddComponent(new RigidbodyComponent(0.5f, 1.f, false, Colors::Red));
				AddObject(mySphere);
			}

			///SPHERE BOUNDS
			//Mid bottom
			GameObject * mySphereBound = new GameObject("SphereBound", Vector3(0, -10.f, 5.f));
			mySphereBound->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound);

			GameObject * mySphereBound2 = new GameObject("SphereBound2", Vector3(-15.5f, 1.f, 5.f));
			mySphereBound2->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound2);

			GameObject * mySphereBound3 = new GameObject("SphereBound3", Vector3(15.5f, 1.f, 5.f));
			mySphereBound3->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound3);

			//Front bottom
			GameObject * mySphereBound4 = new GameObject("SphereBound4", Vector3(0, -10.f, -5.f));
			mySphereBound4->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound4);

			GameObject * mySphereBound5 = new GameObject("SphereBound5", Vector3(-15.5f, 1.f, -5.f));
			mySphereBound5->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound5);

			GameObject * mySphereBound6 = new GameObject("SphereBound6", Vector3(15.5f, 1.f, -5.f));
			mySphereBound6->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound6);

			//Back bottom
			GameObject * mySphereBound7 = new GameObject("SphereBound7", Vector3(0, -10.f, 15.f));
			mySphereBound7->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound7);

			GameObject * mySphereBound8 = new GameObject("SphereBound8", Vector3(-15.5f, 1.f, 15.f));
			mySphereBound8->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound8);

			GameObject * mySphereBound9 = new GameObject("SphereBound9", Vector3(15.5f, 1.f, 15.f));
			mySphereBound9->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound9);

			//Back and front
			GameObject * mySphereBound10 = new GameObject("SphereBound10", Vector3(0, 1.f, 25.f));
			mySphereBound10->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound10);

			GameObject * mySphereBound11 = new GameObject("SphereBound11", Vector3(0, 1.f, -15.f));
			mySphereBound11->AddComponent(new RigidbodyComponent(10.f, 100.f, true));
			AddObject(mySphereBound11);

		}
		break;
		case 1:
		{
			//Build OrientedBoundingBoxes, check aabbs
			GameObject * myObb = new GameObject("OBB", Vector3( 5, -9.45, 10.f ), Quaternion::CreateFromAxisAngle(Vector3(0,0,1), 2.0f));
			myObb->AddComponent(new RigidbodyComponent( Vector3(1.231,15,10), 10.f, true ));
			AddObject(myObb);

			GameObject * mySphere2 = new GameObject("Sphere2", Vector3( 1, 6, 10.f));
			mySphere2->AddComponent(new RigidbodyComponent(0.5f, 10.f, false));
			AddObject(mySphere2);


		}
		break;
	}
	

}
