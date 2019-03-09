#ifndef OBJECTSYSTEM_H_
#define OBJECTSYSTEM_H_

#include "System.h"
#include "GameObject.h"
#include "RigidbodyComponent.h"
#include <vector>

///OBJECT FACTORY/ STORES POINTERS TO GAMEOBJECTS AND ALLOWS US TO FILTER COMPONENTS
//FOR THE DIFFERENT SYSTEMS
class ObjectSystem : public System {
private:
	/*Here will be the instance stored*/
	static ObjectSystem* m_instance;
	/*Private constructor to prevent instancing*/
	ObjectSystem();
public:
	~ObjectSystem();
	//Singleton
	static ObjectSystem* GetInstance();
	//Funcs
	//@Events
	void Initialize(ID3D11Device1* device = nullptr, ID3D11DeviceContext1 * deviceContext = nullptr);
	void InitWindow(D3D11_VIEWPORT screenViewport = D3D11_VIEWPORT{ 0, 0, 0, 0 });
	virtual void Update(float dt = 0);
	virtual void Reset();
	//Utility
	int AddObject(GameObject * obj);
	void RemoveAllObjects();
	void RemoveObject(GameObject * object);
	std::vector< RigidbodyComponent * > GetRigidbodyComponentList();
	void LoadScene(unsigned int index);

	//Variables
	std::vector<GameObject * > m_objectList;
	int m_sceneIndex;
};

#endif /*OBJECTSYSTEM_H_*/