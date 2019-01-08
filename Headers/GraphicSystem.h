#ifndef GRAPHICSYSTEM_H_
#define GRAPHICSYSTEM_H_

#include "System.h"
//#include "RigidbodyComponent.h"
#include <string>
#include <vector>


class GraphicSystem : public System {
private:
	/*Here will be the instance stored*/
	static GraphicSystem* m_instance;
	/*Private constructor to prevent instancing*/
	GraphicSystem();
public:
	~GraphicSystem();
	//Singleton
	static GraphicSystem* GetInstance();

	///Functions
	//@Events
	void Initialize(ID3D11Device1* device, ID3D11DeviceContext1 * deviceContext);
	void InitWindow(D3D11_VIEWPORT newScreenViewport);
	virtual void Update(float dt);
	virtual void Reset();
	

	///Variables
	//Graphics
	//Matrices - Coordinate spaces
};

#endif /*GRAPHICSYSTEM_H_*/