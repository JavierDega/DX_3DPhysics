#include "pch.h"
#include "GraphicSystem.h"
#include <stdio.h>

using namespace std;

//Instance
GraphicSystem * GraphicSystem::m_instance = NULL;
GraphicSystem * GraphicSystem::GetInstance()
{
	//Singleton
	if (m_instance == NULL)
	{
		m_instance = new GraphicSystem();
	}
	return m_instance;
}

//Constructor
GraphicSystem::GraphicSystem() {
}
//Destructor (Singleton so..?)
GraphicSystem::~GraphicSystem() {

}
//Init
void GraphicSystem::Initialize(ID3D11Device1* device, ID3D11DeviceContext1 * deviceContext) {
}
void GraphicSystem::InitWindow(D3D11_VIEWPORT newScreenViewport)
{
}
//Update
void GraphicSystem::Update(float dt) {
	///DRAW ALL RENDERABLES:
}

void GraphicSystem::Reset()
{
}
