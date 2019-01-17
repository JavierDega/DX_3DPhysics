#ifndef INPUTSYSTEM_H_
#define INPUTSYSTEM_H_
#include "System.h"

class InputSystem :
	public System
{
private:
	/*Here will be the instance stored*/
	static InputSystem* m_instance;
	/*Private constructor to prevent instancing*/
	InputSystem();
public:
	~InputSystem();
	//Singleton
	static InputSystem* GetInstance();
	///Functions
	//@Events
	void Initialize(ID3D11Device1* device = nullptr, ID3D11DeviceContext1 * deviceContext = nullptr);
	void InitWindow(D3D11_VIEWPORT screenViewport = D3D11_VIEWPORT{ 0, 0, 0, 0 });
	virtual void Update(float dt = 0);
	virtual void Reset();
	//Utility
	void SetMouseWindow(HWND window);
	void ScanInput();

	//Variables
	//@Logic components point to these variables
	DirectX::SimpleMath::Vector3 * m_cam;
	DirectX::SimpleMath::Vector3 * m_look;
	float m_pitch, m_yaw;
	std::unique_ptr<DirectX::Keyboard> m_keyboard;
	std::unique_ptr<DirectX::Mouse> m_mouse;

};
#endif /*INPUTSYSTEM_H_*/