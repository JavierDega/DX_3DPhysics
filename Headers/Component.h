#ifndef COMPONENT_H_
#define COMPONENT_H_

class GameObject;

enum ComponentMessage {
	MSG_ONE,
	MSG_TWO,
	MSG_THREE
};

class Component {
public:
	//USE DYNAMIC CASTS FOR TYPE IDENTIFICATION?
	Component();
	virtual ~Component();
	//Entity level messaging
	virtual bool Receive (ComponentMessage msg) = 0;

	//Variables
	GameObject * m_owner;
};
#endif /*COMPONENT_H_*/