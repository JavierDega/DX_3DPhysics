#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Component.h"
#include <vector>

class System
{
public:
	System();
	///All systems need a virtual destructor to have their destructor called 
	virtual ~System();
	// It's good practice to separate the construction and initialization code.
	virtual void Initialize() = 0;
	// All systems must update each game loop
	virtual void Update(float dt) = 0;
	// This recieves any messages sent to the core engine in Engine.cpp
	//virtual void SendMessage( ComponentMessage *msg ) = 0;
};

#endif /*SYSTEM_H_*/