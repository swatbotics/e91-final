/* Hyper-Bare bones class I wrote to get 
   access to the m_Joint data of MotionModules. */

#include <stdio.h>
#include "Body.h"

using namespace Robot;

Body* Body::m_UniqueInstance = new Body();

Body::Body() 
{ 
}

Body::~Body() 
{
}

void Body::Initialize() {}

void Body::Process() {}
