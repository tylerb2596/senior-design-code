//SBD Metallic Debris Collector Project
//Stanley Black and Decker
//Georgia Institute of Technology
//created by Tyler Brown

//This is a base class used to abstract out commonalities in the various state
//class definitions.

#ifndef STATE
#define STATE

#include "state_type.h"

//class definition
class State {

//public members of the class
public:

	state_type type;

};

#endif