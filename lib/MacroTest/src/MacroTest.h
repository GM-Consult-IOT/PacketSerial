
#ifndef __LIBRARY
#define __LIBRARY

// #include "main_config.h"
#include <Arduino.h>

/* uncomment the line below  or add it to your main.cpp above all includes to 
* enable printing of debug information*/


#ifndef PS_DEBUG
/// @brief Set to true to enable printing of debug information to Serial.
#define PS_DEBUG false
#endif // PS_DEBUG


class Library{

public:

Library();

#if PS_DEBUG 

void print();

#endif

};


#endif // __LIBRARY