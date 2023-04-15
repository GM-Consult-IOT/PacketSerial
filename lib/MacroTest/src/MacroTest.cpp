#include "MacroTest.h"

Library::Library(){}

#if PS_DEBUG 

    void Library::print(){
        Serial.println("Printing");
    }

#endif // DEBUG