/******************************************************************************
 * utilities.h
 * 
 * Various useful definitions, functions and classes.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 10Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _UTILITIES_
#define _UTILITIES_


// Includes
//*****************************************************************************
#include "pico/stdlib.h"
#include "pico/sync.h"


// Macros
//*****************************************************************************


// Useful types
//*****************************************************************************
// Simple scoped object for holding and releasing a critical section as
// required.
//-----------------------------------------------------------------------------
class ScopedLock {
    private:
        critical_section_t *mpCriticalSection;
        
    public:
        ScopedLock(critical_section_t *pCriticalSection) :
            mpCriticalSection(pCriticalSection) {
            critical_section_enter_blocking(mpCriticalSection);
        }

        ~ScopedLock() {
            critical_section_exit(mpCriticalSection);
        }
};


#endif // _UTILITIES_
