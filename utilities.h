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
#define TRACE           {printf("%d.", __LINE__); fflush(stdout);}
#define _arraysize(a)   (sizeof(a)/sizeof(a[0]))


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

// Trimming std::string
//-----------------------------------------------------------------------------
#include <algorithm> 
#include <cctype>
#include <locale>

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}


#endif // _UTILITIES_
