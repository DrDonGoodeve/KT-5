/******************************************************************************
 * ADCEngine.cpp
 * 
 * Implementation file for ADCEngine class.
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 31Mar2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

// Includes
//*****************************************************************************
#include "ADCEngine.h"
#include <math.h>


// Defines
//*****************************************************************************
#define kK                  (1024)
#define kMaxBytes           (100 * kK)  // Of the available 264KB
#define kADCClock           (48.0e6f)   // 48MHz
#define kClocksPerSample    (96)        // Hardware
#define kMinSampleRate      (10.0f)     // 10Hz
#define kMaxSampleRate      (kADCClock / (float)kClocksPerSample)   // 500kHz


// Local functions
//*****************************************************************************


// Local classes
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
}


// Class methods
//*****************************************************************************
// ADCEngine::
// Implementation of ADCEngine class.
//-----------------------------------------------------------------------------
static ADCEngine *ADCEngine::mspSelf = nullptr; // Singleton pointer declaration

// Constructor
ADCEngine::ADCEngine(
    uint uADCChannel, float fSampleRateHz,
    float fBufferTimeSec, uint uFrames) :
    mbIsValid(false), mbIsRunning(false) {

    // Singleton protection and self-pointer (static)
    if (mspSelf != nullptr) {
        return;
    }
    mspSelf = this;

    // Pull sample rate into range if needed and compute clock divisor
    fSampleRateHz = (fSampleRateHz < kMinSampleRate)?kMinSampleRate:((fSampleRateHz > kMaxSampleRate)?kMaxSampleRate:fSampleRateHz);
    muClockDivisor = (uint)roundf(kADCClock / fSampleRateHz);
    mfSampleRateHz = kADCClock / (float)muClockDiv;

    // Compute buffer dimensions (note that a sample is 8-bits - ie. a byte)
    uint uSamples((uint)roundf(fBufferTimeSec * fSampleRateHz));
    uSamples = (uSamples > kMaxBytes)?kMaxBytes:uSamples;
    uint uFrameSize(uSamples / uFrames);
    uSamples = uFrameSize * uFrames;  // Make all equal size and correct overall size (down)
    mpBuffer = new uint8_t[uSamples];
    if (nullptr == mpBuffer) {
        mspSelf = nullptr;
        return;
    }

    // Setup free list - signal list is empty
    for(uint i=0; i<uFrames; i++) {
        mlFreeList.push_back(Frame(mpBuffer + (i*uFrameSize), uFrameSize));
    }

    // Initialize critical section
    critical_section_init(&mcStoreLock);

    // So far, so good
    mbIsValid = true;
}

/// Destructor - deallocate resources and shut down
ADCEngine::~ADCEngine() {
    // stop everything and free up all resources
    setActive(false);

    // Free DMA resources

    // Free ADC/GPIO resources
    
    // Free all memory resources
    {   ScopedLock cLock(&mcStoreLock);
        mlSignalBuffer.clear();
        mlFreeList.clear();
        delete []mpBuffer;
    }

    // De-initialize critical section
    critical_section_deinit(&mcStoreLock);

    // And flag we are clean
    mspSelf = nullptr;
}

// Check status flags
bool ADCEngine::isValid(void) const {
    return mbIsValid;
}

bool ADCEngine::isRunning(void) const {
    return mbIsRunning;
}

/// Start/stop capture
bool ADCEngine::setActive(bool bActive=true) {
    if (bActive == mbIsRunning) {
        return true;
    }

    if (true == bActive) {
        // Set everything going if resources allow
    } else {
        // Stop everything
    }
    mbIsRunning = bActive;
}

/// Attach a consumer to process the next frame. If no frame is available,
/// returns 'false' and cConsumer::process will not be called.
bool ADCEngine::processFrame(Consumer &cConsumer) {
    if ()
}
