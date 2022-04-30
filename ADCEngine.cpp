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
#include "pico/sync.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "utilities.h"
#include <cstdio>

volatile uint guADCLine = 0;

// Defines
//*****************************************************************************
#define kK                  (1024)
#define kMaxBytes           (100 * kK)  // Of the available 264KB
#define kADCClock           (48.0e6f)   // 48MHz
#define kClocksPerSample    (96)        // Hardware limitation
#define kMinSampleRate      (10.0f)     // 10Hz
#define kMaxSampleRate      (kADCClock / (float)kClocksPerSample)   // 500kHz


// Local functions
//*****************************************************************************
static int siIRQCounter = 0;
void _dmaIRQHandler(void) {
    ADCEngine *pOwner(ADCEngine::mspSelf);
    if (nullptr == pOwner) {
        return; // Should never happen
    }
    if (nullptr == pOwner->mpDMAFrame) {
        return; // Also should never happen...
    }

guADCLine = __LINE__;
    // Test for valid DMA interrupt
    uint uDMAChannel(pOwner->muDMAChannel);
    if (false == dma_channel_get_irq0_status(uDMAChannel)) {
        return; // Spurious interrupt...
    }

    // Place just completed DMA frame on to end of signal list
    guADCLine = __LINE__; 
    pOwner->mpDMAFrame->mpNext = nullptr;
    if (nullptr == pOwner->mpSignalListHead) {
        pOwner->mpSignalListHead = pOwner->mpSignalListTail = pOwner->mpDMAFrame;
    } else {
        pOwner->mpSignalListTail->mpNext = pOwner->mpDMAFrame;
        pOwner->mpSignalListTail = pOwner->mpDMAFrame;
    }
    pOwner->mpDMAFrame = nullptr;   // Clear the target frame

    // Find the next target frame - from free list, or overwrite oldest in
    // signal list if there are none left (consumer not keeping up...)
    if (pOwner->mpFreeListHead != nullptr) {
        pOwner->mpDMAFrame = pOwner->mpFreeListHead;
        pOwner->mpFreeListHead = pOwner->mpDMAFrame->mpNext;
        pOwner->mpDMAFrame->mpNext = nullptr;

    } else if (pOwner->mpSignalListHead != nullptr) {
        pOwner->mpDMAFrame = pOwner->mpSignalListHead;
        pOwner->mpSignalListHead = pOwner->mpDMAFrame->mpNext;
        pOwner->mpDMAFrame->mpNext = nullptr;
        if (nullptr == pOwner->mpSignalListHead) {
            pOwner->mpSignalListTail = nullptr;
        }
    }

    if (nullptr == pOwner->mpDMAFrame) {
        return; // Should be impossible - protect from stupid values below
    }

    // Set up for next transfer - and start
guADCLine = __LINE__;
    pOwner->mpDMAFrame->muSequence = pOwner->muSequence++;	// Set next sequence number

guADCLine = __LINE__;
    dma_channel_set_write_addr(uDMAChannel, pOwner->mpDMAFrame->mpSamples, false);
    dma_channel_set_trans_count(uDMAChannel, pOwner->mpDMAFrame->muCount, true); 
guADCLine = __LINE__;

    // Clear the interrupt
    dma_hw->ints0 = 0x1 << uDMAChannel;
}

static void _configureDMAChannel(uint uChannel, uint8_t *pTarget, uint uTargetSize) {
     // Grab and manipulate default configuration
    dma_channel_config cDMAConfig(dma_channel_get_default_config(uChannel));

    // 8 bits per, constant source address, changing write address
    channel_config_set_transfer_data_size(&cDMAConfig, DMA_SIZE_8);
    channel_config_set_read_increment(&cDMAConfig, false);
    channel_config_set_write_increment(&cDMAConfig, true);
    channel_config_set_dreq(&cDMAConfig, DREQ_ADC);    // DMA request from ADC

    // Apply the configuration.
    // In case dma_channel_configure baulks on nullptr, zero for transfer parameters
    static uint8_t spDummyTarget[10];
    dma_channel_configure(uChannel, &cDMAConfig,
        pTarget,        // dst
        &adc_hw->fifo,  //src
        uTargetSize,    // transfer count
        false           // do not start immediately
    );

    channel_config_set_irq_quiet(&cDMAConfig, false);
}


// Class methods
//*****************************************************************************
// ADCEngine::
// Implementation of ADCEngine class.
//-----------------------------------------------------------------------------
ADCEngine *ADCEngine::mspSelf = nullptr; // Singleton pointer declaration

// Constructor
ADCEngine::ADCEngine(
    uint uADCChannel, float fSampleRateHz,
    float fBufferTimeSec, uint uFrames) :
    mpSignalListHead(nullptr), mpSignalListTail(nullptr),
    mpFreeListHead(nullptr), mpDMAFrame(nullptr),
    mbIsValid(false), mbIsRunning(false) {
 guADCLine = __LINE__;
    // Singleton protection and self-pointer (static)
    if (mspSelf != nullptr) {
        return;
    }
    mspSelf = this;
 guADCLine = __LINE__;
    // Pull sample rate into range if needed and compute clock divisor
    fSampleRateHz = (fSampleRateHz < kMinSampleRate)?kMinSampleRate:((fSampleRateHz > kMaxSampleRate)?kMaxSampleRate:fSampleRateHz);
    muClockDivisor = (uint)roundf(kADCClock / fSampleRateHz);
    mfSampleRateHz = kADCClock / (float)muClockDivisor;
    //printf("Sample Rate Hz = %.0f, muClockDivisor=%d\r\n", mfSampleRateHz, muClockDivisor);
 guADCLine = __LINE__;
    // Compute buffer dimensions (note that a sample is 8-bits - ie. a byte)
    uint uSamples((uint)roundf(fBufferTimeSec * fSampleRateHz));
    uSamples = (uSamples > kMaxBytes)?kMaxBytes:uSamples;
    uint uFrameSize(uSamples / uFrames);
    uSamples = uFrameSize * uFrames;  // Make all equal size and correct overall size (down)

    //printf("uSamples = %d, uFrameSize = %d, uFrames = %d\r\n", uSamples, uFrameSize, uFrames);
    //printf("Frame time = %.0f msec\r\n", ((float)uFrameSize / mfSampleRateHz) * 1000.0f);
 guADCLine = __LINE__;
    // Setup free list - signal list is empty
 guADCLine = __LINE__;
   for(uint i=0; i<uFrames; i++) {
       //printf("allocating frame %d\r\n", i);
 guADCLine = __LINE__;
        Frame *pNewFrame(new Frame(uFrameSize));
 guADCLine = __LINE__;
        pNewFrame->mpNext = mpFreeListHead;
        mpFreeListHead = pNewFrame;
guADCLine = __LINE__;
    }
 guADCLine = __LINE__;

    // Initialize critical section
    critical_section_init(&mcStoreLock);

    // Setup ADC resouces
    uADCChannel = (uADCChannel>2)?3:uADCChannel;
    adc_gpio_init(26 + uADCChannel);    // Allowable are 26-29
    adc_init();
    adc_select_input(uADCChannel);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
    adc_set_clkdiv(muClockDivisor);

    //printf("uADCChannel = %d, muClockDivisor=%d\r\n", uADCChannel, muClockDivisor);

    // Claim DMA resources (panic and fail if not available)
    muDMAChannel = dma_claim_unused_channel(true);

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
        Frame *pFrame(mpSignalListHead);
        while(pFrame != nullptr) {
            Frame *pLast(pFrame);
            pFrame = pFrame->mpNext;
            delete pLast;
        }
        mpSignalListHead = mpSignalListTail = nullptr;

        pFrame = mpFreeListHead;
        while(pFrame != nullptr) {
            Frame *pLast(pFrame);
            pFrame = pFrame->mpNext;
            delete pLast;
        }
        mpFreeListHead = nullptr;
    }

    // De-initialize critical section
    critical_section_deinit(&mcStoreLock);

    // And flag we are clean
    mspSelf = nullptr;
}

// Return actual (computed) sample rate
float ADCEngine::getSampleRateHz(void) const {
    return mfSampleRateHz;
}

// Check status flags
bool ADCEngine::isValid(void) const {
    return mbIsValid;
}

bool ADCEngine::isRunning(void) const {
    return mbIsRunning;
}

/// Start/stop capture
bool ADCEngine::setActive(bool bActive) {
    if (bActive == mbIsRunning) {
        return true;
    }

guADCLine = __LINE__;
   if (true == bActive) {
        // Discard all signal buffers back into free list
guADCLine = __LINE__;
        {   ScopedLock cLock(&mcStoreLock);
            Frame *pFrame(mpSignalListHead);
            while(pFrame != nullptr) {
                Frame *pNext(pFrame->mpNext);
                pFrame->mpNext = mpFreeListHead;
                mpFreeListHead = pFrame;
            }
            mpSignalListHead = mpSignalListTail = nullptr;
        }
        muSequence = 0; // Starting again...
guADCLine = __LINE__;
               
        // Setup initial transfer target
        {   ScopedLock cLock(&mcStoreLock);
            mpDMAFrame = mpFreeListHead;
            mpFreeListHead = mpDMAFrame->mpNext;
            mpDMAFrame->mpNext = nullptr;
            mpDMAFrame->muSequence = muSequence++;
        }
guADCLine = __LINE__;

        // Configure DMA channel
        _configureDMAChannel(muDMAChannel, mpDMAFrame->mpSamples, mpDMAFrame->muCount);
    
        // Configure interrupts
guADCLine = __LINE__;
        dma_channel_set_irq0_enabled(muDMAChannel, true);
        irq_set_exclusive_handler(DMA_IRQ_0, _dmaIRQHandler);
        irq_set_enabled(DMA_IRQ_0, true);

        // Start muDMAChannel
 guADCLine = __LINE__;
        dma_channel_set_write_addr(muDMAChannel, mpDMAFrame->mpSamples, false);
        dma_channel_set_trans_count(muDMAChannel, mpDMAFrame->muCount, true); 
    
        // And kick off the adc
 guADCLine = __LINE__;
        adc_run(true);

        mbIsRunning = true;
        return true;

    } else {
        // Stop everything - switch off and disable interrupts
        adc_run(false);
        dma_channel_abort(muDMAChannel);
        irq_set_enabled(DMA_IRQ_0, false);
        dma_channel_set_irq0_enabled(muDMAChannel, false);

        mbIsRunning = false;
        return true;
    }
}

/// Attach a consumer to process the next frame. If no frame is available,
/// returns 'false' and cConsumer::process will not be called.
bool ADCEngine::processFrame(Consumer *pConsumer) {
    if (nullptr == pConsumer) {
        return false;
    }
    
    Frame *pFrame(nullptr);   // Grab signal frame to process
    {   ScopedLock cLock(&mcStoreLock);
        if (mpSignalListHead != nullptr) {
            pFrame = mpSignalListHead;
            mpSignalListHead = mpSignalListHead->mpNext;
            if (nullptr == mpSignalListHead) {
                mpSignalListTail = nullptr;
            }
        }
    }
    if (nullptr == pFrame) {
        return false;
    }

    // Consumer now has exclusive access to frame
    //printf("Calling Consumer.process\r\n");
    pConsumer->process(*pFrame);

    // Release consumed frame into free list
    {   ScopedLock cLock(&mcStoreLock);
        pFrame->mpNext = mpFreeListHead;
        mpFreeListHead = pFrame;
    }

    return true;
}
