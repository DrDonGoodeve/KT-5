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
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"


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
void _dmaIRQHandler(void) {
    ADCEngine *pOwner(ADCEngine::mspSelf);

    // Which channel interrupted (other channel is now running)
    uint uIdleDMAChannel(0), uActiveDMAChannel(0);
    if (true == dma_channel_get_irq0_status(pOwner->muDMAChannelA)) {
        uIdleDMAChannel = pOwner->muDMAChannelA;
        uActiveDMAChannel = pOwner->muDMAChannelB;
    } else if (true == dma_channel_get_irq0_status(pOwner->muDMAChannelB)) {
        uIdleDMAChannel = pOwner->muDMAChannelB;
        uActiveDMAChannel = pOwner->muDMAChannelA;
   } else {
        return; // Spurious interrupt...
    }

    // Find the next target buffer - from free list, or overwrite oldest in
    // signal if there are none left (consumer not keeping up...)
    ADCEngine::Frame cNewFrame;
    if (false == pOwner->mlFreeList.empty()) {
        cNewFrame = pOwner->mlFreeList.front();
        pOwner->mlFreeList.pop_front();
    } else if (false == pOwner->mlSignalBuffer.empty()) {
        cNewFrame = pOwner->mlSignalBuffer.front(); // Oldest signal discarded
        pOwner->mlSignalBuffer.pop_front();
    }
    if (true == cNewFrame.isEmpty()) {
        return; // Should be impossible - protect from using stupid values below...
    }

    // Set up for next transfer - do not start yet...
    cNewFrame.muSequence = pOwner->muSequence++;	// Set next sequence number
    dma_channel_set_write_addr(uIdleDMAChannel, cNewFrame.mpSamples, false);
    dma_channel_set_trans_count(uIdleDMAChannel, cNewFrame.muCount, false); 

    // If active channel is no longer active, trigger this channel
    // (This should be achieved by chaining however this reduces the race condition
    //  that for some reason the IRQ handler has not completed before the other channel
    //  has completed its transfer to a few cycles. In this case, the other channel interrupt
    //  should now be pending and so we will come back into this function again. My starting
    //  here will not intefere with this process.)
    if (false == dma_channel_is_busy(uActiveDMAChannel)) {
        dma_channel_start(uIdleDMAChannel);     // Idle no more...
    }
     
    // Clear the interrupt for uIdleDMAChannel
    dma_hw->ints0 = 0x1 << uIdleDMAChannel;
}


static void _configureDMAChannel(uint uThisChannel, uint uChainToDMAChannel) {
     // Grab and manipulate default configuration
    dma_channel_config cDMAConfig(dma_channel_get_default_config(uThisChannel));

    // 8 bits per, constant source address, changing write address
    channel_config_set_transfer_data_size(&cDMAConfig, DMA_SIZE_8);
    channel_config_set_read_increment(&cDMAConfig, false);
    channel_config_set_write_increment(&cDMAConfig, true);
    channel_config_set_dreq(&cDMAConfig, DREQ_ADC);    // DMA request from ADC

    // Apply the configuration.
    // In case dma_channel_configure baulks on nullptr, zero for transfer parameters
    static uint8_t spDummyTarget[10];
    dma_channel_configure(uThisChannel, &cDMAConfig,
        spDummyTarget,          // dst
        &adc_hw->fifo,          // src
        sizeof(spDummyTarget),  // transfer count
        false                   // do not start immediately
    );

    channel_config_set_chain_to(&cDMAConfig, uChainToDMAChannel);
    channel_config_set_irq_quiet(&cDMAConfig, false);
}


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
};


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
    mbIsValid(false), mbIsRunning(false) {

    // Singleton protection and self-pointer (static)
    if (mspSelf != nullptr) {
        return;
    }
    mspSelf = this;

    // Pull sample rate into range if needed and compute clock divisor
    fSampleRateHz = (fSampleRateHz < kMinSampleRate)?kMinSampleRate:((fSampleRateHz > kMaxSampleRate)?kMaxSampleRate:fSampleRateHz);
    muClockDivisor = (uint)roundf(kADCClock / fSampleRateHz);
    mfSampleRateHz = kADCClock / (float)muClockDivisor;

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
        Frame cFrame(mpBuffer + (i*uFrameSize), uFrameSize);
        mlFreeList.push_back(cFrame);
    }

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

    // Claim DMA resources (panic and fail if not available)
    muDMAChannelA = dma_claim_unused_channel(true);
    muDMAChannelB = dma_claim_unused_channel(true);

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

    if (true == bActive) {
        // Discard all signal buffers back into free list
        {   ScopedLock cLock(&mcStoreLock);
            while(false == mlSignalBuffer.empty()) {
                mlFreeList.push_back(mlSignalBuffer.front());
                mlSignalBuffer.pop_front();
            }
        }
        muSequence = 0; // Starting again...

        // Configure DMA channels
        _configureDMAChannel(muDMAChannelA, muDMAChannelB);
        _configureDMAChannel(muDMAChannelB, muDMAChannelA);

        // Configure interrupts
        dma_channel_set_irq0_enabled(muDMAChannelA, true);
        dma_channel_set_irq0_enabled(muDMAChannelB, true);
        irq_set_exclusive_handler(DMA_IRQ_0, _dmaIRQHandler);
        irq_set_enabled(DMA_IRQ_0, true);
               
        // Setup initial transfer targets
        Frame cFrameA, cFrameB;
        {   ScopedLock cLock(&mcStoreLock);
            cFrameA = mlFreeList.front();
            cFrameA.muSequence = muSequence++;
            mlFreeList.pop_front();
            cFrameB = mlFreeList.front();
            cFrameB.muSequence = muSequence++;
            mlFreeList.pop_front();
        }

        // Configure both channels and start muDMAChannelA
        dma_channel_set_write_addr(muDMAChannelB, cFrameB.mpSamples, false);
        dma_channel_set_trans_count(muDMAChannelB, cFrameB.muCount, false);
        dma_channel_set_write_addr(muDMAChannelA, cFrameA.mpSamples, false);
        dma_channel_set_trans_count(muDMAChannelB, cFrameA.muCount, true); // Go!

        mbIsRunning = true;
        return true;

    } else {
        // Stop everything - switch off and disable interrupts
        dma_channel_abort(muDMAChannelA);
        dma_channel_abort(muDMAChannelB);
        irq_set_enabled(DMA_IRQ_0, false);
        dma_channel_set_irq0_enabled(muDMAChannelA, false);
        dma_channel_set_irq0_enabled(muDMAChannelB, false);

        mbIsRunning = false;
        return true;
    }
}

/// Attach a consumer to process the next frame. If no frame is available,
/// returns 'false' and cConsumer::process will not be called.
bool ADCEngine::processFrame(Consumer &cConsumer) {
    Frame cFrame;   // Grab signal frame to process
    {   ScopedLock cLock(&mcStoreLock);
        if (false == mlSignalBuffer.empty()) {
            cFrame = mlSignalBuffer.front();
            mlSignalBuffer.pop_front();
        }
    }
    if (true == cFrame.isEmpty()) {
        return false;
    }

    // Consumer now has exclusive access to frame
    cConsumer.process(cFrame);

    // Release consumed frame
    {   ScopedLock cLock(&mcStoreLock);
        mlFreeList.push_front(cFrame);
    }

    return true;
}
