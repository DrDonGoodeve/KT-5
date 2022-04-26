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

    // Test for valid DMA interrupt
    uint uDMAChannel(pOwner->muDMAChannelA);
    if (false == dma_channel_get_irq0_status(uDMAChannel)) {
        return; // Spurious interrupt...
    }

    // Clear the interrupt for uIdleDMAChannel
    dma_hw->ints0 = 0x1 << uDMAChannel;
    //printf("%d: Completed DMA 0x%08x\r\n", siIRQCounter++, pOwner->mcDMAFrame.mpSamples);

    // Pull from mlDMAList and place in signal list
    pOwner->mlSignalBuffer.push_back(pOwner->mcDMAFrame);
    pOwner->mcDMAFrame.clear();

    // Find the next target buffer - from free list, or overwrite oldest in
    // signal if there are none left (consumer not keeping up...)
    ADCEngine::Frame cFreshFrame;
    if (false == pOwner->mlFreeList.empty()) {
        cFreshFrame = pOwner->mlFreeList.front();
        pOwner->mlFreeList.pop_front();
    } else if (false == pOwner->mlSignalBuffer.empty()) {
        cFreshFrame = pOwner->mlSignalBuffer.front(); // Oldest signal discarded
        pOwner->mlSignalBuffer.pop_front();
    }
    if (true == cFreshFrame.isEmpty()) {
        return; // Should be impossible - protect from using stupid values below...
    }
    //printf("!");

    // Set up for next transfer - and start
    cFreshFrame.muSequence = pOwner->muSequence++;	// Set next sequence number
    pOwner->mcDMAFrame = cFreshFrame;

    dma_channel_set_write_addr(uDMAChannel, cFreshFrame.mpSamples, false);
    dma_channel_set_trans_count(uDMAChannel, cFreshFrame.muCount, true); 
    //printf("Next DMA 0x%08x\r\n", pOwner->mcDMAFrame.mpSamples);
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
    //printf("Sample Rate Hz = %.0f, muClockDivisor=%d\r\n", mfSampleRateHz, muClockDivisor);

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
    //printf("uSamples = %d, uFrameSize = %d\r\n", uSamples, uFrameSize);
    //printf("Frame time = %.0f msec\r\n", ((float)uFrameSize / mfSampleRateHz) * 1000.0f);

    // Setup free list - signal list is empty
    for(uint i=0; i<uFrames; i++) {
        Frame cFrame(mpBuffer + (i*uFrameSize), uFrameSize);
        mlFreeList.push_back(cFrame);
    }
    //printf("Freelist contains %d frames\r\n", mlFreeList.size());

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
    muDMAChannelA = dma_claim_unused_channel(true);

    // So far, so good
    mbIsValid = true;
    //printf("ADC init success\r\n");
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
               
        // Setup initial transfer target
        Frame cTargetFrame;
        {   ScopedLock cLock(&mcStoreLock);
            cTargetFrame = mlFreeList.front();
            cTargetFrame.muSequence = muSequence++;
            mlFreeList.pop_front();
            mcDMAFrame = cTargetFrame;
        }

        // Configure DMA channel
        _configureDMAChannel(muDMAChannelA, cTargetFrame.mpSamples, cTargetFrame.muCount);
    
        // Configure interrupts
        dma_channel_set_irq0_enabled(muDMAChannelA, true);
        irq_set_exclusive_handler(DMA_IRQ_0, _dmaIRQHandler);
        irq_set_enabled(DMA_IRQ_0, true);

        // Start muDMAChannelA
        dma_channel_set_write_addr(muDMAChannelA, cTargetFrame.mpSamples, false);
        dma_channel_set_trans_count(muDMAChannelA, cTargetFrame.muCount, true); 
        //printf("A .mpSamples=0x%08x, .muCount=%d\r\n", cTargetFrame.mpSamples, cTargetFrame.muCount);
 
        // And kick off the adc
        adc_run(true);

        mbIsRunning = true;
        return true;

    } else {
        // Stop everything - switch off and disable interrupts
        adc_run(false);
        dma_channel_abort(muDMAChannelA);
        irq_set_enabled(DMA_IRQ_0, false);
        dma_channel_set_irq0_enabled(muDMAChannelA, false);

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
    //printf("Calling Consumer.process\r\n");
    cConsumer.process(cFrame);

    // Release consumed frame
    {   ScopedLock cLock(&mcStoreLock);
        mlFreeList.push_front(cFrame);
    }

    return true;
}
