/******************************************************************************
 * ADCEngine.h
 * 
 * Singleton class that manages capturing continuous stream data from
 * the ADC under DMA control. The result is a continually refreshing
 * contiguous data stream composed of frames. A consumer can attach
 * to the stream a frame at a time to process it, after which the
 * frame is released back into the buffer pool. The size of the buffer
 * specifies a limit on the total buffering, not the latency of processing.
 * To simplify synchronize data is processed inside the ::process method of
 * an ADCCaptureEngine::Consumer implementation (pure virtual base class).
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 31Mar2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _ADCENGINE_
#define _ADCENGINE_

// Includes
//*****************************************************************************
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include <list>


// Types
// ****************************************************************************

// ADCEngine class
/// Captures data under DMA control from the ADC (single channel) and provides
/// a stream of frames for asynchronous processing.
//-----------------------------------------------------------------------------
class ADCEngine {
    public:
         class Frame {
            public:
                uint uSequence;     // Out-of-sequence implies discontinuity... 
                uint8_t *mpSamples; // Data
                uint muCount;       // Number of samples in data
                Frame(uint8_t pSamples, uint uCount) : 
                    muSequence(0), mpSamples(pSamples), muCount(uCount) {
                }
        };       
        
        // A consumer object can be attached to the ADCEngine 
        class Consumer {
            public:
                // Implemented by subclass - process pSamples. The bDiscontinuity flag
                // will be set if there has been a buffer overrun leading to this frame
                // not being contiguous with the last processed. Note that the first
                // frame processed is *always* marked as discontinuous.
                virtual void process(const Frame &cFrame, bool bDiscontinuity) = 0;
        };

    private:
        bool mbIsValid;             // true iff class initialization has worked out...
        bool mbIsRunning;           // true if DMA/ADC currently running
        float mfSampleRateHz;       // Actual sample rate achieved
        uint muClockDivisor;        // Clock divisor to proce requested sample rate
        uint mpDMAChannel[2];       // Uses two DMA channels (chained)
        Consumer *mpLastConsumer;   // Last attached consumer object
        bool mbDiscontinuity;       // Discontinuity flag

        uint8_t *mpBuffer;          // Buffer from which mlSignalList, mlFreeList frames come...
        
        critical_section_t mcStoreLock;     // IRQ and multicore safe mutex
        std::list<Frame*> mlSignalBuffer;   // Allocated data in signal sequence order
        std::list<Frame*> mlFreeList;       // Available blocks in arbitrary order

        friend void dma_irq_handler(void);  // Has access to this class
        static ADCEngine *mspSelf;          // Singleton pointer

    public:
        /// Construct a Servo object managing the specified pin. User can
        /// specify polarity and timing parameters for PWM signal. The
        /// PWM signal operates at a 'standard' 20msec period (50Hz) and
        /// is active high.
        ADCEngine(uint uADCChannel, float fSampleRateHz,
                  float fBufferTimeSec, float fFrameTimeSec);

        /// Destructor - switches off PWM and deallocates the GPIO
        ~ADCEngine();

        /// Returns true if set up and good to go. This implements protection
        /// for multiple construction (which is illegal for this object type).
        bool isValid(void) const;

        /// Start/stop capture
        bool setActive(bool bActive=true);

        /// Attach a consumer to process the next frame. If no frame is available,
        /// returns 'false' and cConsumer::process will not be called.
        bool processFrame(Consumer &cConsumer);
};

#endif // _ADCENGINE_