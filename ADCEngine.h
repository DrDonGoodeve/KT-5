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
#include <list>


// Types
// ****************************************************************************

// ADCEngine class
/// Captures data under DMA control from the ADC (single channel) and provides
/// a stream of frames for asynchronous processing.
//-----------------------------------------------------------------------------
class ADCEngine {
    public:
        // Simple transparent data class
         class Frame {
            public:
                uint muSequence;    // Out-of-sequence implies discontinuity... 
                uint8_t *mpSamples; // Data
                uint muCount;       // Number of elements in mpSamples
                Frame *mpNext;      // Next frame in this list/stack

                Frame(void) :
                    muSequence(0), mpSamples(nullptr), muCount(0), mpNext(nullptr) {      
                }

                Frame(uint uCount) : 
                    muSequence(0), mpSamples(new uint8_t[uCount]), muCount(uCount), mpNext(nullptr) {
                }

                ~Frame() {
                    delete []mpSamples;
                }
        };       
        
        // A consumer object can be attached to the ADCEngine 
        class Consumer {
            public:
                // Implemented by subclass - process pSamples. The bDiscontinuity flag
                // will be set if there has been a buffer overrun leading to this frame
                // not being contiguous with the last processed. Note that the first
                // frame processed is *always* marked as discontinuous.
                virtual void process(const Frame &cFrame) = 0;
        };

    private:
        bool mbIsValid;             // true iff class initialization has worked out...
        bool mbIsRunning;           // true if DMA/ADC currently running
        float mfSampleRateHz;       // Actual sample rate achieved
        uint muClockDivisor;        // Clock divisor to proce requested sample rate
        uint muDMAChannel;          // DMA channel
        Consumer *mpLastConsumer;   // Last attached consumer object
        uint muSequence;            // Sequence number
        
        // The following are explicitly managed via pointers to avoid causing
        // issues due to heap management routines not being interrupt-safe.
        critical_section_t mcStoreLock;  // IRQ and multicore safe mutex
        Frame *mpSignalListHead;         // Head of signal list (FIFO)
        Frame *mpSignalListTail;         // Tail of signal list
        Frame *mpFreeListHead;           // Head of free list (LIFO)
        Frame *mpDMAFrame;               // Current DMA target frame

        // Support for capture buffer reporting
        uint8_t *mpCaptureBuffer;
        uint32_t muCaptureBufferSize;
        uint32_t muCaptureBufferWriteIndex;
        uint32_t muCaptureBufferReadIndex;

        // Class singleton access
        friend void _dmaIRQHandler(void);  // Has access to this class
        static ADCEngine *mspSelf;         // Singleton pointer

    public:
        /// Construct an ADCEngine object. The object sets up a single ADC Channel
        /// ready for sampling and should be the only ADC sampling mechanism in use.
        /// Specify which channel and the parameters of the sampling to use. Prepares
        /// for use and should be considered 'ready to go' if ::isValid() returns true.
        ADCEngine(
            uint uADCChannel,       // ADC Channel to use (0-3)
            float fSampleRateHz,    // Sample rate to use - will limit to valid range
            float fBufferTimeSec,   // Total buffering time to provide
            uint uFrames            // Number of frames within total buffer to provide
        );

        /// Destructor - switches off PWM and deallocates the GPIO
        ~ADCEngine();

        /// Obtain actual sample rate
        float getSampleRateHz(void) const;

        /// Returns true if set up and good to go. This implements protection
        /// for multiple construction (which is illegal for this object type).
        bool isValid(void) const;

        // Returns true if the ADC engine is currently actively capturing.
        bool isRunning(void) const;

        /// Start/stop capture
        bool setActive(bool bActive=true);

        /// Attach a consumer to process the next frame. If no frame is available,
        /// returns 'false' and cConsumer::process will not be called.
        bool processFrame(Consumer &cConsumer);

        /// Trigger capture of a time sequence
        bool startCapture(uint32_t uTotalSamples);

        /// Report previously captured samples (returns nullptr once exhausted)
        const uint8_t *getNextSamples(uint32_t &uFirstSampleNumber, uint32_t &uRequestedCount);
};

#endif // _ADCENGINE_