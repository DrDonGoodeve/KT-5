/******************************************************************************
 * SpeedMeasurement.h
 * 
 * SpeedMeasurement class. Subclass of ADCEngine::Consumer which processes
 * signal information from the transducer (via basic signal conditioning)
 * to produce running estimates of time since start (in seconds), speed
 * (in knots), distance travelled (in nautical miles) and average speed
 * (in knots).
 * 
 * Don Goodeve  don@radiocode.ca  ground broken: 1Apr2022
 ******************************************************************************
 * See LICENSE for details of how this code can be used.
 *****************************************************************************/

#ifndef _SPEEDMEASUREMENT_
#define _SPEEDMEASUREMENT_


// Includes
//-----------------------------------------------------------------------------
#ifndef _sandbox
#include "ADCEngine.h"
#include "pico/stdlib.h"
#endif

// Defines
//-----------------------------------------------------------------------------
#define kDefaultPPSToKts        (0.2f)
#define kDefaultPulseMagToKts   ((2.0f * 0.97f) / 5.0f)
#define kDefaultPeakDecayTC     (5.0f)
#define kDefaultAvgFilterTC     (5.0f)
#define kDefaultEdgeThreshold   (0.67f)
#define kDefaultEdgeHysteresis  (0.5f)
#define kMinimumPulseMagnitude  (10)
#define kDefaultPPSAvgConstant  (0.05f)


// SpeedMeasurement class
/// Primary function is to estimate speed from the transducer signal. Creates
/// derived measurements.
//-----------------------------------------------------------------------------
class SpeedMeasurement : public ADCEngine::Consumer {
    public:
            // Measurement method to use to obtain current speed and all derived measurements
        typedef enum {
            kPulseMethod = 0,
            kRangeMethod = 1,
            kHybridMethod = 2
        }EMethod;

    private:
        friend bool _integrationCallback(repeating_timer_t *);

        float mfSampleRateHz;       // Sample rate of frames passed to ::process method

        repeating_timer_t mcTimer;  // Required for integration mechanism

        // Edge detector mechanism
        bool mbResetAll;    // Reset everything on first sample
        float mfMax;        // Max signal decayed at mfPeakDecayRate per sample to mfAvg
        float mfMin;        // Min signal decayed at mfPeakDecayRate per sample to mfAvg
        float mfPeakDecayTC;    // Time constant for mfMax and mfMin to decay to mfAvg

        float mfAvg;            // Average signal filtered at mfAvgFilterRate
        float mfAvgFilterTC;    // Time constant for filtering average signal

        typedef enum {
            kUndefinedSignal,
            kInPositivePulse,
            kInNegativePulse
        }ESignalState;

        ESignalState meSignalState;     // Current signal state (machine)
        uint32_t muLastRisingTrigger;   // Sample at last rising trigger
        uint32_t muLastFallingTrigger;  // Sample at last falling trigger 
        uint32_t muSampleCount;         // Tracking current sample number (start of latest frame)

        float mfEdgeThresholdProportion;  // Proportion of peak for threshold
        float mfEdgeHysteresis;           // Proportion of peak for edge hysteresis
        uint8_t muMinimumPulseMagnitude;  // Less than this min/max is considered noise and ignored
		uint32_t muPulseCount;			  // Number of pulses detected since run start

        // Derived measurements
        float mfPPSAvgConst;            // Contribution to running average of a new PPS measurement
        float mfAvgPulsesPerSecond;     // Pulses per second - measured/filtered
        float mfMeasurementDecayTC;     // Per-sample decay time constant for pulse measurements

        // Conversion constants
        float mfPPSToKts;
        float mfPulseMagnitudeToKts;

        // Measurement method
        EMethod meMethod;

        // Derived measurements
        float mfCurrentSpeedKts;        // Computed current speed
        uint32_t muElapsedTimeSec;      // Time since start
        float mfDistanceTravelledNm;    // Integrated distance travelled
        float mfIntegratedSpeedKts;     // Integrated speed - divide by time for avg speed

    public:
        /// Initialize the class and set accumulated time to zero.
        SpeedMeasurement(float fSampleRateHz);

        /// Destructor - switches off PWM and deallocates the GPIO
        ~SpeedMeasurement();

        // Set all control constants
        void setConstants(
            float fPPSToKts = kDefaultPPSToKts,
            float fPulseMagnitudeToKts = kDefaultPulseMagToKts,
            float fPeakDecayTC=kDefaultPeakDecayTC,
            float fAvgFilterTC=kDefaultAvgFilterTC,
            float fEdgeThreshold=kDefaultEdgeThreshold,
            float fEdgeHysteresis=kDefaultEdgeHysteresis,
            uint8_t uMinimumMagnitude=kMinimumPulseMagnitude,
            float fPPSAvgConst=kDefaultPPSAvgConstant,
            EMethod eMethod=kPulseMethod
        );

        void setPPSToKts(float fPPSToKts);
        void setPulseMagnitudeToKts(float fPulseMagnitudeToKts);
        void setPeakDecayTC(float fPeakDecayTC);
        void setAvgFilterTC(float fAvgFilterTC);
        void setEdgeThreshold(float fEdgeThreshold);
        void setEdgeHysteresis(float fEdgeHysteresis);
        void setMinimumPulseMagnitude(uint8_t uMinimumMagnitude);
        void setPPSAvgConst(float fPPSAvgConstant);
        void setMethod(SpeedMeasurement::EMethod eMethod);

        /// ADCEngine::Consumer method
        virtual void process(const ADCEngine::Frame &cFrame);

        /// Parameter reporting
        void reportParameters(void) const;          // Report all programmed parameters
        void reportDynamicState(void) const;        // Report all current state to stdout

        // Derived measurement reporting
        float getSpeedKts(void) const;
        float getAvgSpeedKts(void) const;
        float getSecondsElapsed(void) const;
        float getDistanceTravelledNm(void) const;
};

#endif // _SPEEDMEASUREMENT_