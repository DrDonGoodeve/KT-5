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
#include "ADCEngine.h"
#include "pico/stdlib.h"


// SpeedMeasurement class
/// Primary function is to estimate speed from the transducer signal. Creates
/// derived measurements.
//-----------------------------------------------------------------------------
class SpeedMeasurement : public ADCEngine::Consumer {
    private:
        friend bool _integrationCallback(repeating_timer_t *);

        float mfSampleRateHz;       // Sample rate of frames passed to ::process method

        repeating_timer_t mcTimer;  // Required for integration mechanism

        uint8_t muRawMeasurement;   // Current raw measurement
        float mfCurrentSpeedKts;    // Computed current speed
        uint muElapsedTimeSec;      // Time since start
        float mfDistanceTravelledNm;

        float mfIntegratedSpeedKts;

    public:
        /// Initialize the class and set accumulated time to zero.
        SpeedMeasurement(float fSampleRateHz);

        /// Destructor - switches off PWM and deallocates the GPIO
        ~SpeedMeasurement();

        /// ADCEngine::Consumer method
        virtual void process(const ADCEngine::Frame &cFrame);

        /// Parameter reporting
        uint8_t getRaw(void) const;
        float getSpeedKts(void) const;
        float getAvgSpeedKts(void) const;
        float getSecondsElapsed(void) const;
        float getDistanceTravelledNm(void) const;
};

#endif // _SPEEDMEASUREMENT_