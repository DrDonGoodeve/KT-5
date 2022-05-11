/******************************************************************************
 * SpeedMeasurement.cpp
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

// Includes
//*****************************************************************************
#include <stdio.h>
#include <math.h>
#include "SpeedMeasurement.h"
#include "pico/stdlib.h"


// Local functions
//*****************************************************************************
// Called every second - updates time, integrated speed and distance
bool _integrationCallback(repeating_timer_t *pTimer) {
    //printf("_integrationCallback\r\n");
    SpeedMeasurement *pOwner(static_cast<SpeedMeasurement*>(pTimer->user_data));
    if (nullptr == pOwner) {
        return false;    // fail-out - stops timer...
    }

    pOwner->muElapsedTimeSec++;
    pOwner->mfIntegratedSpeedKts += pOwner->mfCurrentSpeedKts;
    pOwner->mfDistanceTravelledNm += (pOwner->mfCurrentSpeedKts * (1/3600.0f));

    return true;
}


// Class methods
//*****************************************************************************
// SpeedMeasurement::
// Implementation of SpeedMeasurement class.
//-----------------------------------------------------------------------------
SpeedMeasurement::SpeedMeasurement(float fSampleRateHz) :
    mfSampleRateHz(fSampleRateHz), muRawMeasurement(0), 
    mfIntegratedSpeedKts(0.0f), mfCurrentSpeedKts(0.0f), 
    mfDistanceTravelledNm(0.0f), muElapsedTimeSec(0) {

    // Start 1 second timer (integration mechanism)
    add_repeating_timer_ms(1000, _integrationCallback, (void*)this, &mcTimer);
}

/// Destructor - clean up all state
SpeedMeasurement::~SpeedMeasurement() {
    // Halt integration timer
    cancel_repeating_timer(&mcTimer);
}

/// ADCEngine::Consumer method
void SpeedMeasurement::process(const ADCEngine::Frame &cFrame) {
    // Simple processing - take peak value and make FSD 8kts
    uint8_t *pData(cFrame.mpSamples);
    uint uCount(cFrame.muCount);
    //printf("SpeedMeasurement::process pData=0x%08x, uCount=%d\r\n", pData, uCount);
    uint8_t uMax(0);
    for(uint i=0; i<uCount; i++) {
        uint8_t uValue(*(pData++));
        uMax = (uValue>uMax)?uValue:uMax;
    }
    muRawMeasurement = uMax;

    //printf("Process frame uMax = %d\r\n", uMax);
    mfCurrentSpeedKts = ((float)uMax / 255.0f) * 8.0f;
}

/// Parameter reporting
uint8_t SpeedMeasurement::getRaw(void) const {
    return muRawMeasurement;
}

float SpeedMeasurement::getSpeedKts(void) const {
    return mfCurrentSpeedKts;
}

float SpeedMeasurement::getAvgSpeedKts(void) const {
    if (muElapsedTimeSec > 0) {
        return (mfIntegratedSpeedKts / (float)muElapsedTimeSec);
    } else {
        return 0.0f;
    }
}

float SpeedMeasurement::getSecondsElapsed(void) const {
    return (float)muElapsedTimeSec;
}

float SpeedMeasurement::getDistanceTravelledNm(void) const {
    return mfDistanceTravelledNm;
}
