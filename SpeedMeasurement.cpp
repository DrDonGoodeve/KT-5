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

// Macros
//*****************************************************************************
#define TC(time,srate)    (powf(10.0f, (log10f(0.632f)/(srate*time))))


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
    mfSampleRateHz(fSampleRateHz), mbResetAll(true),
    mfIntegratedSpeedKts(0.0f), mfCurrentSpeedKts(0.0f), 
    mfDistanceTravelledNm(0.0f), muElapsedTimeSec(0) {

    // Initialize everything to defaults
    setConstants();

    // Start 1 second timer (integration mechanism)
    add_repeating_timer_ms(1000, _integrationCallback, (void*)this, &mcTimer);
}

/// Destructor - clean up all state
SpeedMeasurement::~SpeedMeasurement() {
    // Halt integration timer
    cancel_repeating_timer(&mcTimer);
}

void SpeedMeasurement::setConstants(
    float fPPSToKts, float fPulseMagnitudeToKts,
    float fPeakDecayTC, float fAvgFilterTC,
    float fEdgeThreshold, float fEdgeHysteresis,
    uint8_t uMinimumMagnitude, float fPPSAvgConstant,
    EMethod eMethod) {

    mfPPSToKts = fPPSToKts;
    mfPulseMagnitudeToKts = fPulseMagnitudeToKts;
    mfPeakDecayTC = fPeakDecayTC;
    mfAvgFilterTC = fAvgFilterTC;
    mfEdgeThresholdProportion = fEdgeThreshold;
    mfEdgeHysteresis = fEdgeHysteresis;
    muMinimumPulseMagnitude = uMinimumMagnitude;
    mfPPSAvgConst = fPPSAvgConstant;
    meMethod = eMethod;
}

void SpeedMeasurement::setPPSToKts(float fPPSToKts) {
    mfPPSToKts = fPPSToKts;
}

void SpeedMeasurement::setPulseMagnitudeToKts(float fPulseMagnitudeToKts) {
    mfPulseMagnitudeToKts = fPulseMagnitudeToKts;
}

void SpeedMeasurement::setPeakDecayTC(float fPeakDecayTC) {
    mfPeakDecayTC = fPeakDecayTC;
}

void SpeedMeasurement::setAvgFilterTC(float fAvgFilterTC) {
    mfAvgFilterTC = fAvgFilterTC;
}

void SpeedMeasurement::setEdgeThreshold(float fEdgeThreshold) {
    mfEdgeThresholdProportion = fEdgeThreshold;
}

void SpeedMeasurement::setEdgeHysteresis(float fEdgeHysteresis) {
    mfEdgeHysteresis = fEdgeHysteresis;
}

void SpeedMeasurement::setPPSAvgConst(float fPPSAvgConstant) {
    mfPPSAvgConst = fPPSAvgConstant;
}

void SpeedMeasurement::setMinimumPulseMagnitude(uint8_t uMinimumMagnitude) {
    muMinimumPulseMagnitude = uMinimumMagnitude;
}

void SpeedMeasurement::setMethod(SpeedMeasurement::EMethod eMethod) {
    meMethod = eMethod;
}

/// ADCEngine::Consumer method
void SpeedMeasurement::process(const ADCEngine::Frame &cFrame) {
    uint8_t *pData(cFrame.mpSamples);
    uint uCount(cFrame.muCount);

    if (true == mbResetAll) {
        mbResetAll = false;
        mfMax = mfMin = mfAvg = (float)(*pData);
        mfAvgPulsesPerSecond = 0.0f;
        muSampleCount = 0;
        muLastRisingTrigger = muLastFallingTrigger = 0;
    }

    // Determine and apply peak detector measurement decay constants (per frame)
    float fFrameTime((float)uCount / mfSampleRateHz);
    float fSampleTime(1.0f / mfSampleRateHz);
    float fPeakDecay(TC(mfPeakDecayTC, fFrameTime));
    float fAvgWeight(TC(mfAvgFilterTC, fSampleTime));
    mfMax = mfAvg + ((mfMax-mfAvg)*fPeakDecay);
    mfMin = mfAvg - ((mfAvg-mfMax)*fPeakDecay);

    // Determine thresholds
    float fMaxAboveAvg(mfMax - mfAvg), fMinBelowAvg(mfAvg - mfMin);
    float fHighThreshold((fMaxAboveAvg * mfEdgeThresholdProportion) + mfAvg);
    float fHighReleaseThreshold((fMaxAboveAvg * (mfEdgeThresholdProportion * (1.0f - mfEdgeHysteresis))) + mfAvg);
    float fLowThreshold(mfAvg - (fMinBelowAvg * mfEdgeThresholdProportion));
    float fLowReleaseThreshold(mfAvg - (fMinBelowAvg * (mfEdgeThresholdProportion * (1.0f - mfEdgeHysteresis))));

    // Loop over measurement state machine
    for(uint i=0; i<uCount; i++) {
        // Abbreviations
        uint8_t &uSample(*(pData++));
        float fSample((float)uSample);
        uint uSampleNumber(muSampleCount+i);

        // Per-sample calculations
        mfMax = (fSample > mfMax)?fSample:mfMax;
        mfMin = (fSample < mfMin)?fSample:mfMin;
        mfAvg = (fSample * fAvgWeight) + (mfAvg * (1.0f - fAvgWeight));

        // Edge-detector logic (state machine)
        bool bPulsesValid((mfMax-mfMin) >= (float)muMinimumPulseMagnitude);
        switch(meSignalState) {
            case kUndefinedSignal:
                if (false == bPulsesValid) {
                    break;
                }
                if (fSample >= fHighThreshold) {
                    meSignalState = kInPositivePulse;
                    if (muLastRisingTrigger != 0) {
                        uint uBetweenRisingEdges(uSampleNumber - muLastRisingTrigger);
                        float fRisingPPS(mfSampleRateHz / (float)uBetweenRisingEdges);
                        if (0.0f == mfAvgPulsesPerSecond) {
                            mfAvgPulsesPerSecond = fRisingPPS;
                        } else {
                            mfAvgPulsesPerSecond = ((fRisingPPS * mfPPSAvgConst) + ((1.0f - mfPPSAvgConst) * mfAvgPulsesPerSecond));
                        }
                    }
                    muLastRisingTrigger = uSampleNumber;

                } else if (fSample <= fLowThreshold) {
                    meSignalState = kInNegativePulse;
                    if (muLastRisingTrigger != 0) {
                        uint uBetweenFallingEdges(uSampleNumber - muLastFallingTrigger);
                        float fFallingPPS(mfSampleRateHz / (float)uBetweenFallingEdges);
                        if (0.0f == mfAvgPulsesPerSecond) {
                            mfAvgPulsesPerSecond = fFallingPPS;
                        } else {
                            mfAvgPulsesPerSecond = ((fFallingPPS * mfPPSAvgConst) + ((1.0f - mfPPSAvgConst) * mfAvgPulsesPerSecond));
                        }
                    }
                    muLastFallingTrigger = uSampleNumber;
                }
                break;

            case kInPositivePulse:
                if (fSample <= fHighReleaseThreshold) {
                    meSignalState = kUndefinedSignal;
                }
                break;
            case kInNegativePulse:
                if (fSample >= fLowReleaseThreshold) {
                    meSignalState = kUndefinedSignal;
                }
                break;
        }
        pData++;
    }
    muSampleCount += uCount;

    // End of frame - update all measurements
    // I now have updated/filtered values of mfMax, mfMin, mfAvg, mfAvgPulsesPerSecond
    switch(meMethod) {
        case kPulseMethod:
            mfCurrentSpeedKts = mfPPSToKts * mfAvgPulsesPerSecond;
            break;
        case kRangeMethod:
            mfCurrentSpeedKts = mfPulseMagnitudeToKts * (mfMax-mfMin);
            break;
        case kHybridMethod: default:
            mfCurrentSpeedKts = (0.5f * (mfPPSToKts * mfAvgPulsesPerSecond)) + (0.5f * (mfPulseMagnitudeToKts * (mfMax-mfMin)));
            break;
    }
}

/// Parameter reporting
static const char *_methodToString(const SpeedMeasurement::EMethod &eMethod) {
    switch(eMethod) {
        case SpeedMeasurement::kPulseMethod: return "pulse";
        case SpeedMeasurement::kRangeMethod: return "range";
        case SpeedMeasurement::kHybridMethod: return "hybrid";
        default: return "undefined";
    }
}

void SpeedMeasurement::reportParameters(void) const {
    printf("\tPPS to Kts = %.2f\r\n", mfPPSToKts);
    printf("\tPulse Magnitude to Kts = %.2f\r\n", mfPulseMagnitudeToKts);
    printf("\tPeak decay time constant = %.2f\r\n", mfPeakDecayTC);
    printf("\tAverage filter time constant = %.2f\r\n", mfAvgFilterTC);
    printf("\tEdge threshold (of peak) = %.3f, hysteresis (of threshold) = %.3f\r\n", mfEdgeThresholdProportion, mfEdgeHysteresis);
    printf("\tMinimum pulse magnitude (counts) = %d\r\n", muMinimumPulseMagnitude);
    printf("\tPPS averaging constant = %.2f\r\n", mfPPSAvgConst);
    printf("\tmeasurement method: %s", _methodToString(meMethod));
}

void SpeedMeasurement::reportDynamicState(void) const {
    printf("\tSignal (min:%.2f, avg:%.2f, max:.2f)\r\n", mfMin, mfAvg, mfMax);
    printf("\tAvg PPS: %.2f\r\n", mfAvgPulsesPerSecond);
    printf("\tCurrent speed kts: %.3f\r\n", mfCurrentSpeedKts);
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
