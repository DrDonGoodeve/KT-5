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

// Macros
//*****************************************************************************
#define TC(time,oprate)    (powf(10.0f, (log10f(0.632f)/(time * oprate))))


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
    float fPeakDecayTC, float fAvgFilterTC, float fPPSFilterTC,
    uint8_t uNoiseThreshold,
    float fEdgeThreshold, float fEdgeHysteresis,
    uint8_t uMinimumMagnitude, float fPPSAvgConstant, float fMaxAcceptablePPS,
    EMethod eMethod) {

    mfPPSToKts = fPPSToKts;
    mfPulseMagnitudeToKts = fPulseMagnitudeToKts;
    mfPeakDecayTC = fPeakDecayTC;
    mfAvgFilterTC = fAvgFilterTC;
	mfPPSFilterTC = fPPSFilterTC;
    muNoiseThreshold = uNoiseThreshold;
    mfEdgeThresholdProportion = fEdgeThreshold;
    mfEdgeHysteresis = fEdgeHysteresis;
    muMinimumPulseMagnitude = uMinimumMagnitude;
    mfPPSAvgConst = fPPSAvgConstant;
    mfMaxAcceptablePPS = fMaxAcceptablePPS;
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

void SpeedMeasurement::setPPSFilterTC(float fPPSFilterTC) {
    mfPPSFilterTC = fPPSFilterTC;
}

void SpeedMeasurement::setNoiseThreshold(uint8_t uNoiseThreshold) {
    muNoiseThreshold = uNoiseThreshold;
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

const char *SpeedMeasurement::getMeasurementMethod(void) const {
    switch(meMethod) {
        case kPulseMethod: return "pulse";
        case kRangeMethod: return "range";
        case kHybridMethod: default: return "hybrid";
    }
}

void SpeedMeasurement::setMaxAcceptablePPS(float fMaxAcceptablePPS) {
    mfMaxAcceptablePPS = _max(kMinPPSLimit, fMaxAcceptablePPS);
}

/// ADCEngine::Consumer method
void SpeedMeasurement::process(const ADCEngine::Frame &cFrame) {
    uint8_t *pData(cFrame.mpSamples);
    uint32_t uCount(cFrame.muCount);

    if (true == mbResetAll) {
        mbResetAll = false;
		meSignalState = kUndefinedSignal;
        mfMax = mfMin = mfAvg = (float)(*pData);
        mfAvgPulsesPerSecond = 0.0f;
        muSampleCount = 0;
		muPulseCount = 0;
        muLastRisingTrigger = muLastFallingTrigger = 0;
    }

    // Determine and apply peak detector measurement decay constants (per frame)
	float fFrameRate(mfSampleRateHz / (float)uCount);
    float fPeakDecay(TC(mfPeakDecayTC, fFrameRate));
    float fAvgFilter(TC(mfAvgFilterTC, fFrameRate));
    float fPPSFilter(TC(mfPPSFilterTC, fFrameRate));
    mfMax = mfAvg + ((mfMax-mfAvg)*fPeakDecay);
    mfMin = mfAvg - ((mfAvg-mfMin)*fPeakDecay);

    // Determine thresholds
    float fMaxAboveAvg(mfMax - mfAvg), fMinBelowAvg(mfAvg - mfMin);
    float fHighThreshold((fMaxAboveAvg * mfEdgeThresholdProportion) + mfAvg);
    float fHighReleaseThreshold((fMaxAboveAvg * (mfEdgeThresholdProportion * (1.0f - mfEdgeHysteresis))) + mfAvg);
    float fLowThreshold(mfAvg - (fMinBelowAvg * mfEdgeThresholdProportion));
    float fLowReleaseThreshold(mfAvg - (fMinBelowAvg * (mfEdgeThresholdProportion * (1.0f - mfEdgeHysteresis))));
    bool bPulsesValid((mfMax-mfMin) >= (float)muMinimumPulseMagnitude);

    // Loop over measurement state machine
	float fSum(0.0f);
    for(uint32_t i=0; i<uCount; i++) {
        // Abbreviations
        uint8_t &uSample(*(pData++));
        float fSample((float)uSample);
		fSum += fSample;
        uint32_t uSampleNumber(muSampleCount+i);

        // Per-sample calculations
        mfMax = (fSample > mfMax)?fSample:mfMax;
        mfMin = (fSample < mfMin)?fSample:mfMin;

        // Edge-detector logic (state machine)
        float fMinInterPulseTime(1.0f / mfMaxAcceptablePPS);
        uint32_t uMinInterPulseSamples((uint32_t)roundf(fMinInterPulseTime * mfSampleRateHz));
        switch(meSignalState) {
            case kUndefinedSignal:
                if (false == bPulsesValid) {
                    break;
                }
                if (fSample > fHighThreshold) {
                    if (muLastRisingTrigger != 0) {
                        uint32_t uBetweenRisingEdges(uSampleNumber - muLastRisingTrigger);
                        if (uBetweenRisingEdges < uMinInterPulseSamples) {
                            break;  // Reject edges that come too fast - noise pulses
                        }
                        float fRisingPPS(mfSampleRateHz / (float)uBetweenRisingEdges);
                        if (0.0f == mfAvgPulsesPerSecond) {
                            mfAvgPulsesPerSecond = fRisingPPS;
                        } else {
                            mfAvgPulsesPerSecond = ((fRisingPPS * mfPPSAvgConst) + ((1.0f - mfPPSAvgConst) * mfAvgPulsesPerSecond));
                        }
                    }
                    meSignalState = kInPositivePulse;
                    muLastRisingTrigger = uSampleNumber;

                } else if (fSample < fLowThreshold) {
                    if (muLastFallingTrigger != 0) {
                        uint32_t uBetweenFallingEdges(uSampleNumber - muLastFallingTrigger);
                         if (uBetweenFallingEdges < uMinInterPulseSamples) {
                            break;  // Reject edges that come too fast - noise pulses
                        }
                        float fFallingPPS(mfSampleRateHz / (float)uBetweenFallingEdges);
                        if (0.0f == mfAvgPulsesPerSecond) {
                            mfAvgPulsesPerSecond = fFallingPPS;
                        } else {
                            mfAvgPulsesPerSecond = ((fFallingPPS * mfPPSAvgConst) + ((1.0f - mfPPSAvgConst) * mfAvgPulsesPerSecond));
                        }
                    }
                    meSignalState = kInNegativePulse;
                    muLastFallingTrigger = uSampleNumber;
                }
                break;

            case kInPositivePulse:
                if (fSample < fHighReleaseThreshold) {
                    meSignalState = kUndefinedSignal;
					muPulseCount++;
                }
                break;

            case kInNegativePulse:
                if (fSample > fLowReleaseThreshold) {
                    meSignalState = kUndefinedSignal;
					muPulseCount++;
                }
                break;
        }
    }
    muSampleCount += uCount;
	float fFrameAverage(fSum / (float)uCount);
	mfAvg = ((1.0f - fAvgFilter) * fFrameAverage) + (fAvgFilter * mfAvg);
    mfAvgPulsesPerSecond *= fPPSFilter;

    // End of frame - update all measurements
    // I now have updated/filtered values of mfMax, mfMin, mfAvg, mfAvgPulsesPerSecond
    float fRange(mfMax - mfMin), fNoiseThreshold((float)muNoiseThreshold);
    float fPeakCounts(((fRange > fNoiseThreshold) ? fRange : 0.0f));
    float fSignalVolts(fPeakCounts * kADCResolutionVolts);
    float fMagnitudeBasedSpeed(mfPulseMagnitudeToKts * fSignalVolts);
    float fPPSBasedSpeed(mfPPSToKts * mfAvgPulsesPerSecond);
	switch (meMethod) {
		case kPulseMethod:
			mfCurrentSpeedKts = fPPSBasedSpeed;
			break;
		case kRangeMethod:
			mfCurrentSpeedKts = fMagnitudeBasedSpeed;
			break;
		case kHybridMethod: default:
			if (fSignalVolts > (0.9f * kADCRange)) {
				mfCurrentSpeedKts = fPPSBasedSpeed;
			}
			else {
				mfCurrentSpeedKts = (0.5f * fPPSBasedSpeed) + (0.5f * fMagnitudeBasedSpeed);
			}
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
    printf("\tPPS to Kts (PPS/kt) = %.2f\r\n", mfPPSToKts);
    printf("\tPulse Magnitude to Kts (V/kt) = %.2f\r\n", mfPulseMagnitudeToKts);
    printf("\tPeak decay time constant = %.2f\r\n", mfPeakDecayTC);
    printf("\tAverage filter time constant = %.2f\r\n", mfAvgFilterTC);
    printf("\tPPS filter time constant = %.2f\r\n", mfPPSFilterTC);
    printf("\tEdge threshold (of peak) = %.3f\r\n\tHysteresis (of threshold) = %.3f\r\n", mfEdgeThresholdProportion, mfEdgeHysteresis);
    printf("\tNoise threshold (counts) = %d\r\n", muNoiseThreshold);
    printf("\tMinimum pulse magnitude (counts) = %d\r\n", muMinimumPulseMagnitude);
    printf("\tPPS averaging constant = %.2f\r\n", mfPPSAvgConst);
    printf("\tMax acceptable PPS = %.1f\r\n", mfMaxAcceptablePPS);
    printf("\tmeasurement method: %s\r\n", _methodToString(meMethod));
}

void SpeedMeasurement::reportDynamicState(void) const {
    printf("\tSignal (min:%.2f, avg:%.2f, max:%.2f, pk/pk:%.2f)\r\n", mfMin, mfAvg, mfMax, (mfMax-mfMin));
	printf("\tSamples:%d, Pulses:%d\r\n", muSampleCount, muPulseCount);
	printf("\tAvg PPS: %.2f\r\n", mfAvgPulsesPerSecond);
    printf("\tMeasurement method: %s\r\n", getMeasurementMethod());
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
