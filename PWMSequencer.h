/******************************************************************************
* @file PWMSequencer.h
*
* @brief Class and associated definitions for controlling the PWM engine
* and running autonomous programming.
*******************************************************************************
* Language: C++11
* File Version: 1.0
* @date 11Oct22
* @author: Don Goodeve (don@radiocode.ca)
*******************************************************************************
* Code Copyright (C) 2022 Don Goodeve Coaching and Consulting. All Rights Reserved.
******************************************************************************/

// includes
//*****************************************************************************

// macros
//*****************************************************************************

// types
//*****************************************************************************
// PWM
// Specifies all that is necessary about a single PWM control.
//-----------------------------------------------------------------------------
typedef struct {
    const char *mpName;     // Textual name
    uint8 muPin;            // Assigned pin
    bool bActiveHighNotLow; // Active state
}PWM;

// PWMTransition
// Specifies all that is necessary about a single PWM control.
//-----------------------------------------------------------------------------
typedef struct {
    const char *mpName;     // Textual name
    float mfInitial;        // Initial duty cycle (0.0-1.0)
    float mfFinal;          // Final duty cycle (0.0-1.0)
}PWMTransition;

// PWMProgramStep
// Contains all information about a step in the intended output sequence. A
// step specifies initial and final values for all PWM signals and a time
// over which this change is to occur. All changes within a step are
// monotonic.
//-----------------------------------------------------------------------------
typedef struct {
    PWMTransition *mpTransitions;
    PWMProgramStep *mpNextStep;
}PWMProgramStep;


// classes
//*****************************************************************************
// PWMSequencer
// Sequences the PWM peripheral and associated pins.
//-----------------------------------------------------------------------------
class PWMSequencer {
    private:
    public:
        // Constructor. Takes a nullptr-terminated list of PWMs and a frequency.
        // Sets all corresponding pins to output and to their inactive state and
        // starts the PWM peripheral ready for operation. Note that fOperationFrequency
        // sets the granularity of all sequence step timing programming.
        PWMSequencer(const PWM *pPWMs, float fOperationFrequency);

        // Destructor
        ~PWMSequencer();

        // Set the next program step to follow. If there is a program step currently
        // running and bForce is false - this step will be enacted after the current
        // step completes. If bForce is true then the new program step will be
        // enacted at the next opportunity.
        void setProgram(PWMProgramStep *pStep, bool bForce = false);
};