#ifndef FEHIO_H
#define FEHIO_H

#include "derivative.h"
#include "MK60DZ10.h"
#include "adc16.h"

/**
 * @brief Objects to be used with the 32 Flex I/O Pins on the FEH Proteus.
 *
 * P3_6 and P3_7 cannot be used for digital encoders. <br/>
 *
 */
class FEHIO
{
public:
    typedef enum
    {
        P0_0 = 0,
        P0_1,
        P0_2,
        P0_3,
        P0_4,
        P0_5,
        P0_6,
        P0_7,
        P1_0,
        P1_1,
        P1_2,
        P1_3,
        P1_4,
        P1_5,
        P1_6,
        P1_7,
        P2_0,
        P2_1,
        P2_2,
        P2_3,
        P2_4,
        P2_5,
        P2_6,
        P2_7,
        P3_0,
        P3_1,
        P3_2,
        P3_3,
        P3_4,
        P3_5,
        P3_6,
        P3_7,
        BATTERY_VOLTAGE
    }FEHIOPin;

    typedef enum
    {
        Bank0 = 0,
        Bank1,
        Bank2,
        Bank3
    }FEHIOPort;

	typedef enum
    {
        RisingEdge = 0x09,
        FallingEdge = 0x0A,
        EitherEdge = 0x0B
    }FEHIOInterruptTrigger;
};

extern void PORTA_IRQHandler();
extern void PORTB_IRQHandler();
extern void PORTC_IRQHandler();
extern void PORTE_IRQHandler();

extern void PIT0_IRQHandler();

/**
 * @brief Configure any of 32 Flex I/O pints as a digital input
 *
 */
// Begin Class Declarations for Pin Types
class DigitalInputPin
{
public:
    DigitalInputPin( FEHIO::FEHIOPin pin );
    /**
     * @brief Returns the value of the DigitalInputPin.
     *  Recall that most of our digital sensors use pull-up resistors, so they return a zero when engaged.
     *
     * @return true the value of the associated DigitalInputPin
     */
    bool Value();

    friend class ButtonBoard;

private:
    FEHIO::FEHIOPin _pin;

    DigitalInputPin();
    void Initialize( FEHIO::FEHIOPin pin );
};

/**
 * @brief Monitor a digital encoder using a digital input pin on the Proteus
 * P3_6 and P3_7 cannot be used for digital encoders as they are hard-wired to other Proteus functions.
 */
class DigitalEncoder
{
public:
    DigitalEncoder( FEHIO::FEHIOPin pin, FEHIO::FEHIOInterruptTrigger trigger );
    DigitalEncoder( FEHIO::FEHIOPin pin);
    /**
     * @brief Return the current counts for DigitalEncoder object
     *
     * @return the current counts for the associated DigitalEncoder object
     */
    int Counts();

    /**
     * @brief Reset counts for DigitalEncoder object to 0
     *
     */
    void ResetCounts();

private:
    FEHIO::FEHIOPin _pin;

    DigitalEncoder();
    void Initialize( FEHIO::FEHIOPin pin, FEHIO::FEHIOInterruptTrigger trigger );
};

/**
 * @brief Configure any of 32 Flex I/O pints as an analog input
 *
 */
class AnalogInputPin
{
protected:
    FEHIO::FEHIOPin pin;
    static tADC_Config Master_Adc_Config;
    static tADC_Config Encoder_Adc_Config;

public:
    AnalogInputPin( FEHIO::FEHIOPin );
    /**
     * @brief Returns the value of the AnalogInputPin
     *
     * @return the value (0 to 3.3V) of the associated analog input pin
     */
    float Value();


    static void InitADCs();
};

/**
 * @brief Configure any of 32 Flex I/O pints as a digital output, such as LEDs
 *
 */
class DigitalOutputPin
{
private:
    FEHIO::FEHIOPin pin;
public:
    DigitalOutputPin( FEHIO::FEHIOPin );

    /**
     *
     * @brief Set the state of the DigitalOutputPin, such as turning an LED on or off
     * @param bool the value to write to the associated DigitalOutputPin
     */
    void Write( bool );

    /**
     * @brief Returns the state of the DigitalOutputPin
     *
     * @return a boolean corresponding to the current value being written to the associated DigitalOutputPin
     */
    bool Status();

    /**
     * @brief Switches the output state of the DigitalOutputPin to the opposite truth value
     *
     */
    void Toggle();
};

/**
 * @brief Set of functions that return whether each of the three buttons (left, middle, and right) of the Proteus ButtonBoard are Pressed or Released
 *
 */
class ButtonBoard
{
public:
    ButtonBoard( FEHIO::FEHIOPort bank );
    /**
     * @brief returns whether the left button of the Proteus ButtonBoard is pressed.
     *
     * @return true if the left button is pressed, false otherwise.
     */
    bool LeftPressed();

    /**
     * @brief returns whether the left button of the Proteus ButtonBoard is released.
     *
     * @return true if the left button is released, false otherwise.
     */
    bool LeftReleased();

    /**
     * @brief returns whether the middle button of the Proteus ButtonBoard is pressed.
     *
     * @return true if the middle button is pressed, false otherwise.
     */
    bool MiddlePressed();

    /**
     * @brief returns whether the middle button of the Proteus ButtonBoard is released.
     *
     * @return true if the middle button is released, false otherwise.
     */
    bool MiddleReleased();

    /**
     * @brief returns whether the right button of the Proteus ButtonBoard is pressed.
     *
     * @return true if the right button is pressed, false otherwise.
     */
    bool RightPressed();

    /**
     * @brief returns whether the right button of the Proteus ButtonBoard is released.
     *
     * @return true if the right button is released, false otherwise.
     */
    bool RightReleased();

private:
    DigitalInputPin _left;
    DigitalInputPin _middle;
    DigitalInputPin _right;
};

/**
 * @brief Monitor an analog encoder using a analog input pin on the Proteus
 *
 */
class AnalogEncoder : public AnalogInputPin
{
private:
    typedef enum
    {
        LOW_STATE,
        HIGH_STATE
    } EncoderState;

    static void SetCounterInit(unsigned int);

    int counts;
    int lowThreshold;
    int highThreshold;

    // These hold the next and previous encoders tied to the same pin
    AnalogEncoder * pNext;
    AnalogEncoder * pPrev;

    bool ProcessIntSelf();
    int EncoderValue();

    struct PinInfo;
    typedef struct PinInfo PinInfo;

    struct PinInfo
    {
        FEHIO::FEHIOPin pin;
        EncoderState state;
        int numEncoders;
        AnalogEncoder * encoderList;
        PinInfo * pNext;
        PinInfo * pPrev;
    };


    static PinInfo * pinList;
    PinInfo * pPinInfo;

public:
    static void Init();
    static void ProcessInt();
    static void SetRate(unsigned int rateHz);
    AnalogEncoder(FEHIO::FEHIOPin);
    ~AnalogEncoder();

    /**
     * @brief Return the current counts for AnalogEncoder object
     *
     * @return the current counts for the associated AnalogEncoder object
     */
    int Counts();

    /**
     * @brief  Reset counts for AnalogEncoder object to 0
     *
     */
    void ResetCounts();

    /**
     * @brief Define the high and low thresholds for an AnalogEncoder object
     *
     * Make sure to provide tolerance between the actual low and high values and their respective low and high thresholds
     *
     * @param low the low threshold value
     * @param high the high threshold value
     */
    void SetThresholds(float low, float high);
};

#endif // FEHIO_H
