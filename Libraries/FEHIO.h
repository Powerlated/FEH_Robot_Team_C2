#ifndef FEHIO_H
#define FEHIO_H

#include "derivative.h"
#include "MK60DZ10.h"
#include "adc16.h"

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
        P3_7
    }FEHIOPin;

    typedef enum
    {
        Bank0 = 0,
        Bank1,
        Bank2,
        Bank3
    }FEHIOPort;
};

// Begin Class Declarations for Pin Types
class DigitalInputPin
{
public:
    DigitalInputPin( FEHIO::FEHIOPin pin );
    int Value();

private:
    FEHIO::FEHIOPin _pin;
};

//class AnalogInputPin
//{
//private:
//    GPIO_pin pin;
//public:
//    AnalogInputPin( GPIO_pin );
//    float Value();
//};

//class DigitalOutputPin
//{
//private:
//    GPIO_pin pin;
//public:
//    DigitalOutputPin( GPIO_pin );
//    void Write( GPIOValue );
//    int PinStatus();
//    void Toggle();
//};

//// Initialize ADC ports
//void InitADCs();
//void InitClocks();


#endif // FEHIO_H
