#include "FEHIO.h"


tADC_Config AnalogInputPin::Master_Adc_Config;
tADC_Config AnalogInputPin::Encoder_Adc_Config;

typedef enum
{
    PortA,
    PortB,
    PortC,
    PortD,
    PortE
} GPIOPort;

typedef enum
{
    low,
    high
} GPIOValue;

typedef enum
{
    ADC0,
    ADC1
} ADCNumber;

const GPIOPort GPIOPorts[ 32 ] =
{
    PortB, PortB, PortB, PortB, PortB, PortB, PortB, PortB,
    PortC, PortC, PortC, PortC, PortC, PortC, PortA, PortA,
    PortA, PortA, PortA, PortA, PortA, PortA, PortA, PortA,
    PortA, PortE, PortE, PortE, PortE, PortE, PortD, PortD
};

const ADCNumber ADCNumbers[ 33 ] =
{
    ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1,
    ADC0, ADC0, ADC1, ADC1, ADC1, ADC1, ADC1, ADC0,
    ADC0, ADC1, ADC1, ADC0, ADC0, ADC1, ADC1, ADC0,
    ADC0, ADC0, ADC0, ADC0, ADC1, ADC0, ADC0, ADC0,
    ADC0
};

const int GPIOPinNumbers[ 32 ] =
{
    11, 10, 7, 6, 5, 4, 1, 0,
    0, 1, 8, 9, 10, 11, 17, 16,
    15, 14, 13, 12, 11, 10, 9, 8,
    7, 25, 24, 26, 27, 28, 1, 6
};

const int AnalogPinNumbers[ 33 ] =
{
    15, 14, 13, 12, 11, 10, 9, 8,
    14, 15, 4, 5, 6, 7, 17, 1,
    20, 1, 20, 0, 19, 0, 19, 11,
    10, 18, 17, 16, 16, 4, 5, 7,
    6
};


// Begin Functions for Digital Input Pin Type
DigitalInputPin::DigitalInputPin( FEHIO::FEHIOPin pin )
{
    Initialize( pin );
}

DigitalInputPin::DigitalInputPin()
{

}

void DigitalInputPin::Initialize( FEHIO::FEHIOPin pin )
{
    // store selected pin number in class
    _pin = pin;
    switch( GPIOPorts[ (int)_pin ] )
    {
        case PortA:
        {
            PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOA_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortB:
        {
            PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOB_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortC:
        {
            PORT_PCR_REG( PORTC_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOC_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortD:
        {
            PORT_PCR_REG( PORTD_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOD_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortE:
        {
            PORT_PCR_REG( PORTE_BASE_PTR, GPIOPinNumbers[ (int)_pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );
            GPIOE_PDDR &= ~GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
    }
}

bool DigitalInputPin::Value()
{
    int ret = 0;
    switch( GPIOPorts[ (int)_pin ] )
    {
        case PortA:
        {
            ret = GPIOA_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortB:
        {
            ret = GPIOB_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortC:
        {
            ret = GPIOC_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortD:
        {
            ret = GPIOD_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
        case PortE:
        {
            ret = GPIOE_PDIR & GPIO_PDIR_PDI( GPIO_PIN( GPIOPinNumbers[ (int)_pin ] ) );
            break;
        }
    }
    return ret;
}

// Begin Functions for Analog Input Pin Type
AnalogInputPin::AnalogInputPin( FEHIO::FEHIOPin _pin )
{
    pin = _pin;
}


//Analog read function causes digital output to behave strangely????????
float AnalogInputPin::Value()
{
    int analogPin = AnalogPinNumbers[pin];
        ADCNumber adcNum = ADCNumbers[pin];

         Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);
         Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);

        unsigned int result;

        // Disable Encoder Interrupts Temporarily
        NVICICER2 |= (1 << (4));

        if (adcNum == ADC0)
        {
             ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC0

             // Check the status control register to see is the COnversion is COmplete
             while (( ADC0_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
             result = ADC0_RA;
        }
        else
        {
             ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC0

             // Check the status control register to see is the COnversion is COmplete
             while (( ADC1_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
             result = ADC1_RA;

        }

        // Re-enable Encoder Interrupt
        NVICICPR2 |= (1 << (4));
        NVICISER2 |= (1 << (4));

        // Re-enable button Interrupt
        NVICICPR2 |= (1 << ( 26 ));
        NVICISER2 |= (1 << ( 26 ));


        float v = (result & 0xFFFFu) *3.33 / (0xFFFFu);
        return v;
}


int FEHEncoder::EncoderValue()
{
    int analogPin = AnalogPinNumbers[pin];
    ADCNumber adcNum = ADCNumbers[pin];

     Encoder_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);
     Encoder_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);

    int result;

    if (adcNum == ADC0)
    {
         ADC_Config_Alt(ADC0_BASE_PTR, &Encoder_Adc_Config);  // config ADC0

         // Check the status control register to see is the COnversion is COmplete
         while (( ADC0_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
         result = ADC0_RA;
    }
    else
    {
         ADC_Config_Alt(ADC1_BASE_PTR, &Encoder_Adc_Config);  // config ADC0

         // Check the status control register to see is the COnversion is COmplete
         while (( ADC1_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
         result = ADC1_RA;
    }
    return result;
}


// Begin Functions for Digital Output Pin Type
DigitalOutputPin::DigitalOutputPin( FEHIO::FEHIOPin _pin )
{
    // store selected pin number in class
    pin = _pin;
    switch( GPIOPorts[ (int)pin ] )
                {
                    case PortA:
                    {
                        PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOA_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortB:
                    {
                        PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOB_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortC:
                    {
                        PORT_PCR_REG( PORTC_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOC_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortD:
                    {
                        PORT_PCR_REG( PORTD_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOD_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                    case PortE:
                    {
                        PORT_PCR_REG( PORTE_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
                        GPIOE_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                        break;
                    }
                }
}

void DigitalOutputPin::Write( bool value )
{
    switch( value )
    {
        case true:
        {
            switch( GPIOPorts[ (int)pin ] )
            {
                case PortA:
                {
                    GPIOA_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                 }
                case PortB:
                {
                    GPIOB_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortC:
                {
                    GPIOC_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortD:
                {
                    GPIOD_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortE:
                {
                    GPIOE_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
            }
            break;
        }
        case false:
        {
            switch( GPIOPorts[ (int)pin ] )
            {
                case PortA:
                {
                    GPIOA_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortB:
                {
                    GPIOB_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortC:
                {
                    GPIOC_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortD:
                {
                    GPIOD_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
                case PortE:
                {
                    GPIOE_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                    break;
                }
            }
        }
    }
}

int DigitalOutputPin::Status()
{
    int ret = 0;
        switch( GPIOPorts[ (int)pin ] )
        {
            case PortA:
            {
                ret = GPIOA_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortB:
            {
                ret = GPIOB_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortC:
            {
                ret = GPIOC_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortD:
            {
                ret = GPIOD_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortE:
            {
                ret = GPIOE_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
        }
        return ret;
}

void DigitalOutputPin::Toggle()
{
    switch( GPIOPorts[ (int)pin ] )
        {
            case PortA:
            {
                GPIOA_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortB:
            {
                GPIOB_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortC:
            {
                GPIOC_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortD:
            {
                GPIOD_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
            case PortE:
            {
                GPIOE_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
                break;
            }
        }

}

////Initialize ADC Function. Needs to be placed somewhere??
void AnalogInputPin::InitADCs()
{
    // setup the initial ADC default configuration
         Master_Adc_Config.CONFIG1  = ADLPC_NORMAL                   // Normal power, (not low power)
                                    | ADC_CFG1_ADIV(ADIV_4)          // Clock divider
                                    | ADLSMP_LONG                    // Take a long time to sample
                                    | ADC_CFG1_MODE(MODE_16)         // 16 bit mode
                                    | ADC_CFG1_ADICLK(ADICLK_BUS);   // use the bus clock
         Master_Adc_Config.CONFIG2  = MUXSEL_ADCB                    // use channel A
                                    | ADACKEN_DISABLED               // Asynch clock disabled?
                                    | ADHSC_NORMAL                   // Asynch clock setting
                                    | ADC_CFG2_ADLSTS(ADLSTS_20) ;
         Master_Adc_Config.COMPARE1 = 0x1234u ;                 // can be anything
         Master_Adc_Config.COMPARE2 = 0x5678u ;                 // can be anything
                                                                // since not using
                                                                // compare feature
         Master_Adc_Config.STATUS2  = ADTRG_SW                  // Software triggered conversion
                                    | ACFE_DISABLED             // Disable comparator (if enabled only registers as an anlog reading if it is greater than a certain value)
                                    | ACFGT_GREATER             // comparator setting
                                    | ACREN_DISABLED            // Compare Function Range disabled
                                    | DMAEN_DISABLED               // Disable DMA
                                    | ADC_SC2_REFSEL(REFSEL_EXT); // external voltage reference

         Master_Adc_Config.STATUS3  = CAL_OFF                     // Calibration begins off
                                    | ADCO_SINGLE                 // Take a single reading
                                    | AVGE_ENABLED                // Enable averaging
                                    | ADC_SC3_AVGS(AVGS_32);      // Average 32 samples

         Master_Adc_Config.PGA      = PGAEN_DISABLED             // PGA disabled
                                    | PGACHP_NOCHOP              // no chopping for PGA?
                                    | PGALP_NORMAL               // Normal (not low power mode)
                                    | ADC_PGA_PGAG(PGAG_64);     // PGA gain of 64

         // Set up channel as all ones for configuration
         Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);

         Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);


        // Configure ADC as it will be used, but becuase ADC_SC1_ADCH is 31,
        // the ADC will be inactive.  Channel 31 is just disable function.
        // There really is no channel 31.

         ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC
         ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC

        // Calibrate the ADC in the configuration in which it will be used:
         ADC_Cal(ADC1_BASE_PTR);                    // do the calibration
         ADC_Cal(ADC0_BASE_PTR);                    // do the calibration

        // The structure still has the desired configuration.  So restore it.
        // Why restore it?  The calibration makes some adjustments to the
        // configuration of the ADC.  The are now undone:

        // config the ADC again to desired conditions
         ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);
         ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);

         //Load Encoder ADC Config (A bit different from Master)
         Encoder_Adc_Config = Master_Adc_Config;

         Encoder_Adc_Config.CONFIG1  = ADLPC_NORMAL                   // Normal power, (not low power)
                                    | ADC_CFG1_ADIV(ADIV_4)          // Clock divider
                                    | ADLSMP_LONG                    // Take a long time to sample
                                    | ADC_CFG1_MODE(MODE_16)         // 16 bit mode
                                    | ADC_CFG1_ADICLK(ADICLK_BUS);   // use the bus clock

         Encoder_Adc_Config.STATUS3  = CAL_OFF                     // Calibration begins off
                                    | ADCO_SINGLE                 // Take a single reading
                                    | AVGE_ENABLED                // Enable averaging
                                    | ADC_SC3_AVGS(AVGS_4);      // Average 4 samples
}

int FEHEncoder::numEncoders = 0;
FEHEncoder * FEHEncoder::encoders[32];

void pit0_isr(void) {
    FEHEncoder::ProcessInt();
    PIT_TFLG0 = PIT_TFLG_TIF_MASK;
    return;
}

void FEHEncoder::Init() {
    // Freeze on Debug
    PIT_MCR = PIT_MCR_FRZ_MASK;

    // Load wait period
    SetCounterInit(0x186A0u);

    //#define PIT_CVAL0                                PIT_CVAL_REG(PIT_BASE_PTR,0)

    // Enable PINT0 and interrupt
    PIT_TCTRL0 = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    // Other Vector Interrupt Register
    NVICICPR2 |= 1 << (4);
    NVICISER2 |= 1 << (4);
}

void FEHEncoder::SetCounterInit(unsigned int val) {
    PIT_LDVAL0  =  val;
}

FEHEncoder::FEHEncoder(FEHIO::FEHIOPin pin_) : AnalogInputPin(pin_){
    state = LOW_STATE;
    encoders[numEncoders] = this;
    numEncoders++;
    ResetCounts();
    lowThreshold = 70*256;
    highThreshold = 210*256;
}

void FEHEncoder::SetThresholds(float low, float high) {
    lowThreshold = low/3.3*0x10000;
    highThreshold = high/3.3*0x10000;
}

void FEHEncoder::ProcessInt() {
    for(int i=0; i< numEncoders; i++)
    {
        encoders[i]->ProcessIntSelf();
    }
}

void FEHEncoder::ProcessIntSelf() {
    int value = EncoderValue();
    if(state == LOW_STATE && value>highThreshold) {
        state = HIGH_STATE;
        counts++;
    }
    if(state== HIGH_STATE && value<lowThreshold) {
        state = LOW_STATE;
        counts++;
    }
}

int FEHEncoder::Counts()
{
    return (int) counts;
}

void FEHEncoder::ResetCounts()
{
    counts=0;
}


ButtonBoard::ButtonBoard( FEHIO::FEHIOPort bank )
{
    switch( bank )
    {
        case FEHIO::Bank0:
        {
            _left.Initialize( FEHIO::P0_0 );
            _middle.Initialize( FEHIO::P0_1 );
            _right.Initialize( FEHIO::P0_2 );
            break;
        }
        case FEHIO::Bank1:
        {
            _left.Initialize( FEHIO::P1_0 );
            _middle.Initialize( FEHIO::P1_1 );
            _right.Initialize( FEHIO::P1_2 );
            break;
        }
        case FEHIO::Bank2:
        {
            _left.Initialize( FEHIO::P2_0 );
            _middle.Initialize( FEHIO::P2_1 );
            _right.Initialize( FEHIO::P2_2 );
            break;
        }
        case FEHIO::Bank3:
        {
            _left.Initialize( FEHIO::P3_0 );
            _middle.Initialize( FEHIO::P3_1 );
            _right.Initialize( FEHIO::P3_2 );
            break;
        }
    }
}

bool ButtonBoard::LeftPressed()
{
    return ( _left.Value() == 0 );
}

bool ButtonBoard::LeftReleased()
{
    return ( _left.Value() == 1 );
}

bool ButtonBoard::MiddlePressed()
{
    return ( _middle.Value() == 0 );
}

bool ButtonBoard::MiddleReleased()
{
    return ( _middle.Value() == 1 );
}

bool ButtonBoard::RightPressed()
{
    return ( _right.Value() == 0 );
}

bool ButtonBoard::RightReleased()
{
    return ( _right.Value() == 1 );
}
