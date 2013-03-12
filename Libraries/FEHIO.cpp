#include "FEHIO.h"

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

tADC_Config Master_Adc_Config;

const GPIOPort GPIOPorts[ 32 ] =
{
    PortB, PortB, PortB, PortB, PortB, PortB, PortB, PortB,
    PortC, PortC, PortC, PortC, PortC, PortC, PortA, PortA,
    PortA, PortA, PortA, PortA, PortA, PortA, PortA, PortA,
    PortA, PortE, PortE, PortE, PortE, PortE, PortD, PortD
};

const ADCNumber ADCNumbers[ 32 ] =
{
    ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1,
    ADC0, ADC0, ADC1, ADC1, ADC1, ADC1, ADC1, ADC0,
    ADC0, ADC1, ADC1, ADC0, ADC0, ADC1, ADC1, ADC0,
    ADC0, ADC0, ADC0, ADC0, ADC1, ADC0, ADC0, ADC0
};

const int GPIOPinNumbers[ 32 ] =
{
    11, 10, 7, 6, 5, 4, 1, 0,
    0, 1, 8, 9, 10, 11, 17, 16,
    15, 14, 13, 12, 11, 10, 9, 8,
    7, 25, 24, 26, 27, 28, 1, 6
};

const int AnalogPinNumbers[ 32 ] =
{
    15, 14, 13, 12, 11, 10, 9, 8,
    14, 15, 4, 5, 6, 7, 17, 1,
    20, 1, 20, 0, 19, 0, 19, 11,
    10, 18, 17, 16, 16, 4, 5, 7
};


// Begin Functions for Digital Input Pin Type
DigitalInputPin::DigitalInputPin( FEHIO::FEHIOPin pin )
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

int DigitalInputPin::Value()
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

//// Begin Functions for Analog Input Pin Type
//AnalogInputPin::AnalogInputPin( GPIO_pin _pin )
//{
//    pin = _pin;
//}


//float AnalogInputPin::Value()
//{
//    int analogPin = AnalogPinNumbers[pin];
//        ADCNumber adcNum = ADCNumbers[pin];

//         Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);
//         Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);

//        int result;

//        if (adcNum == ADC0)
//        {
//             ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC0

//             // Check the status control register to see is the COnversion is COmplete
//             while (( ADC0_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
//             result = ADC0_RA;
//        }
//        else
//        {
//             ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC0

//             // Check the status control register to see is the COnversion is COmplete
//             while (( ADC1_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK){}
//             result = ADC1_RA;

//        }

//        float v = result *3.33 / (0x10000);
//}


//// Begin Functions for Digital Output Pin Type
//DigitalOutputPin::DigitalOutputPin( GPIO_pin _pin )
//{
//    // store selected pin number in class
//    pin = _pin;
//    switch( GPIOPorts[ (int)pin ] )
//                {
//                    case PortA:
//                    {
//                        PORT_PCR_REG( PORTA_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
//                        GPIOA_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                        break;
//                    }
//                    case PortB:
//                    {
//                        PORT_PCR_REG( PORTB_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
//                        GPIOB_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                        break;
//                    }
//                    case PortC:
//                    {
//                        PORT_PCR_REG( PORTC_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
//                        GPIOC_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                        break;
//                    }
//                    case PortD:
//                    {
//                        PORT_PCR_REG( PORTD_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
//                        GPIOD_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                        break;
//                    }
//                    case PortE:
//                    {
//                        PORT_PCR_REG( PORTE_BASE_PTR, GPIOPinNumbers[ (int)pin ] ) = ( 0 | PORT_PCR_MUX( 1 ) );
//                        GPIOE_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                        break;
//                    }
//                }
//}

//void DigitalOutputPin::Write( GPIOValue value )
//{
//    switch( value )
//    {
//        case high:
//        {
//            switch( GPIOPorts[ (int)pin ] )
//            {
//                case PortA:
//                {
//                    GPIOA_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                 }
//                case PortB:
//                {
//                    GPIOB_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//                case PortC:
//                {
//                    GPIOC_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//                case PortD:
//                {
//                    GPIOD_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//                case PortE:
//                {
//                    GPIOE_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//            }
//        }
//        case low:
//        {
//            switch( GPIOPorts[ (int)pin ] )
//            {
//                case PortA:
//                {
//                    GPIOA_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//                case PortB:
//                {
//                    GPIOB_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//                case PortC:
//                {
//                    GPIOC_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//                case PortD:
//                {
//                    GPIOD_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//                case PortE:
//                {
//                    GPIOE_PDOR &= ~GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                    break;
//                }
//            }
//        }
//    }
//}

//int DigitalOutputPin::PinStatus()
//{
//    int ret = 0;
//        switch( GPIOPorts[ (int)pin ] )
//        {
//            case PortA:
//            {
//                ret = GPIOA_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortB:
//            {
//                ret = GPIOB_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortC:
//            {
//                ret = GPIOC_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortD:
//            {
//                ret = GPIOD_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortE:
//            {
//                ret = GPIOE_PDOR & GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//        }
//        return ret;
//}

//void DigitalOutputPin::Toggle()
//{
//    switch( GPIOPorts[ (int)pin ] )
//        {
//            case PortA:
//            {
//                GPIOA_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortB:
//            {
//                GPIOB_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortC:
//            {
//                GPIOC_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortD:
//            {
//                GPIOD_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//            case PortE:
//            {
//                GPIOE_PTOR |= GPIO_PDOR_PDO( GPIO_PIN( GPIOPinNumbers[ (int)pin ] ) );
//                break;
//            }
//        }

//}

////Initialize ADC Function. Needs to be placed somewhere??

//void InitADCs()
//{
//    // setup the initial ADC default configuration
//         Master_Adc_Config.CONFIG1  = ADLPC_NORMAL                   // Normal power, (not low power)
//                                    | ADC_CFG1_ADIV(ADIV_4)          // Clock divider
//                                    | ADLSMP_LONG                    // Take a long time to sample
//                                    | ADC_CFG1_MODE(MODE_16)         // 16 bit mode
//                                    | ADC_CFG1_ADICLK(ADICLK_BUS);   // use the bus clock
//         Master_Adc_Config.CONFIG2  = MUXSEL_ADCB                    // use channel A
//                                    | ADACKEN_DISABLED               // Asynch clock disabled?
//                                    | ADHSC_NORMAL                   // Asynch clock setting
//                                    | ADC_CFG2_ADLSTS(ADLSTS_20) ;
//         Master_Adc_Config.COMPARE1 = 0x1234u ;                 // can be anything
//         Master_Adc_Config.COMPARE2 = 0x5678u ;                 // can be anything
//                                                                // since not using
//                                                                // compare feature
//         Master_Adc_Config.STATUS2  = ADTRG_SW                  // Software triggered conversion
//                                    | ACFE_DISABLED             // Disable comparator (if enabled only registers as an anlog reading if it is greater than a certain value)
//                                    | ACFGT_GREATER             // comparator setting
//                                    | ACREN_DISABLED            // Compare Function Range disabled
//                                    | DMAEN_DISABLED               // Disable DMA
//                                    | ADC_SC2_REFSEL(REFSEL_EXT); // external voltage reference

//         Master_Adc_Config.STATUS3  = CAL_OFF                     // Calibration begins off
//                                    | ADCO_SINGLE                 // Take a single reading
//                                    | AVGE_ENABLED                // Enable averaging
//                                    | ADC_SC3_AVGS(AVGS_32);      // Average 32 samples

//         Master_Adc_Config.PGA      = PGAEN_DISABLED             // PGA disabled
//                                    | PGACHP_NOCHOP              // no chopping for PGA?
//                                    | PGALP_NORMAL               // Normal (not low power mode)
//                                    | ADC_PGA_PGAG(PGAG_64);     // PGA gain of 64

//         // Set up channel as all ones for configuration
//         Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);

//// start of area that conflicts with digital
//         Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(31);


//        // Configure ADC as it will be used, but becuase ADC_SC1_ADCH is 31,
//        // the ADC will be inactive.  Channel 31 is just disable function.
//        // There really is no channel 31.

//         ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC
//         ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC

//        // Calibrate the ADC in the configuration in which it will be used:
//         ADC_Cal(ADC1_BASE_PTR);                    // do the calibration
//         ADC_Cal(ADC0_BASE_PTR);                    // do the calibration

//        // The structure still has the desired configuration.  So restore it.
//        // Why restore it?  The calibration makes some adjustments to the
//        // configuration of the ADC.  The are now undone:

//        // config the ADC again to desired conditions
//         ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);
//         ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);
//}

//// initialize clocks for GPIO and the ADCs, This will also be moved somewhere else...
//void InitClocks()
//{
//    // Clocks for GPIO
//    SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

//    // Clocks for ADC
//    // Turn on the ADC0 and ADC1 clocks
//     SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK );
//     SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK );
//}
