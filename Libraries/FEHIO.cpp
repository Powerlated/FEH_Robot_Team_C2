#include "CMSIS/ARMCM4.h"
#include "FEHIO.h"
#include <optional>
#include "FEHLCD.h"

tADC_Config AnalogInputPin::Master_Adc_Config;
tADC_Config AnalogInputPin::Encoder_Adc_Config;

typedef enum {
    PortA,
    PortB,
    PortC,
    PortD,
    PortE
} GPIOPort;

typedef enum {
    low,
    high
} GPIOValue;

typedef enum {
    ADC0,
    ADC1
} ADCNumber;

constexpr GPIOPort GPIOPorts[32] =
        {
                PortB, PortB, PortB, PortB, PortB, PortB, PortB, PortB,
                PortC, PortC, PortC, PortC, PortC, PortC, PortA, PortA,
                PortA, PortA, PortA, PortA, PortA, PortA, PortA, PortA,
                PortA, PortE, PortE, PortE, PortE, PortE, PortD, PortD
        };

constexpr ADCNumber ADCNumbers[33] =
        {
                ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1, ADC1,
                ADC0, ADC0, ADC1, ADC1, ADC1, ADC1, ADC1, ADC0,
                ADC0, ADC1, ADC1, ADC0, ADC0, ADC1, ADC1, ADC0,
                ADC0, ADC0, ADC0, ADC0, ADC1, ADC0, ADC0, ADC0,
                ADC0
        };

constexpr int GPIOPinNumbers[32] =
        {
                11, 10, 7, 6, 5, 4, 1, 0,
                0, 1, 8, 9, 10, 11, 17, 16,
                15, 14, 13, 12, 11, 10, 9, 8,
                7, 25, 24, 26, 27, 28, 1, 6
        };

constexpr int AnalogPinNumbers[33] =
        {
                15, 14, 13, 12, 11, 10, 9, 8,
                14, 15, 4, 5, 6, 7, 17, 1,
                20, 1, 20, 0, 19, 0, 19, 11,
                10, 18, 17, 16, 16, 4, 5, 7,
                6
        };

// Begin Functions for Digital Input Pin Type
DigitalInputPin::DigitalInputPin(FEHIO::FEHIOPin pin) {
    Initialize(pin);
}

DigitalInputPin::DigitalInputPin() {

}

void DigitalInputPin::Initialize(FEHIO::FEHIOPin pin) {
    // store selected pin number in class
    _pin = pin;
    switch (GPIOPorts[(int) _pin]) {
        case PortA: {
            PORT_PCR_REG(PORTA_BASE_PTR, GPIOPinNumbers[(int) _pin]) = (0 | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |
                                                                        PORT_PCR_PS_MASK);
            GPIOA_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortB: {
            PORT_PCR_REG(PORTB_BASE_PTR, GPIOPinNumbers[(int) _pin]) = (0 | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |
                                                                        PORT_PCR_PS_MASK);
            GPIOB_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortC: {
            PORT_PCR_REG(PORTC_BASE_PTR, GPIOPinNumbers[(int) _pin]) = (0 | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |
                                                                        PORT_PCR_PS_MASK);
            GPIOC_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortD: {
            PORT_PCR_REG(PORTD_BASE_PTR, GPIOPinNumbers[(int) _pin]) = (0 | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |
                                                                        PORT_PCR_PS_MASK);
            GPIOD_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortE: {
            PORT_PCR_REG(PORTE_BASE_PTR, GPIOPinNumbers[(int) _pin]) = (0 | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK |
                                                                        PORT_PCR_PS_MASK);
            GPIOE_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
    }
}

bool DigitalInputPin::Value() {
    int ret = 0;
    switch (GPIOPorts[(int) _pin]) {
        case PortA: {
            ret = GPIOA_PDIR & GPIO_PDIR_PDI(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortB: {
            ret = GPIOB_PDIR & GPIO_PDIR_PDI(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortC: {
            ret = GPIOC_PDIR & GPIO_PDIR_PDI(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortD: {
            ret = GPIOD_PDIR & GPIO_PDIR_PDI(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
        case PortE: {
            ret = GPIOE_PDIR & GPIO_PDIR_PDI(GPIO_PIN(GPIOPinNumbers[(int) _pin]));
            break;
        }
    }
    return ret;
}

DigitalEncoder *encoder_pinsA[32]{};
DigitalEncoder *encoder_pinsB[32]{};

DigitalEncoder::DigitalEncoder(FEHIO::FEHIOPin pin1, FEHIO::FEHIOPin pin2) : pin1(pin1), pin2(pin2) {
    encoder_pinsA[pin1] = this;
    encoder_pinsB[pin2] = this;
    SetupGPIO(pin1);
    SetupGPIO(pin2);
}

void DigitalEncoder::ChannelAEdge(bool is_high) {
    channel_a_high = is_high;

    if (channel_a_high) {
        if (channel_b_high) {
            counts++;
        } else {
            counts--;
        }
    } else {
        if (channel_b_high) {
            counts--;
        } else {
            counts++;
        }
    }
}

void DigitalEncoder::ChannelBEdge(bool is_high) {
    channel_b_high = is_high;

    if (channel_b_high) {
        if (channel_a_high) {
            counts--;
        } else {
            counts++;
        }
    } else {
        if (channel_a_high) {
            counts++;
        } else {
            counts--;
        }
    }
}

int DigitalEncoder::Counts() const { return counts; }

void DigitalEncoder::ResetCounts() {
	// Block interrupts during DigitalEncoder::ResetCounts to prevent race condition
	__set_PRIMASK(1);
	counts = 0;
	__set_PRIMASK(0);
}

constexpr void DigitalEncoder::SetupGPIO(FEHIO::FEHIOPin pin) {
    auto trig = (unsigned char) FEHIO::FEHIOInterruptTrigger::EitherEdge;
    uint32_t pcr = 0 | PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(trig) | PORT_PCR_PFE_MASK;
    auto gpio_pin_number = GPIOPinNumbers[pin];
    switch (GPIOPorts[pin]) {
        case PortA: {
            PORTA_BASE_PTR->PCR[gpio_pin_number] = pcr;
            GPIOA_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(gpio_pin_number));
            NVIC_EnableIRQ((IRQn_Type)(INT_PORTA - 16));
            break;
        }
        case PortB: {
            PORTB_BASE_PTR->PCR[gpio_pin_number] = pcr;
            GPIOB_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(gpio_pin_number));
			NVIC_EnableIRQ((IRQn_Type)(INT_PORTB - 16));
            break;
        }
        case PortC: {
            PORTC_BASE_PTR->PCR[gpio_pin_number] = pcr;
            GPIOC_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(gpio_pin_number));
			NVIC_EnableIRQ((IRQn_Type)(INT_PORTC - 16));
            break;
        }
        case PortD: {
            //Port D is already in use for power reset pin. Therefore Digital Encoders cannot be used on P3_6 and P3_7
            break;
        }
        case PortE: {
            PORTE_BASE_PTR->PCR[gpio_pin_number] = pcr;
            GPIOE_PDDR &= ~GPIO_PDDR_PDD(GPIO_PIN(gpio_pin_number));
			NVIC_EnableIRQ((IRQn_Type)(INT_PORTE - 16));
            break;
        }
    }
}

/*
 * Called by the PORT IRQ handlers to resolve Proteus logical pin numbers from interrupt bit flags,
 * and update the DigitalEncoders.
 */
void pin_isr(uint32_t port_ifsr, uint32_t gpio_pdir, const int8_t bit_pin_map[]) {
    // Resolve the logical pin numbers from the PORT IFSR (Interrupt Flag Status Register)
    int bit = 0;
    while (port_ifsr != 0) {
        // __builtin_ctz counts the 0 bits at the end of port_ifsr
        int tz = __builtin_ctz(port_ifsr);
        // check to see if the bit we want to look at is set
        bool bit_set = port_ifsr & (1 << tz);
        // Shift away the trailing zeros AND the set bit
        port_ifsr >>= tz + 1;
        // Add tz to the bit counter so bit becomes the index of the set bit
        bit += tz;

        if (bit_set) {
            int8_t pin = bit_pin_map[bit];
            // A pin of -1 indicates that a PORT bit is not associated with a Proteus pin.
            if (pin != -1) {
                // Update encoders
                bool pin_high = gpio_pdir & (1 << bit);
                if (encoder_pinsA[pin] != nullptr) {
                    encoder_pinsA[pin]->ChannelAEdge(pin_high);
                }
                if (encoder_pinsB[pin] != nullptr) {
                    encoder_pinsB[pin]->ChannelBEdge(pin_high);
                }
            }
        }

        // Now add 1 to the bit counter, so it becomes the index of the bit after the set bit
        bit++;
    }
}

/*
 * These arrays map PORT/GPIO bit numbers to Proteus logical pin numbers.
 * A pin of -1 indicates that a PORT bit is not associated with a Proteus pin.
 *
 * Generated from the original code using JavaScript.
 *
 * Example for Port A:
 *
 * let arr = [17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7];
 * let out = Array(32).fill(-1);
 * let pinOffs = 0;
 *
 * for (let i = 0; i < arr.length; i++) {
 *     let bit = arr[i];
 *     let pin = i;
 *
 *     out[bit] = pin + pinOffs;
 * }
 *
 * console.log(out);
 */

int8_t port_b_bit_pin_map[] = {7, 6, -1, -1, 5, 4, 3, 2,
                               -1, -1, 1, 0, -1, -1, -1, -1,
                               -1, -1, -1, -1, -1, -1, -1, -1,
                               -1, -1, -1, -1, -1, -1, -1, -1};
int8_t port_c_bit_pin_map[] = {8, 9, -1, -1, -1, -1, -1, -1,
                               10, 11, 12, 13, -1, -1, -1, -1,
                               -1, -1, -1, -1, -1, -1, -1,
                               -1, -1, -1, -1, -1, -1, -1, -1, -1};
int8_t port_a_bit_pin_map[] = {-1, -1, -1, -1, -1, -1, -1, 24,
                               23, 22, 21, 20, 19, 18, 17, 16,
                               15, 14, -1, -1, -1, -1, -1, -1,
                               -1, -1, -1, -1, -1, -1, -1, -1};
int8_t port_e_bit_pin_map[] = {-1, -1, -1, -1, -1, -1, -1, -1,
                               -1, -1, -1, -1, -1, -1, -1, -1,
                               -1, -1, -1, -1, -1, -1, -1, -1,
                               26, 25, 27, 28, 29, -1, -1, -1};

/*
 * These handle interrupts for DigitalEncoder.
 * PORT ISFR registers are write 1 to clear, so writing 0xFFFFFFFF will clear the entire register.
 */

void PORTB_IRQHandler() {
    pin_isr(PORTB_ISFR, GPIOB_PDIR, port_b_bit_pin_map);
    PORTB_ISFR = 0xFFFFFFFF;
}

void PORTC_IRQHandler() {
    pin_isr(PORTC_ISFR, GPIOC_PDIR, port_c_bit_pin_map);
    PORTC_ISFR = 0xFFFFFFFF;
}

void PORTA_IRQHandler() {
    pin_isr(PORTA_ISFR, GPIOA_PDIR, port_a_bit_pin_map);
    PORTA_ISFR = 0xFFFFFFFF;
}

void PORTE_IRQHandler() {
    pin_isr(PORTE_ISFR, GPIOE_PDIR, port_e_bit_pin_map);
    PORTE_ISFR = 0xFFFFFFFF;
}

// Begin Functions for Analog Input Pin Type
AnalogInputPin::AnalogInputPin(FEHIO::FEHIOPin _pin) {
    pin = _pin;
}


//Analog read function causes digital output to behave strangely????????
float AnalogInputPin::Value() {
    int analogPin = AnalogPinNumbers[pin];
    int adcNum = ADCNumbers[pin];

    Master_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);
    Master_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);

    unsigned int result;

    if (adcNum == 0) {
        ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // config ADC0

        // Check the status control register to see is the COnversion is COmplete
        while ((ADC0_SC1A & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK) {}
        result = ADC0_RA;
    } else {
        ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  // config ADC0

        // Check the status control register to see is the COnversion is COmplete
        while ((ADC1_SC1A & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK) {}
        result = ADC1_RA;

    }

    float v = (result & 0xFFFFu) * 3.33 / (0xFFFFu);
    return v;
}


int AnalogEncoder::EncoderValue() {
    int analogPin = AnalogPinNumbers[pin];
    int adcNum = ADCNumbers[pin];

    Encoder_Adc_Config.STATUS1A = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);
    Encoder_Adc_Config.STATUS1B = AIEN_OFF | DIFF_SINGLE | ADC_SC1_ADCH(analogPin);

    int result;

    if (adcNum == 0) {
        ADC_Config_Alt(ADC0_BASE_PTR, &Encoder_Adc_Config);  // config ADC0

        // Check the status control register to see is the COnversion is COmplete
        while ((ADC0_SC1A & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK) {}
        result = ADC0_RA;
    } else {
        ADC_Config_Alt(ADC1_BASE_PTR, &Encoder_Adc_Config);  // config ADC0

        // Check the status control register to see is the COnversion is COmplete
        while ((ADC1_SC1A & ADC_SC1_COCO_MASK) != ADC_SC1_COCO_MASK) {}
        result = ADC1_RA;
    }
    return result;
}


// Begin Functions for Digital Output Pin Type
DigitalOutputPin::DigitalOutputPin(FEHIO::FEHIOPin _pin) {
    // store selected pin number in class
    pin = _pin;
    switch (GPIOPorts[(int) pin]) {
        case PortA: {
            PORT_PCR_REG(PORTA_BASE_PTR, GPIOPinNumbers[(int) pin]) = (0 | PORT_PCR_MUX(1));
            GPIOA_PDDR |= GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortB: {
            PORT_PCR_REG(PORTB_BASE_PTR, GPIOPinNumbers[(int) pin]) = (0 | PORT_PCR_MUX(1));
            GPIOB_PDDR |= GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortC: {
            PORT_PCR_REG(PORTC_BASE_PTR, GPIOPinNumbers[(int) pin]) = (0 | PORT_PCR_MUX(1));
            GPIOC_PDDR |= GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortD: {
            PORT_PCR_REG(PORTD_BASE_PTR, GPIOPinNumbers[(int) pin]) = (0 | PORT_PCR_MUX(1));
            GPIOD_PDDR |= GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortE: {
            PORT_PCR_REG(PORTE_BASE_PTR, GPIOPinNumbers[(int) pin]) = (0 | PORT_PCR_MUX(1));
            GPIOE_PDDR |= GPIO_PDDR_PDD(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
    }
}

void DigitalOutputPin::Write(bool value) {
    switch (value) {
        case true: {
            switch (GPIOPorts[(int) pin]) {
                case PortA: {
                    GPIOA_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortB: {
                    GPIOB_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortC: {
                    GPIOC_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortD: {
                    GPIOD_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortE: {
                    GPIOE_PDOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
            }
            break;
        }
        case false: {
            switch (GPIOPorts[(int) pin]) {
                case PortA: {
                    GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortB: {
                    GPIOB_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortC: {
                    GPIOC_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortD: {
                    GPIOD_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
                case PortE: {
                    GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
                    break;
                }
            }
        }
    }
}

bool DigitalOutputPin::Status() {
    int ret = 0;
    switch (GPIOPorts[(int) pin]) {
        case PortA: {
            ret = GPIOA_PDOR & GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortB: {
            ret = GPIOB_PDOR & GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortC: {
            ret = GPIOC_PDOR & GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortD: {
            ret = GPIOD_PDOR & GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortE: {
            ret = GPIOE_PDOR & GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
    }
    return ret;
}

void DigitalOutputPin::Toggle() {
    switch (GPIOPorts[(int) pin]) {
        case PortA: {
            GPIOA_PTOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortB: {
            GPIOB_PTOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortC: {
            GPIOC_PTOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortD: {
            GPIOD_PTOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
        case PortE: {
            GPIOE_PTOR |= GPIO_PDOR_PDO(GPIO_PIN(GPIOPinNumbers[(int) pin]));
            break;
        }
    }

}

////Initialize ADC Function. Needs to be placed somewhere??
void AnalogInputPin::InitADCs() {
    // setup the initial ADC default configuration
    Master_Adc_Config.CONFIG1 = ADLPC_NORMAL                   // Normal power, (not low power)
                                | ADC_CFG1_ADIV(ADIV_4)          // Clock divider
                                | ADLSMP_LONG                    // Take a long time to sample
                                | ADC_CFG1_MODE(MODE_16)         // 16 bit drivetrain_mode
                                | ADC_CFG1_ADICLK(ADICLK_BUS);   // use the bus clock
    Master_Adc_Config.CONFIG2 = MUXSEL_ADCB                    // use channel A
                                | ADACKEN_DISABLED               // Asynch clock disabled?
                                | ADHSC_NORMAL                   // Asynch clock setting
                                | ADC_CFG2_ADLSTS(ADLSTS_20);
    Master_Adc_Config.COMPARE1 = 0x1234u;                 // can be anything
    Master_Adc_Config.COMPARE2 = 0x5678u;                 // can be anything
    // since not using
    // compare feature
    Master_Adc_Config.STATUS2 = ADTRG_SW                  // Software triggered conversion
                                |
                                ACFE_DISABLED             // Disable comparator (if enabled only registers as an anlog reading if it is greater than a certain value)
                                | ACFGT_GREATER             // comparator setting
                                | ACREN_DISABLED            // Compare Function Range disabled
                                | DMAEN_DISABLED               // Disable DMA
                                | ADC_SC2_REFSEL(REFSEL_EXT); // external voltage reference

    Master_Adc_Config.STATUS3 = CAL_OFF                     // Calibration begins off
                                | ADCO_SINGLE                 // Take a single reading
                                | AVGE_ENABLED                // Enable averaging
                                | ADC_SC3_AVGS(AVGS_32);      // Average 32 samples

    Master_Adc_Config.PGA = PGAEN_DISABLED             // PGA disabled
                            | PGACHP_NOCHOP              // no chopping for PGA?
                            | PGALP_NORMAL               // Normal (not low power drivetrain_mode)
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

    Encoder_Adc_Config.CONFIG1 = ADLPC_NORMAL                   // Normal power, (not low power)
                                 | ADC_CFG1_ADIV(ADIV_4)          // Clock divider
                                 | ADLSMP_LONG                    // Take a long time to sample
                                 | ADC_CFG1_MODE(MODE_16)         // 16 bit drivetrain_mode
                                 | ADC_CFG1_ADICLK(ADICLK_BUS);   // use the bus clock

    Encoder_Adc_Config.STATUS3 = CAL_OFF                     // Calibration begins off
                                 | ADCO_SINGLE                 // Take a single reading
                                 | AVGE_ENABLED                // Enable averaging
                                 | ADC_SC3_AVGS(AVGS_4);      // Average 4 samples
}

#define NULL 0
AnalogEncoder::PinInfo *AnalogEncoder::pinList = NULL;

volatile long __interrupt_counter = 0;

void PIT0_IRQHandler() {
    __interrupt_counter++;
    AnalogEncoder::ProcessInt();
    PIT_TFLG0 = PIT_TFLG_TIF_MASK;
    return;
}

void AnalogEncoder::SetRate(unsigned int rateHz) {
    unsigned long countDown = 44000000UL / rateHz;
    if (countDown < 0x100000000UL) {
        PIT_LDVAL0 = countDown;
    }
}

void AnalogEncoder::Init() {
    // Freeze on Debug
    PIT_MCR = PIT_MCR_FRZ_MASK;


    // Load wait period (100,000 clock divider = 440 Hz interrupts)
    SetCounterInit(0x186A0);

    //#define PIT_CVAL0                                PIT_CVAL_REG(PIT_BASE_PTR,0)

    // Enable PINT0 and interrupt
    PIT_TCTRL0 = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    // Other Vector Interrupt Register
    NVICICPR2 |= 1 << (4);
    NVICISER2 |= 1 << (4);
}

void AnalogEncoder::SetCounterInit(unsigned int val) {
    PIT_LDVAL0 = val;
}

AnalogEncoder::AnalogEncoder(FEHIO::FEHIOPin pin_) : AnalogInputPin(pin_) {
    if (pinList == NULL) {
        // Initialize linked list of encoders within the pPinInfo Object
        this->pNext = NULL;
        this->pPrev = NULL;

        // Add encoder linked list to pinInfo
        pPinInfo = new PinInfo;
        pPinInfo->encoderList = this;
        pPinInfo->numEncoders = 1;
        pPinInfo->pin = pin_;
        pPinInfo->pNext = NULL;
        pPinInfo->pPrev = NULL;
        pPinInfo->state = LOW_STATE;

        // Since the pinList is empty, set the pinInfo to the head
        pinList = pPinInfo;
    } else // There is already a list in place (see if this pin is already in it)
    {
        PinInfo *pCurPin = pinList;
        // Loop through list trying to find our pin
        while (pCurPin->pin != pin_ && pCurPin->pNext != NULL) {
            pCurPin = pCurPin->pNext;
        }

        // If the pin is in the list
        if (pCurPin->pin == pin_) {
            pPinInfo = pCurPin;
            AnalogEncoder * pCurEnc = pPinInfo->encoderList;
            // Loop to end of encoder list
            while (pCurEnc->pNext != NULL) {
                pCurEnc = pCurEnc->pNext;
            }
            // Link the encoder info the end of the list
            pCurEnc->pNext = this;
            this->pPrev = pCurEnc;
            this->pNext = NULL;
            pPinInfo->numEncoders++;
        } else { // Need to add a new pin to the list
            this->pNext = NULL;
            this->pPrev = NULL;

            // Add encoder linked list to pinInfo
            pPinInfo = new PinInfo;
            pPinInfo->encoderList = this;
            pPinInfo->numEncoders = 1;
            pPinInfo->pin = pin_;
            pPinInfo->pNext = NULL;
            pPinInfo->state = LOW_STATE;

            // Link the end of the list to the new pin
            pPinInfo->pPrev = pCurPin;
            pCurPin->pNext = pPinInfo;
        }
    }

    ResetCounts();
    lowThreshold = 70 * 256;
    highThreshold = 210 * 256;
}

void AnalogEncoder::SetThresholds(float low, float high) {
    lowThreshold = low / 3.3 * 0x10000;
    highThreshold = high / 3.3 * 0x10000;
}

AnalogEncoder::~AnalogEncoder() {
    // Remove this encoder from the encoder list in the pin info
    AnalogEncoder *pPrevEnc = this->pPrev;
    AnalogEncoder *pNextEnc = this->pNext;

    if (pPrevEnc == NULL && pNextEnc == NULL) {
        //Then this was a lone encoder, must kill the pinInfo entry too
        PinInfo *pPrevPin = pPinInfo->pPrev;
        PinInfo *pNextPin = pPinInfo->pNext;
        if (pPrevPin == NULL && pNextPin == NULL) {
            // The pin list is now empty
            pinList = NULL;
        } else {
            if (pPrevPin != NULL) {
                pPrevPin->pNext = pNextPin;
            } else { // Reset the head
                pinList = pNextPin;
            }

            if (pNextPin != NULL) {
                pNextPin->pPrev = pPrevPin;
            }
        }
        // Delete the pinInfo
        delete pPinInfo;
    } else {
        // If we aren't at the start of the list
        if (pPrevEnc != NULL) {
            pPrevEnc->pNext = pNextEnc;
        } else { // Reset the head
            pPinInfo->encoderList = pNextEnc;
        }

        if (pNextEnc != NULL) {
            pNextEnc->pPrev = pPrevEnc;
        }
        pPinInfo->numEncoders--;
    }
}

void AnalogEncoder::ProcessInt() {

    PinInfo *pCurPin = pinList;
    while (pCurPin != NULL) {
        EncoderState oldState = pCurPin->state;
        // Process interrupt for the first encoder on each pin
        bool countReceived = pCurPin->encoderList->ProcessIntSelf();

        // If the state changed and there are more encoders tied to this pin
        if (countReceived && pCurPin->numEncoders > 1) {
            // Get the pointer to the next object
            AnalogEncoder *pCurEnc = pCurPin->encoderList->pNext;
            while (pCurEnc != NULL) {
                // Update other encoders
                pCurEnc->counts++;

                // Move to the next in the list
                pCurEnc = pCurEnc->pNext;
            }
        }
        pCurPin = pCurPin->pNext;
    }
}

bool AnalogEncoder::ProcessIntSelf() {
    int value = EncoderValue();
    EncoderState state = pPinInfo->state;
    if (state == LOW_STATE && value > highThreshold) {
        pPinInfo->state = HIGH_STATE;
        counts++;
        return true;
    }
    if (state == HIGH_STATE && value < lowThreshold) {
        pPinInfo->state = LOW_STATE;
        counts++;
        return true;
    }
    return false;
}

int AnalogEncoder::Counts() {
    return (int) counts;
}

void AnalogEncoder::ResetCounts() {
    counts = 0;
}


ButtonBoard::ButtonBoard(FEHIO::FEHIOPort bank) {
    switch (bank) {
        case FEHIO::Bank0: {
            _left.Initialize(FEHIO::P0_0);
            _middle.Initialize(FEHIO::P0_1);
            _right.Initialize(FEHIO::P0_2);
            break;
        }
        case FEHIO::Bank1: {
            _left.Initialize(FEHIO::P1_0);
            _middle.Initialize(FEHIO::P1_1);
            _right.Initialize(FEHIO::P1_2);
            break;
        }
        case FEHIO::Bank2: {
            _left.Initialize(FEHIO::P2_0);
            _middle.Initialize(FEHIO::P2_1);
            _right.Initialize(FEHIO::P2_2);
            break;
        }
        case FEHIO::Bank3: {
            _left.Initialize(FEHIO::P3_0);
            _middle.Initialize(FEHIO::P3_1);
            _right.Initialize(FEHIO::P3_2);
            break;
        }
    }
}

bool ButtonBoard::LeftPressed() {
    return (_left.Value() == 0);
}

bool ButtonBoard::LeftReleased() {
    return (_left.Value() == 1);
}

bool ButtonBoard::MiddlePressed() {
    return (_middle.Value() == 0);
}

bool ButtonBoard::MiddleReleased() {
    return (_middle.Value() == 1);
}

bool ButtonBoard::RightPressed() {
    return (_right.Value() == 0);
}

bool ButtonBoard::RightReleased() {
    return (_right.Value() == 1);
}
