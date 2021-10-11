#ifndef FEHBATTERY_H
#define FEHBATTERY_H

#include "FEHIO.h"

/**
 * @brief Access to the Proteus battery
 *
 * Allows the user to check the Proteus' battery level
 */
class FEHBattery : public AnalogInputPin
{
public:
    FEHBattery(FEHIO::FEHIOPin);

    /**
     * @brief Find battery charge
     *
     * Max charge is 11.2-11.7 V
     *
     * @return float Voltage level
     */
    float Voltage();
};

extern FEHBattery Battery;
#endif // FEHBATTERY_H
