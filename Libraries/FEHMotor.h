#ifndef FEHMOTOR_H
#define FEHMOTOR_H

#include "derivative.h"

/**
 * @brief Access to the Proteus motor ports
 * 
 */
class FEHMotor
{
public:
    /**
     * @brief Motor port values to be used when declaring an FEHMotor
     * 
     * Motor port values to be used when declaring an FEHMotor
     */
    typedef enum
    {
        Motor0 = 0,
        Motor1,
        Motor2,
        Motor3
    } FEHMotorPort;
    
    /**
     * @brief Declare a new FEHMotor object
     * 
     * Reccommended max voltages for provided DC motors:<br/>
     * - Acroname = 12.0<br/>
     * - Hacked Futaba = 5.0<br/>
     * - Hacked FITEC = 5.0<br/>
     * - Igwan = 9.0<br/>
     * - Vex Motor = 7.2<br/>
     * - GMH-34 = 7.2
     * 
     * @param motorport Motor port number used to power motor
     * @param max_voltage Maximum voltage allows to power motor
     */
    FEHMotor( FEHMotorPort motorport, float max_voltage );

    /**
     * @brief Stop powering a motor
     * 
     * Stop powering a motor.<br/>
     * Note: This is the same as using SetPercent(0), so it will not apply physical brake force to the motor. Momentum may still affect your movement.
     * 
     */
    void Stop();
    /**
     * @brief Set motor percent for your motor
     * 
     * Motor percent must be greater than -100 and less than 100.
     * 
     * @param percent Percent speed to power motor at
     */
	void SetPercent( float percent );

private:
    void SetPower( int power );
    float _max_percent;
    FEHMotorPort _motorport;
	char _power;
};

#endif // FEHMOTOR_H
