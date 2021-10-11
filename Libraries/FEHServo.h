#ifndef FEHSERVO_H
#define FEHSERVO_H

/**
 * @brief Access to the Proteus servo ports
 * 
 * Before using a new servo, you must do the following steps:<br/>
 * 1. Declare a new FEHServo object<br/>
 * 2. Call TouchCalibrate() to determine minimum and maximum values<br/>
 * 3. At the beginning of your code, call SetMin() and SetMax() with their values determined above<br/>
 * After getting the servo's minimum and maximum values, you no longer need the TouchCalibrate() function call.<br/>
 * Each individual servo will need different minimmum and maximum values, so follow this process with each new servo.<br/>
 * <br/>
 * If you have decided to hack your servo motor, use the FEHMotor class instead of FEHServo.
 * 
 */
class FEHServo
{
public:
     /**
     * @brief Servo port values to be used when declaring an FEHServo
     * 
     * Servo port values to be used when declaring an FEHServo
     */
    typedef enum
    {
        Servo0 = 0,
        Servo1,
        Servo2,
        Servo3,
        Servo4,
        Servo5,
        Servo6,
        Servo7
    } FEHServoPort;

    /**
     * @brief Declare a new FEHServo object
     * 
     * Declare a new FEHServo object
     * 
     * @param servoport Servo port number used to power servo
     */
    FEHServo( FEHServoPort servoport);

    /**
     * @brief Turn servo motor to specific angle  
     * 
     * Servo angle must be greater than 0 and less than 180.
     * 
     * @param percent Angle to turn servo to
     */
    void SetDegree( float degree);

    /**
     * @brief Determine minimum and maximum values for your servo motor
     * 
     * Only needs to be used during first time setup of motor to get values. Must call SetMin() and SetMax() with these values for the servo to function properly.
     * 
     */
    void TouchCalibrate();

    /**
     * @brief Turn off a servo
     * 
     * Cuts off current supply to your servo. This can help extend the Proteus's battery life.<br/>
     * Note: your servo arm may drop very quickly if you turn the servo off so plan accordingly
     * 
     */
    void Off();

    /**
     * @brief Set the maximum value for the servo motor
     * 
     * This should be called at the start of your program before making any other calls to the servo.<br/>
     * This max value is determined by using the TouchCalibrate() function.
     * 
     * @param max Maximum compensation value
     */
    void SetMax( int max);

    /**
     * @brief Set the minimum value for the servo motor
     * 
     * This should be called at the start of your program before making any other calls to the servo.<br/>
     * This min value is determined by using the TouchCalibrate() function.
     * 
     * @param min Minimum compensation value
     */
    void SetMin( int min);

    //Debug functions:
    void Calibrate(); //Deprecated because uses button board
    void DigitalOn();
    void DigitalOff();
    
private:
    FEHServoPort servo_port;
    unsigned short servo_min;
    unsigned short servo_max;
	float _position;
};

#endif // FEHSERVO_H
