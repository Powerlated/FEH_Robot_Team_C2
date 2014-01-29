#ifndef FEHSERVO_H
#define FEHSERVO_H

class FEHServo
{
public:
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

    FEHServo( FEHServoPort );
    void SetDegree( float );
    void Calibrate();
    void Off();
    void SetMax( int );
    void SetMin( int );
private:
    FEHServoPort servo_port;
    unsigned short servo_min;
    unsigned short servo_max;
	int _position;
};

#endif // FEHSERVO_H
