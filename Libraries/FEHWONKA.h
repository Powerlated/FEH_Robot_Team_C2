#ifndef FEHWONKA_H
#define FEHWONKA_H

#include <FEHXBee.h>
#include "FEHServo.h"

class FEHWONKA
{
public:

    FEHWONKA( FEHServo::FEHServoPort );

	// Creates a menu to allow you to pick the correct region
	// Assumes ButtonBoard is plugged into Bank3
	// Right button increments region
	// Left button decrements region
	// Middle button selects region
	void InitializeMenu();

	// Manually pick and configure a region
	// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
	// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
	void Initialize( int region );
	void Initialize( char region );

    // Enable receiving of WONKA data
	void Enable();

    // Disable receiving of WONKA data
	void Disable();

	// return the current course number { 1, 2, 3 }
	unsigned char CurrentCourse();

	// returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
	char CurrentRegionLetter();

	// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
	int CurrentRegion();


	// Objective functions:
    // returns the number of oven button presses
    int Oven();

    // returns the number of times the oven button has been pressed
    int OvenPressed();

    // returns true if the chute switch is activated
    bool Chute();

	// returns the match time in seconds
	unsigned char Time();

	unsigned char WaitForPacket();

	float X();
	float Y();
	float Heading();

private:
	FEHXBee _xbee;
	int _region;
	FEHServo _irbeacon;
};

extern FEHWONKA WONKA;

#endif // FEHWONKA_H
