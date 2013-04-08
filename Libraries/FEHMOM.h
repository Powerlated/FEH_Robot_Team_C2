#ifndef FEHMOM_H
#define FEHMOM_H

#include <FEHXBee.h>

class FEHMOM
{
public:
	FEHMOM();

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

	// Enable receiving of MOM data
	void Enable();

	// Disable receiving of MOM data
	void Disable();

	// return the current course number { 1, 2, 3 }
	unsigned char CurrentCourse();

	// returns the letter of the current region { A, B, C, D, E, F, G, H, I, J, K, L }
	char CurrentRegionLetter();

	// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
	int CurrentRegion();


	// Objective functions:
	// returns true if the RIGHT Stone should be moved
	bool Stone();

	// returns true if Generator switch should be moved BACKWARD
	bool Generator();

	// returns true if Top Satellite button has been pressed
	bool TopButton();

	// returns true if Bottom Satellite button has been pressed
	bool BottomButton();

	// returns the match time in seconds
	unsigned char Time();

	unsigned char WaitForPacket();

private:
	FEHXBee _xbee;

	int _region;
};

extern FEHMOM MOM;

#endif // FEHMOM_H
