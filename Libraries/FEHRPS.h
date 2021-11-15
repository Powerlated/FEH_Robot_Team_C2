#ifndef FEHRPS_H
#define FEHRPS_H

#include <FEHXBee.h>

/**
 * @brief Access to RPS functionality
 * 
 * Your Proteus will show a blue light indicator when it is in range of RPS.<br/>
 * You must first call InitializeTouchMenu() before RPS functionality will work.
 * 
 */
class FEHRPS
{
public:

	/**
	 * @brief Setup system used to select RPS course
	 * 
	 * You must be in range of the course to be able to successfully initialize.
	 * 
	 */
	void InitializeTouchMenu();

	/**
	 * @brief Get course number corresponding to current region
	 * 
	 * Get course number corresponding to current region
	 * 
	 * @return unsigned char 1 (regions A-D), 2 (regions E-H)
	 */
	unsigned char CurrentCourse();
	
	/**
	 * @brief Get current region's corresponding letter
	 * 
	 * Get current region's corresponding letter from 
	 * set { A, B, C, D, E, F, G, H, I, J, K, L }
	 * 
	 * @return char Region letter RPS was initialized to
	 */
	char CurrentRegionLetter();

	/**
	 * @brief Get time remaining for a match
	 * 
	 * Begins transmitting time remaining with 90 seconds left in the match.
	 * 
	 * @return int Number of seconds remaining
	 */
	int Time();
	
	/**
	 * @brief Get current X position of QR code on course
	 * 
	 * Get current X position of QR code on course
	 * 
	 * @return float X coordinate (inches)
	 */
	float X();

	/**
	 * @brief Get current Y position of QR code on course
	 * 
	 * Get current Y position of QR code on course
	 * 
	 * @return float Y coordinate (inches)
	 */
	float Y();
	
	/**
	 * @brief Get current heading angle of QR code
	 * 
	 * Get current heading angle of QR code
	 * 
	 * @return float Heading angle (degrees)
	 */
	float Heading();

	// Objective functions:
	/**
	 * @brief Get required ice cream flavor
	 * 
	 * Get required ice cream flavor
	 * 
	 * @return int 0 (left lever/vanilla), 1 (middle lever/twist), 2 (right lever/chocolate)
	 */
    int GetIceCream();

	//RPS debug/deprecated functions:
	unsigned char WaitForPacket();
	int WaitForPacketDebug(int *packetsFound, int *packetsLost,int *lastFoundPacketTime);
	// returns the number of the current course { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
	int CurrentRegion();
	// Creates a menu to allow you to pick the correct region
	// Assumes ButtonBoard is plugged into Bank3
	// Right button increments region
	// Left button decrements region
	// Middle button selects region
	void InitializeMenu(); //Deprecated because uses button board
	// Manually pick and configure a region
	// int region => { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }
	// char region => { a, b, c, d, e, f, g, h, i, j, k, l } || { A, B, C, D, E, F, G, H, I, J, K, L }
	void Initialize( int region );
	void Initialize( char region );

	FEHRPS();

private:
	FEHXBee _xbee;
	int _region;
};

/**
 * @brief Global access to FEHRPS class
 * 
 * Global access to FEHRPS class
 * 
 */
extern FEHRPS RPS;

#endif // FEHRPS_H