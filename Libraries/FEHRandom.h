#ifndef FEHRANDOM_H
#define FEHRANDOM_H

#include <stdlib.h>

/**
 * @brief Access to the Proteus random number generator.
 * 
 * Provides the ability to generate a random number, without having to worry about providing a seed.  This is done by using the random number generator in the Proteus to generate a seed, which is used to produce random numbers.
 * 
 */
class FEHRandom
{
public:
	// TODO make private
	void Initialize();
	void Seed();
	/**
	 * @brief Returns a random integer between 0 and 32767.
	 * 
	 * No seed function is necessary.
	 * 
	 * @return int An integer between 0 and 32767 (inclusive).
	 */
	int RandInt();
private:
	bool seeded;
};

/**
 * @brief Global access to the FEHRandom class.
 */
extern FEHRandom Random;

#endif // FEHRANDOM_H
