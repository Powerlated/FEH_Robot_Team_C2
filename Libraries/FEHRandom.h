#ifndef FEHRANDOM_H
#define FEHRANDOM_H

#include <stdlib.h>

class FEHRandom
{
public:
	void Initialize();
	void Seed();
	int RandInt();
private:
	bool seeded;
};

extern FEHRandom Random;

#endif // FEHRANDOM_H
