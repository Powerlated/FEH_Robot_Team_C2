#include "FEHRandom.h"
#include "MK60DZ10.h"
#include "FEHLCD.h"

FEHRandom Random;

void FEHRandom::Initialize() {
	SIM_SCGC3 |= SIM_SCGC3_RNGB_MASK; // Give the clock to the random module
	RNG_CMD |= (RNG_CMD_GS_MASK | RNG_CMD_CI_MASK | RNG_CMD_CE_MASK); // Generate new seed, clear interrupt, clear error
}

void FEHRandom::Seed() {
	while (RNG_SR & RNG_SR_BUSY_MASK) {} // While the seed is being generated
	while ((RNG_SR & RNG_SR_FIFO_LVL_MASK) == 0) {} // Until there's something there
	long seed = RNG_OUT_RANDOUT(RNG_OUT); // Get a random seed
	srand(seed); // Use srand
	seeded = true;
}

int FEHRandom::RandInt() {
	if (!seeded) {
		Seed();
	}
	
	return rand(); // steal rand from stdlib.h
}