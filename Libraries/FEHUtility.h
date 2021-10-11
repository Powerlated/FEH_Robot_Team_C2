#ifndef FEHUTILITY_H
#define FEHUTILITY_H

/**
 * @brief Pauses the Proteus for an amount of time.
 * 
 * @param msec Time, in milliseconds, to pause
 */
void Sleep( int msec );

/**
 * @brief Pauses the Proteus for an amount of time.
 * 
 * @param sec Time, in seconds, to pause
 */
void Sleep( float sec );

/**
 * @brief Pauses the Proteus for an amount of time.
 * 
 * @param sec Time, in seconds, to pause
 */
void Sleep( double sec );

/**
 * @brief Returns the amount of time since the Proteus was turned on.
 * 
 * @return double Amount of time, in seconds, since the Proteus was turned on
 */
double TimeNow();

// TODO make private? (what is TimeNowSec?)
unsigned int TimeNowSec();
unsigned long TimeNowMSec();
void ResetTime();

#endif // FEHUTILITY_H
