#include "FEHUtility.h"
#include "lptmr.h"

void Sleep( int msec )
{
    time_delay_ms( (unsigned int)msec );
}

void Sleep( float sec )
{
    time_delay_ms( (unsigned int)( sec * 1000 ) );
}

void Sleep( double sec )
{
    time_delay_ms( (unsigned int)( sec * 1000 ) );
}
