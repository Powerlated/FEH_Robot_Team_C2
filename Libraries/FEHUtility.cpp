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

double TimeNow()
{
    double t = (unsigned int) RTC_TSR;
    t+=((unsigned int) RTC_TPR) / ((double) 0x8000u);
    return t;
}

unsigned int TimeNows() {
    return (unsigned int) RTC_TSR;
}

unsigned long TimeNowms() {
    unsigned long m;
    m = ((unsigned int) RTC_TSR)*1000;
    m+= (((unsigned int) RTC_TPR)*1000) >> 15;
    return m;
}
/*void ResetTime()
{
    RTC_SR &=  ~RTC_SR_TCE_MASK;
    RTC_TSR = 0x0u;
    RTC_TPR = 0x0u;
    RTC_SR =  RTC_SR_TCE_MASK;
}*/
