#include "FEHLCD.h"

FEHLCD LCD;

FEHLCD::FEHLCD()
{
}

void FEHLCD::Clear( FEHLCDColor color )
{
    _backcolor = color;

    // TODO: Clear screen
}

void FEHLCD::SetFontColor( FEHLCDColor color )
{
    _forecolor = color;
}

void FEHLCD::Write( const char *str )
{

}

void FEHLCD::WriteLine( const char *str )
{

}
