#ifndef FEHLCD_H
#define FEHLCD_H

#include "derivative.h"
#include <string>

using namespace std;

class FEHLCD
{
public:

    typedef enum
    {
        Black = 0,
        White
    } FEHLCDColor;



    FEHLCD();

    void Initialize();

    void Clear( FEHLCDColor color );

    void SetFontColor( FEHLCDColor color );
    void SetBackgroundColor( FEHLCDColor color );

    void Write( const char* str );
//    void Write( string str );
    void Write( int i );
    void Write( float f );
    void Write( double d );
    void Write( bool b );

    void WriteLine( const char* str );
//    void WriteLine( string str );
    void WriteLine( int i );
    void WriteLine( float f );
    void WriteLine( double d );
    void WriteLine( bool b );

private:
    FEHLCDColor _forecolor;
    FEHLCDColor _backcolor;

    typedef struct regColVal {
        uint32_t BVal;
        uint32_t CVal;
        uint32_t DVal;
    } RegisterColorValues;

    int _currentline;
    int _currentchar;

    void _Initialize();
    void _Clear();
    void _RepeatColor();
    void _BackPixel();
    void _ForePixel();

    void SetRegisterColorValues();
    RegisterColorValues foreRegisterValues, backRegisterValues;


    void WriteChar(int row, int col, char c);
    void WriteIndex( unsigned char index );
    void WriteParameter( unsigned char param );
    void RepeatColor();




    void NextLine();
    void CheckLine();
    void NextChar();

    static unsigned char fontData[];
};

extern FEHLCD LCD;

#endif // FEHLCD_H
