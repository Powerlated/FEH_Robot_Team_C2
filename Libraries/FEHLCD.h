#ifndef FEHLCD_H
#define FEHLCD_H

#include "derivative.h"
//#include <string>

//using namespace std;

class FEHLCD
{
public:

    typedef enum
    {
        Black = 0,
        White,
        Red,
        Green,
        Blue
    } FEHLCDColor;

    typedef enum
    {
        North = 0,
        South,
        East,
        West
    } FEHLCDOrientation;

    FEHLCD();

    void PrintImage(int x, int y);
    void Initialize();
    void SetOrientation(FEHLCDOrientation orientation);

    void Clear( FEHLCDColor color );
    void Clear(unsigned int color);
    void Clear();

    void SetFontColor( FEHLCDColor color );
    void SetFontColor( unsigned int color);
    void SetBackgroundColor( FEHLCDColor color );
    void SetBackgroundColor(unsigned int color);

    // Drawing Functions
    void DrawPixel(int x, int y);
    void DrawHorizontalLine(int y,int x1, int x2);
    void DrawVerticalLine(int x, int y1, int y2);
    void DrawLine(int x1, int y1, int x2, int y2);
    void DrawRectangle(int x, int y, int width, int height);
    void FillRectangle(int x, int y, int width, int height);
    void DrawCircle(int x0, int y0, int r);
    void FillCircle(int x0, int y0, int r);

    // Write information at a specific Pixel on the screen
    void WriteAt(const char * str, int x, int y);
    void WriteAt(int i, int x, int y);
    void WriteAt(float f, int x, int y);
    void WriteAt(double d, int x, int y);
    void WriteAt(bool b, int x, int y);

    // Write to the screen
    void Write( const char* str );
    void Write( int i );
    void Write( float f );
    void Write( double d );
    void Write( bool b );

    // Write to the screeen and advance to next line
    void WriteLine( const char* str );
    void WriteLine( int i );
    void WriteLine( float f );
    void WriteLine( double d );
    void WriteLine( bool b );

private:
    typedef struct regColVal {
        uint32_t BVal;
        uint32_t CVal;
        uint32_t DVal;
    } RegisterColorValues;


    void _Initialize();
    void _Clear();
    void _RepeatColor();
    void _BackPixel();
    void _ForePixel();
    void SetRegisterColorValues();

    void WriteChar(int row, int col, char c);
    void WriteCharAt(int x, int y, char c);

    void WriteIndex( unsigned char index );
    void WriteParameter( unsigned char param );
    void RepeatColor();

    unsigned int ConvertFEHColorTo24Bit(FEHLCDColor color);
    unsigned int Convert24BitColorTo16Bit(unsigned int color);
    unsigned int ConvertRGBColorTo16Bit(unsigned char r,unsigned char g,unsigned char b);

    void NextLine();
    void CheckLine();
    void NextChar();
    void SetDrawRegion(int x, int y, int width, int height);

    FEHLCDOrientation _orientation;

    int _maxlines;
    int _maxcols;
    int _width;
    int _height;
    int _currentline;
    int _currentchar;
    unsigned int _forecolor;
    unsigned int _backcolor;
    RegisterColorValues foreRegisterValues, backRegisterValues;

    static unsigned char fontData[];
};

extern FEHLCD LCD;

#endif // FEHLCD_H
