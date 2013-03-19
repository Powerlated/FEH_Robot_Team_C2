#ifndef FEHLCD_H
#define FEHLCD_H

class FEHLCD
{
public:

    typedef enum
    {
        Black = 0,
        White
    } FEHLCDColor;

    FEHLCD();

    void Clear( FEHLCDColor color );
    void SetFontColor( FEHLCDColor color );

    void Write( const char* str );

    void WriteLine( const char* str );

private:
    FEHLCDColor _forecolor;
    FEHLCDColor _backcolor;
};

extern FEHLCD LCD;

#endif // FEHLCD_H
