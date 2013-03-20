#include "FEHLCD.h"
#include "FEHUtility.h"

#include "derivative.h"
#include "mcg.h"
#include "lptmr.h"
#include "stdio.h"

#define CLR_CS GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 3 ) )
#define SET_CS GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 3 ) )

#define CLR_RS GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 18 ) )
#define SET_RS GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 18 ) )

#define CLR_Rd GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 16 ) )
#define SET_RD GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 16 ) )

#define CLR_WR GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 17 ) )
#define SET_WR GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 17 ) )

#define CLR_RESET GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 3 ) )
#define SET_RESET GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 3 ) )

// font height = 20, maxlines = 240 / 20
// TODO: support portrait text
#define MAXLINES 14
#define MAXCHARS 26

#define CharHeight 17
#define CharWidth  12


unsigned char FEHLCD::fontData[] = {
    0x00, 0x00, 0x00, 0x00, 0x00,// (space)
    0x00, 0x00, 0x5F, 0x00, 0x00,// !
    0x00, 0x07, 0x00, 0x07, 0x00,// "
    0x14, 0x7F, 0x14, 0x7F, 0x14,// #
    0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
    0x23, 0x13, 0x08, 0x64, 0x62,// %
    0x36, 0x49, 0x55, 0x22, 0x50,// &
    0x00, 0x05, 0x03, 0x00, 0x00,// '
    0x00, 0x1C, 0x22, 0x41, 0x00,// (
    0x00, 0x41, 0x22, 0x1C, 0x00,// )
    0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
    0x08, 0x08, 0x3E, 0x08, 0x08,// +
    0x00, 0x50, 0x30, 0x00, 0x00,// ,
    0x08, 0x08, 0x08, 0x08, 0x08,// -
    0x00, 0x60, 0x60, 0x00, 0x00,// .
    0x20, 0x10, 0x08, 0x04, 0x02,// /
    0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
    0x00, 0x42, 0x7F, 0x40, 0x00,// 1
    0x42, 0x61, 0x51, 0x49, 0x46,// 2
    0x21, 0x41, 0x45, 0x4B, 0x31,// 3
    0x18, 0x14, 0x12, 0x7F, 0x10,// 4
    0x27, 0x45, 0x45, 0x45, 0x39,// 5
    0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
    0x01, 0x71, 0x09, 0x05, 0x03,// 7
    0x36, 0x49, 0x49, 0x49, 0x36,// 8
    0x06, 0x49, 0x49, 0x29, 0x1E,// 9
    0x00, 0x36, 0x36, 0x00, 0x00,// :
    0x00, 0x56, 0x36, 0x00, 0x00,// ;
    0x00, 0x08, 0x14, 0x22, 0x41,// <
    0x14, 0x14, 0x14, 0x14, 0x14,// =
    0x41, 0x22, 0x14, 0x08, 0x00,// >
    0x02, 0x01, 0x51, 0x09, 0x06,// ?
    0x32, 0x49, 0x79, 0x41, 0x3E,// @
    0x7E, 0x11, 0x11, 0x11, 0x7E,// A
    0x7F, 0x49, 0x49, 0x49, 0x36,// B
    0x3E, 0x41, 0x41, 0x41, 0x22,// C
    0x7F, 0x41, 0x41, 0x22, 0x1C,// D
    0x7F, 0x49, 0x49, 0x49, 0x41,// E
    0x7F, 0x09, 0x09, 0x01, 0x01,// F
    0x3E, 0x41, 0x41, 0x51, 0x32,// G
    0x7F, 0x08, 0x08, 0x08, 0x7F,// H
    0x00, 0x41, 0x7F, 0x41, 0x00,// I
    0x20, 0x40, 0x41, 0x3F, 0x01,// J
    0x7F, 0x08, 0x14, 0x22, 0x41,// K
    0x7F, 0x40, 0x40, 0x40, 0x40,// L
    0x7F, 0x02, 0x04, 0x02, 0x7F,// M
    0x7F, 0x04, 0x08, 0x10, 0x7F,// N
    0x3E, 0x41, 0x41, 0x41, 0x3E,// O
    0x7F, 0x09, 0x09, 0x09, 0x06,// P
    0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
    0x7F, 0x09, 0x19, 0x29, 0x46,// R
    0x46, 0x49, 0x49, 0x49, 0x31,// S
    0x01, 0x01, 0x7F, 0x01, 0x01,// T
    0x3F, 0x40, 0x40, 0x40, 0x3F,// U
    0x1F, 0x20, 0x40, 0x20, 0x1F,// V
    0x7F, 0x20, 0x18, 0x20, 0x7F,// W
    0x63, 0x14, 0x08, 0x14, 0x63,// X
    0x03, 0x04, 0x78, 0x04, 0x03,// Y
    0x61, 0x51, 0x49, 0x45, 0x43,// Z
    0x00, 0x00, 0x7F, 0x41, 0x41,// [
    0x02, 0x04, 0x08, 0x10, 0x20,// "\"
    0x41, 0x41, 0x7F, 0x00, 0x00,// ]
    0x04, 0x02, 0x01, 0x02, 0x04,// ^
    0x40, 0x40, 0x40, 0x40, 0x40,// _
    0x00, 0x01, 0x02, 0x04, 0x00,// `
    0x20, 0x54, 0x54, 0x54, 0x78,// a
    0x7F, 0x48, 0x44, 0x44, 0x38,// b
    0x38, 0x44, 0x44, 0x44, 0x20,// c
    0x38, 0x44, 0x44, 0x48, 0x7F,// d
    0x38, 0x54, 0x54, 0x54, 0x18,// e
    0x08, 0x7E, 0x09, 0x01, 0x02,// f
    0x08, 0x14, 0x54, 0x54, 0x3C,// g
    0x7F, 0x08, 0x04, 0x04, 0x78,// h
    0x00, 0x44, 0x7D, 0x40, 0x00,// i
    0x20, 0x40, 0x44, 0x3D, 0x00,// j
    0x00, 0x7F, 0x10, 0x28, 0x44,// k
    0x00, 0x41, 0x7F, 0x40, 0x00,// l
    0x7C, 0x04, 0x18, 0x04, 0x78,// m
    0x7C, 0x08, 0x04, 0x04, 0x78,// n
    0x38, 0x44, 0x44, 0x44, 0x38,// o
    0x7C, 0x14, 0x14, 0x14, 0x08,// p
    0x08, 0x14, 0x14, 0x18, 0x7C,// q
    0x7C, 0x08, 0x04, 0x04, 0x08,// r
    0x48, 0x54, 0x54, 0x54, 0x20,// s
    0x04, 0x3F, 0x44, 0x40, 0x20,// t
    0x3C, 0x40, 0x40, 0x20, 0x7C,// u
    0x1C, 0x20, 0x40, 0x20, 0x1C,// v
    0x3C, 0x40, 0x30, 0x40, 0x3C,// w
    0x44, 0x28, 0x10, 0x28, 0x44,// x
    0x0C, 0x50, 0x50, 0x50, 0x3C,// y
    0x44, 0x64, 0x54, 0x4C, 0x44,// z
    0x00, 0x08, 0x36, 0x41, 0x00,// {
    0x00, 0x00, 0x7F, 0x00, 0x00,// |
    0x00, 0x41, 0x36, 0x08, 0x00,// }
    0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
    0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};

FEHLCD LCD;

bool initialized = false;

FEHLCD::FEHLCD()
{
    _forecolor = Black;
    _backcolor = White;

    _currentline = 0;
    _currentchar = 0;
    SetRegisterColorValues();
}

void FEHLCD::Initialize()
{
    if( initialized == false )
    {
        initialized = true;

        _Initialize();
    }
}

void FEHLCD::Clear( FEHLCDColor color )
{
    _backcolor = color;
    SetRegisterColorValues();

    // TODO: Clear screen
    _Clear();
}

void FEHLCD::SetFontColor( FEHLCDColor color )
{
    _forecolor = color;
    SetRegisterColorValues();
}

void FEHLCD::Write( const char *str )
{
    int i=0;
    CheckLine();
    while(str[i] != '\0') {
        WriteChar(_currentline,_currentchar,str[i]);
        NextChar();
        i++;
    }
}

void FEHLCD::Write( int i )
{
    char num[50];
    sprintf(num,"%d",i);
    Write(num);
}

void FEHLCD::Write( float f )
{
    char num[50];
    int d,r;
    d = (int) f;
    r = (int) ((f-d)*1000);
    sprintf(num,"%d.%d",d,r);
    Write(num);
}

void FEHLCD::Write( double d )
{
    Write((float) d);
}

void FEHLCD::Write( bool b )
{
    if( b )
    {
        Write( "true" );
    }
    else
    {
        Write( "false" );
    }
}

void FEHLCD::WriteLine( const char* str )
{
    CheckLine();
    Write( str );
    NextLine();
}


void FEHLCD::WriteLine( int i )
{
    CheckLine();
    Write( i );
    NextLine();
}

void FEHLCD::WriteLine( float f )
{
    CheckLine();
    Write( f );
    NextLine();
}

void FEHLCD::WriteLine( double d )
{
    CheckLine();
    Write( d );
    NextLine();
}

void FEHLCD::WriteLine( bool b )
{
    CheckLine();
    Write( b );
    NextLine();
}

void FEHLCD::_Initialize()
{
    // Setup pins
    // CS - C3
    // RS - B18
    // RD - B16
    // WR - B17
    // RESET - B3
    PORTC_PCR3 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR18 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR16 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR17 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR3 = ( PORT_PCR_MUX( 1 ) );
    GPIOC_PDDR |= ( 1 << 3 );
    GPIOB_PDDR |= ( ( 1 << 18 ) );
    GPIOB_PDDR |= ( ( 1 << 16 ) );
    GPIOB_PDDR |= ( ( 1 << 17 ) );
    SET_RESET;
    GPIOB_PDDR |= ( ( 1 << 3 ) );

    // D0 - B9
    // D1 - B8
    // D2 - C4
    // D3 - C5
    // D4 - C6
    // D5 - C7
    // D6 - C12
    // D7 - C13
    // D8 - C14
    // D9 - C15
    // D10 - C16
    // D11 - C17
    // D12 - C18
    // D13 - C19
    // D14 - D0
    // D15 - D2
    // D16 - D3
    // D17 - D4
    PORTB_PCR9 = ( PORT_PCR_MUX( 1 ) );
    PORTB_PCR8 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR4 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR5 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR6 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR7 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR12 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR13 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR14 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR15 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR16 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR17 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR18 = ( PORT_PCR_MUX( 1 ) );
    PORTC_PCR19 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR0 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR2 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR3 = ( PORT_PCR_MUX( 1 ) );
    PORTD_PCR4 = ( PORT_PCR_MUX( 1 ) );
    GPIOB_PDDR |= ( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDDR |= ( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDDR |= ( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );

    // CS = 1;
    SET_CS;

    // RESET = 1;
    SET_RESET;

    Sleep( 500 );

    // uchar i;

    // widx(0x36); //set adress mode
    WriteIndex( 0x36 );

    // wpat( 0x00 ); //set page,column,line,RGB adress order;display data latch data,fli horizontal,vertical
    WriteParameter( 0x00 );

    // widx( 0xf0 ); //set pixel data interface
    WriteIndex( 0xF0 );

    // wpat( 0x04 ); //select 18bit
    WriteParameter( 0x04 );

    // widx( 0x29 );  //display on
    WriteIndex( 0x29 );
}

void FEHLCD::WriteIndex( unsigned char index )
{
    // CS = 0;
    CLR_CS;

    // RS = 0;
    CLR_RS;

    // RD = 1;
    SET_RD;

    // dataport0 = index;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( index & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // WR = 0;
    CLR_WR;

    // // delay2( 1 )

    // WR = 1;
    SET_WR;

    // CS = 1;
    SET_CS;

    // // dataport0 = 0xff;
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // RS = 1;
    SET_RS;

    // delay2( 1 );
}

void FEHLCD::WriteParameter( unsigned char param )
{
    // CS = 0;
    CLR_CS;

    // RD = 1;
    SET_RD;

    // RS = 1;
    SET_RS;

    // dataport0 = data1;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) );

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( ( ( param & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // WR = 0;
    CLR_WR;

    // // delay2( 1 );

    // WR = 1;
    SET_WR;

    // CS = 1;
    SET_CS;

    // //dataport0 = 0xff;

    // RS = 0;
    CLR_RS;

    // delay2( 1 );
}

void FEHLCD::_Clear()
{
    _currentline = 0;
    _currentchar = 0;

    unsigned int color = 0x00000;

    switch( _backcolor )
    {
        case Black: color = 0x00000; break;
        case White: color = 0x3FFFF; break;
    }


    unsigned char c1 = (unsigned char)( color & 0xFF );
    unsigned char c2 = (unsigned char)( ( color >> 8 ) & 0xFF );
    unsigned char c3 = (unsigned char)( ( color >> 16 ) & 0x03 );

    // widx(0x2a); //set column address
    WriteIndex( 0x2A );

    // wpat(0x00); //start column number high byte
    WriteParameter( 0x00 );

    //wpat(0x00); //start column number low byte
    WriteParameter( 0x00 );

    //wpat(0x01); //end column number high byte
    WriteParameter( 0x01 );

    //wpat(0x3f); //end column number low byte
    WriteParameter( 0x3F );

    //widx(0x2b); //set row adress
    WriteIndex( 0x2b );

    //wpat(0x00); //start row number high byte
    WriteParameter( 0x00 );

    //wpat(0x00); //start row number low byte
    WriteParameter( 0x00 );

    //wpat(0x00); //end row number high byte
    WriteParameter( 0x00 );

    //wpat(0xef); //end row number low byte
    WriteParameter( 0xEF );

    //widx(0x2c);       //enter write mode
    WriteIndex( 0x2C );

    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );


    GPIOB_PDOR |= backRegisterValues.BVal;
    GPIOC_PDOR |= backRegisterValues.CVal;
    GPIOD_PDOR |= backRegisterValues.DVal;


    // CS = 0;
    CLR_CS;

    // RS = 1;
    SET_RS;

    for( unsigned int i = 0; i < 240; i++ )
    {
        for( unsigned int j = 0; j < 320; j++ )
        {
            // clr_cs, set_rs moved outside of loop

            //set color
            // moved outside of loop

            // WR = 0;
            CLR_WR;

            // WR = 1;
            SET_WR;

            // set_cs, clr_rs moved after loop
        }
    }

    // CS = 1;
    SET_CS;

    // RS = 0;
    CLR_RS;
}

void FEHLCD::NextLine() {
    _currentline ++;
    _currentchar = 0;
}

void FEHLCD::CheckLine()
{
    if( _currentline >= MAXLINES)
    {
        _currentline = 0;
        _Clear();
    }
}

void FEHLCD::NextChar() {
    _currentchar++;
    if(_currentchar == MAXCHARS) {
        NextLine();
        CheckLine();
    }
}

void FEHLCD::WriteChar(int charRow, int charCol, char charNum) {
    if(charNum > 125 || charNum <32)
    {
        charNum = 32;
    }
    charNum-=32;

   unsigned int pixStartRow = 3+charRow*CharHeight;
   unsigned int pixEndRow = pixStartRow+CharHeight-1;

   unsigned int pixStartCol = 2+charCol*CharWidth;
   unsigned int pixEndCol   = pixStartCol+CharWidth-1;

    // widx( 0x2a ); //set column adress
    WriteIndex( 0x2A );

    // wpat( 0x00 ); //start column number high byte
    WriteParameter((pixStartCol & 0xf00) >> 8 );

    // wpat( 0x00 ); //start column number low byte
    WriteParameter( pixStartCol & 0xff);

    // wpat( 0x01 ); //end column number high byte
    WriteParameter((pixEndCol & 0x0f00) >> 8 );

    // wpat( 0x3f ); //end column number low byte
    WriteParameter( pixEndCol & 0xff);


    // widx( 0x2b ); //set row address
    WriteIndex( 0x2B );

    // wpat( 0x00 ); //start row number high byte
    WriteParameter((pixStartRow & 0x0f00) >> 8 );

    // wpat( 0x00 ); //start row number low byte
    WriteParameter( pixStartRow & 0xff);

    // wpat( 0x00 ); //end row number high byte
    WriteParameter( (pixEndRow & 0x0f00) >> 8 );

    // wpat( 0xef ); //end row number low byte
    WriteParameter( pixEndRow & 0xff);

    // widx( 0x2c );       //enter write mode
    WriteIndex( 0x2C );

    unsigned char charData[5];
    charData[0] = fontData[5*charNum];
    charData[1] = fontData[5*charNum+1];
    charData[2] = fontData[5*charNum+2];
    charData[3] = fontData[5*charNum+3];
    charData[4] = fontData[5*charNum+4];


    bool pix;
    bool prevPix;

    for (int row = 0 ; row <7 ; row++)
    {
        _BackPixel();
        RepeatColor();
        prevPix=false;

        for(int col = 0; col < 5; col++)
        {
            bool pix = charData[col] & 0x01;
            if (pix)
            {
                if(!prevPix)
                    _ForePixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            else
            {
                if(prevPix)
                    _BackPixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            prevPix = pix;
            //charData[col] >>= 1;
            //WriteData1( n );
        }
        if(prevPix)
            _BackPixel();
        else
            RepeatColor();

        RepeatColor();
        prevPix = false;
        for(int col = 0; col < 5; col++)
        {
            bool pix = charData[col] & 0x01;
            if (pix)
            {
                if(!prevPix)
                    _ForePixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            else
            {
                if(prevPix)
                    _BackPixel();
                else
                    RepeatColor();
                RepeatColor();
            }
            prevPix = pix;
            charData[col] >>= 1;
            //WriteData1( n );
        }
    }


    _BackPixel();
    prevPix = false;
    for(int i=1;i<CharWidth*3; i++)
    {
        RepeatColor();

    }
}

void FEHLCD::RepeatColor()
{
    // RD=1;
    SET_RD;

    // CS=0;
    CLR_CS;

    // RS=1;
    SET_RS;

    // WR=0;
    CLR_WR;

    // WR=1;
    SET_WR;

    // CS=1;
    SET_CS;

    // RS=0;
    CLR_RS;
}

void FEHLCD::_ForePixel(){
    unsigned char c;

    // RD=1;
    SET_RD;

    // CS=0;
    CLR_CS;

    // RS=1;
    SET_RS;

    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );

    GPIOB_PDOR |= foreRegisterValues.BVal;
    GPIOC_PDOR |= foreRegisterValues.CVal;
    GPIOD_PDOR |= foreRegisterValues.DVal;

    // WR=0;
    CLR_WR;

    // WR=1;
    SET_WR;

    // CS=1;
    SET_CS;

    // RS=0;
    CLR_RS;
}

void FEHLCD::_BackPixel() {

    // RD=1;
    SET_RD;

    // CS=0;
    CLR_CS;

    // RS=1;
    SET_RS;

    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 9 ) | ( 1 << 8 ) );
    GPIOC_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 4 ) | ( 1 << 5 ) | ( 1 << 6 ) | ( 1 << 7 ) | ( 1 << 12 ) | ( 1 << 13 ) | ( 1 << 14 ) | ( 1 << 15 ) | ( 1 << 16 ) | ( 1 << 17 ) | ( 1 << 18 ) | ( 1 << 19 ) );
    GPIOD_PDOR &= ~GPIO_PDOR_PDO( ( 1 << 0 ) | ( 1 << 2 ) | ( 1 << 3 ) | ( 1 << 4 ) );


    GPIOB_PDOR |= backRegisterValues.BVal;
    GPIOC_PDOR |= backRegisterValues.CVal;
    GPIOD_PDOR |= backRegisterValues.DVal;


    /*GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 9 ) );
    GPIOB_PDOR |= GPIO_PDOR_PDO( ( 1 << 8 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 4 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 5 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 6 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 7 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 12 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 13 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 14 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 15 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 16 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 17 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 18 ) );
    GPIOC_PDOR |= GPIO_PDOR_PDO( ( 1 << 19 ) );
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( 1 << 0 ) );
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( 1 << 2 ) );
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( 1 << 3 ) );
    GPIOD_PDOR |= GPIO_PDOR_PDO( ( 1 << 4 ) );*/


    // WR=0;
    CLR_WR;

    // WR=1;
    SET_WR;

    // CS=1;
    SET_CS;

    // RS=0;
    CLR_RS;
}

void FEHLCD::SetRegisterColorValues()
{
    unsigned int color = 0x00000;

    switch( _backcolor )
    {
        case Black: color = 0x00000; break;
        case White: color = 0x3FFFF; break;
    }

    unsigned char c1 = (unsigned char)( color & 0xFF );
    unsigned char c2 = (unsigned char)( ( color >> 8 ) & 0xFF );
    unsigned char c3 = (unsigned char)( ( color >> 16 ) & 0x03 );




    foreRegisterValues.BVal = 0;
    foreRegisterValues.CVal = 0;
    foreRegisterValues.DVal = 0;

    backRegisterValues.BVal = 0;
    backRegisterValues.CVal = 0;
    backRegisterValues.DVal = 0;


    // dataport0 = sel_color[ i ];

    backRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    backRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // dataport2 = sel_color[ i + 1 ];
//    c = sel_color[ i + 1 ];
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 14 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 15 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 16 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 17 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 18 ) );
    backRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 19 ) );
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 0 ) );
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 2 ) );

    // dataport3 = sel_color[ i + 2 ];
//    c = sel_color[ i + 2 ];
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 3 ) );
    backRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );


    switch( _forecolor )
    {
        case Black: color = 0x00000; break;
        case White: color = 0x3FFFF; break;
    }

    c1 = (unsigned char)( color & 0xFF );
    c2 = (unsigned char)( ( color >> 8 ) & 0xFF );
    c3 = (unsigned char)( ( color >> 16 ) & 0x03 );


    foreRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 9 ) );
    foreRegisterValues.BVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 8 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 5 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 6 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 7 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 12 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c1 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 13 ) );

    // dataport2 = sel_color[ i + 1 ];
//    c = sel_color[ i + 1 ];
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 14 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 15 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 2 ) ) ? ( 1 ) : ( 0 ) ) << 16 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 3 ) ) ? ( 1 ) : ( 0 ) ) << 17 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 4 ) ) ? ( 1 ) : ( 0 ) ) << 18 ) );
    foreRegisterValues.CVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 5 ) ) ? ( 1 ) : ( 0 ) ) << 19 ) );
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 6 ) ) ? ( 1 ) : ( 0 ) ) << 0 ) );
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c2 & ( 1 << 7 ) ) ? ( 1 ) : ( 0 ) ) << 2 ) );

    // dataport3 = sel_color[ i + 2 ];
//    c = sel_color[ i + 2 ];
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 0 ) ) ? ( 1 ) : ( 0 ) ) << 3 ) );
    foreRegisterValues.DVal |= GPIO_PDOR_PDO( ( ( ( c3 & ( 1 << 1 ) ) ? ( 1 ) : ( 0 ) ) << 4 ) );
}

