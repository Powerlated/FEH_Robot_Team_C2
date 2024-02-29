#pragma once

#include <cstdint>
#include <LCDColors.h>

namespace FastLCD
{
    // Create color states
    typedef enum
    {
        Black = 0,
        White,
        Red,
        Green,
        Blue,
		Scarlet,
		Gray
    } FEHLCDColor;

    // Create directional states
    typedef enum
    {
        North = 0,
        South,
        East,
        West
    } FEHLCDOrientation;

    void DrawScreen();

    /**
     * @brief Clears LCD screen and sets background color
     * 
     * @param color
     *      Color that the LCD background is set to after being cleared
     * 
     * Note: This comment applies to all public member variations of
     * {@code} Clear()
     * 
     * Accesses private member {@code} _Clear() to clear screen.
     * If an argument is provided, the function also clears the 
     * screen either by converting 24 bit enum type color
     * to 16 bits or by converting an unsigned int to 16 bits,
     * then accessing {@code} _Clear()
     */
    void Clear();

    void SetFontPaletteIndex(uint8_t index);
    void SetBackgroundPaletteIndex(uint8_t index);
    void SetPaletteColor(uint8_t index, FEHLCDColor color);
    void SetPaletteColor(uint8_t index, unsigned int color);

    /**
     * @brief Draws patterns to LCD screen
     * 
     * Note: This comment applies to all public member variations of
     * {@code} DrawXXX() and FillXXX()
     * 
     * Draws or fills pixels on the LCD screen based on given
     * coordinates and dimensions.
     */
    void DrawPixel(int x, int y);
    void DrawHorizontalLine(int y,int x1, int x2);
    void DrawVerticalLine(int x, int y1, int y2);
    void DrawLine(int x1, int y1, int x2, int y2);
    void DrawThickLine(int x1, int y1, int x2, int y2);
    void DrawRectangle(int x, int y, int width, int height);
    void FillRectangle(int x, int y, int width, int height);
    void DrawCircle(int x0, int y0, int r);
    void FillCircle(int x0, int y0, int r);

    /**
     * @brief Writes message to LCD at specific coordinates
     * 
     * Note: This comment applies to all public member variations of
     * {@code} WriteAt()
     * 
     * @param 1
     *      message to be printed to the screen
     * 
     * @param x
     *      X-coordinate where a message will be printed
     * 
     * @param y
     *      Y-coordinate where a message will be printed
     */
	void WriteAt( const char * str, int x, int y );
	void WriteAt( int i, int x, int y );
	void WriteAt( float f, int x, int y );
	void WriteAt( double d, int x, int y );
	void WriteAt( bool b, int x, int y );
	void WriteAt( char c, int x, int y );

	// Write to Row, Column

    /**
     * @brief Writes message to LCD at a specific row and column
     * 
     * Note: This comment applies to all public member variations of
     * {@code} WriteRC()
     * 
     * @param 1
     *      message to be printed to the screen
     * 
     * @param row
     *      Row where a message will be printed
     * 
     * @param column
     *      Column where a message will be printed
     */
	void WriteRC( const char * str, int row, int col );
	void WriteRC( int i, int row, int col );
	void WriteRC( float f, int row, int col );
	void WriteRC( double d, int row, int col );
	void WriteRC( bool b, int row, int col );
	void WriteRC( char c, int row, int col );

    /**
     * @brief Writes message to LCD screen
     * 
     * Note: This comment applies to all public member variations of
     * {@code} Write()
     * 
     * @param 1
     *      message to be printed to the screen
     */
    void Write( const char* str );
    void Write( int i );
    void Write( float f );
    void Write( double d );
    void Write( bool b );
	void Write( char c );

    /**
     * @brief Writes message to LCD and returns to a new line
     * 
     * Note: This comment applies to all public member variations of
     * {@code} WriteLine()
     * 
     * @param 1
     *      message to be printed to the screen before a new line
     */
    void WriteLine( const char* str );
    void WriteLine( int i );
    void WriteLine( float f );
    void WriteLine( double d );
    void WriteLine( bool b );
	void WriteLine( char c );

    typedef struct regColVal {
        uint32_t BVal;
        uint32_t CVal;
        uint32_t DVal;
    } RegisterColorValues;
	
	int abs(int);

    static void _Clear();
    static void _RepeatColor();
    static void _BackPixel();
    static void _ForePixel();
    static void SetRegisterColorValues();

    static void WriteChar(int row, int col, char c);
    static void WriteCharAt(int x, int y, char c);

    static void WriteIndex( unsigned char index );
    static void WriteParameter( unsigned char param );

    static unsigned int ConvertFEHColorTo24Bit(FEHLCDColor color);
    static unsigned int Convert24BitColorTo18Bit(unsigned int color);
    static unsigned int ConvertRGBColorTo18Bit(unsigned char r,unsigned char g,unsigned char b);

    static void NextLine();
    static void CheckLine();
    static void NextChar();
    static void SetDrawRegion(int x, int y, int width, int height);
    static void LCDDrawRegion(int x, int y, int width, int height);

    static int current_line;
    static int current_char;
	static RegisterColorValues palette[4];
    static int lastx;
    static int lasty;

    static int _foreground_palette_index, _background_palette_index;

    void LCDSetColor(uint8_t palette_index);

    static void LCDDrawPixel();

    static void LCDDrawEnd();

    static void LCDDrawPrepare();
};
