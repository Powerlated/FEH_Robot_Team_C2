#ifndef FEHLCD_H
#define FEHLCD_H

#include "spi.h"
#include "derivative.h"
#include <LCDColors.h>

/**
* @brief Access to the Proteus LCD
*
* Allows user to edit and interact with Proteus LCD screen 
*/
class FEHLCD
{
public:

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

    /**
     * @brief Sets initial state and boundaries on LCD board
     * 
     * Initializes foreground and background colors, orientation,
     * initial haracter location, and sets screen interaction boundaries.
     * Also sets register color values.
     */
    FEHLCD();
	
    /**
     * @brief Processes user touch on the LCD screen
     * 
     * @param x_pos
     *      X-coordinate that the user touches
     * @param y_pos
     *      Y-coordinate that the user touches
     * 
     * @return the boolean value of whether or not the user has
     * touched the board.
     */
	bool Touch(float *x_pos,float *y_pos);
	bool Touch(int* x_pos, int* y_pos);

    void ClearBuffer();

    /**
     * @brief Prints an image to the LCD screen
     * 
     * @param x
     *      X-coordinate for the start of the draw region
     *  
     * @param y
     *      Y-coordinate for the start of the draw region
     * 
     * Draws image to the LCD screen for sizeof(image) / sizeof(image[0])
     */
    void PrintImage(int x, int y);

    /**
     * @brief Prints OSU logo to the LCD screen
     * 
     * @param x
     *      X-coordinate for the start of the draw region
     *  
     * @param y
     *      Y-coordinate for the start of the draw region
     * Draws logo to the LCD screen for sizeof(logo) / sizeof(logo[0])
     */
    void PrintLogo(int x, int y);

    /**
     * @brief Access function to private member {@code} _Initialize()
     * 
     * If LCD is not already intialized, do so by calling {@code} _Initialize()
     */
    void Initialize();

    /**
     * @brief Sets orientation of LCD screen
     * 
     * @param Orientation
     *      Current orientation of LCD screen
     * 
     * Calls {@code} WriteParameter() and assigns different maximum
     * lines and columns as well as width and height boundaries on the
     * LCD screen
     */
    void SetOrientation(FEHLCDOrientation orientation);

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
    void Clear( FEHLCDColor color );
    void Clear(unsigned int color);
    void Clear();

    /**
     * @brief Sets LCD font color
     * 
     * @param color
     *      Color of font to be set
     * 
     * Note: This comment applies to all public member variations of
     * {@code} SetFontColor()
     * 
     * Accesses private member {@code} ConvertFEHColorto24Bit() if
     * {@param} color is type FEHLCDColor or private member {@code}
     * Convert24BitColorTo16Bit() if {@param} color is an unsigned int.
     * Register values are set after necessary bit conversions are made
     */
    void SetFontColor( FEHLCDColor color );
    void SetFontColor( unsigned int color);

    /**
     * @brief Sets LCD background color
     * 
     * @param color
     *      Color of the background to be set
     * 
     * Note: This comment applies to all public member variations of
     * {@code} SetBackgroundColor()
     * 
     * Accesses private member {@code} ConvertFEHColorto24Bit() if
     * {@param} color is type FEHLCDColor or private member {@code}
     * Convert24BitColorTo16Bit() if {@param} color is an unsigned int.
     * Register values are set after necessary bit conversions are made
     */
    void SetBackgroundColor( FEHLCDColor color );
    void SetBackgroundColor(unsigned int color);

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

private:
    typedef struct regColVal {
        uint32_t BVal;
        uint32_t CVal;
        uint32_t DVal;
    } RegisterColorValues;
	
	

	void TS_SPI_Init();

	int abs(int);
	
    /**
     * @brief Initialize Proteus for user interaction
     * 
     * Initializes all necessary pins, sets LCD panel mode and Pixel data
     * interface, sets PLL, DotClk, and PWM frequencies, and sets addresses
     * for necessary fields. Also indicates Proteus initialization to user
     * through FEHBuzzer.h
     */
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
	
    int lastx;
    int lasty;

    static unsigned char fontData[];
};

namespace FEHIcon
{
	/* Class definition for software icons */
	class Icon
	{
		private:
			int x_start, x_end;
			int y_start, y_end;
			int width;
			int height;
			unsigned int color;
			unsigned int textcolor;
			char label[200];
			int set;
		public:
			Icon();
			void SetProperties(char name[20], int start_x, int start_y, int w, int h, unsigned int c, unsigned int tc);
			void Draw();
			void Select();
			void Deselect();
			int Pressed(float x, float y, int mode);
			void WhilePressed(float xi, float yi);
            void ChangeLabelString(const char new_label[20]);
            void ChangeLabelFloat(float val);
            void ChangeLabelInt(int val);
    };
		
	/* Function prototype for drawing an array of icons in a rows by cols array with top, bot, left, and right margins from edges of screen, labels for each icon from top left across each row to the bottom right, and color for the rectangle and the text color */
    void DrawIconArray(Icon icon[], int rows, int cols, int top, int bot, int left, int right, char labels[][20], unsigned int col, unsigned int txtcol);
}

extern FEHLCD LCD;

#endif // FEHLCD_H
