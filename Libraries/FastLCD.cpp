#include "FastLCD.h"
#include "derivative.h"
#include <cstdio>
#include <cstring>
#include <cmath>

using namespace FastLCD;

#define CLR_CS GPIOC_PCOR = ( 1 << 3 )
#define SET_CS GPIOC_PSOR = ( 1 << 3 )

#define CLR_RS GPIOB_PCOR = ( 1 << 18 )
#define SET_RS GPIOB_PSOR = ( 1 << 18 )

#define CLR_Rd GPIOB_PCOR = ( 1 << 16 )
#define SET_RD GPIOB_PSOR = ( 1 << 16 )

#define CLR_WR GPIOB_PCOR = ( 1 << 17 )
#define SET_WR GPIOB_PSOR = ( 1 << 17 )

#define CLR_RESET GPIOB_PCOR = ( 1 << 3 )
#define SET_RESET GPIOB_PSOR = ( 1 << 3 )

constexpr int MAX_LINES = 14;
constexpr int MAX_COLS = 26;
constexpr int WIDTH = 320;
constexpr int HEIGHT = 240;

int current_line;
int current_char;

int foreground_palette_index;

#define CharHeight 17
#define CharWidth  12

unsigned char fontData[] = {
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

#define LCD_WIDTH           320
#define LCD_HEIGHT          240

typedef struct regColVal {
    uint32_t BVal;
    uint32_t CVal;
    uint32_t DVal;
} RegisterColorValues;

// 4-bit LCD buffer
static RegisterColorValues palette[16];
uint32_t LCDBuffer[LCD_WIDTH * LCD_HEIGHT / 8];
int region_x, region_y, region_width, region_height;
int draw_x, draw_y;

void set_pixel(int x, int y, uint8_t val) {
    if ((unsigned int) x >= LCD_WIDTH) return;
    if ((unsigned int) y >= LCD_HEIGHT) return;
    int pixel_number = LCD_WIDTH * y + x;
    int index = pixel_number / 8;
    int bit_number = (pixel_number % 8) * 4;
    LCDBuffer[index] &= ~(0b1111 << bit_number);
    LCDBuffer[index] |= (val << bit_number);
}

void next_pixel() {
    draw_x++;

    if (draw_x >= region_x + region_width) {
        draw_x = region_x;
        draw_y++;

        if (draw_y >= region_y + region_height) {
            draw_y = region_y;
        }
    }
}

int ConvertRGBColorTo18Bit(unsigned char r, unsigned char g, unsigned char b) {
    unsigned int ru = r >> 2;
    unsigned int gu = g >> 2;
    unsigned int bu = b >> 2;
    return (ru << 12) | (gu << 6) | bu;
}

int Convert24BitColorTo18Bit(unsigned int color) {
    unsigned char r = (color & 0xFF0000u) >> 16;
    unsigned char g = (color & 0x00FF00u) >> 8;
    unsigned char b = (color & 0x0000FFu);
    return ConvertRGBColorTo18Bit(r, g, b);
}

void FastLCD::SetFontPaletteIndex(uint8_t index) {
    foreground_palette_index = index;
}

void FastLCD::SetPaletteColor(uint8_t index, unsigned int color) {
    uint32_t rgb18 = Convert24BitColorTo18Bit(color);

    auto c1 = (unsigned char) (rgb18 & 0xFF);
    auto c2 = (unsigned char) ((rgb18 >> 8) & 0xFF);
    auto c3 = (unsigned char) ((rgb18 >> 16) & 0x03);

    palette[index].BVal = 0;
    palette[index].CVal = 0;
    palette[index].DVal = 0;

    palette[index].BVal |= GPIO_PDOR_PDO((((c1 & (1 << 0)) ? (1) : (0)) << 9));
    palette[index].BVal |= GPIO_PDOR_PDO((((c1 & (1 << 1)) ? (1) : (0)) << 8));
    palette[index].CVal |= GPIO_PDOR_PDO((((c1 & (1 << 2)) ? (1) : (0)) << 4));
    palette[index].CVal |= GPIO_PDOR_PDO((((c1 & (1 << 3)) ? (1) : (0)) << 5));
    palette[index].CVal |= GPIO_PDOR_PDO((((c1 & (1 << 4)) ? (1) : (0)) << 6));
    palette[index].CVal |= GPIO_PDOR_PDO((((c1 & (1 << 5)) ? (1) : (0)) << 7));
    palette[index].CVal |= GPIO_PDOR_PDO((((c1 & (1 << 6)) ? (1) : (0)) << 12));
    palette[index].CVal |= GPIO_PDOR_PDO((((c1 & (1 << 7)) ? (1) : (0)) << 13));
    palette[index].CVal |= GPIO_PDOR_PDO((((c2 & (1 << 0)) ? (1) : (0)) << 14));
    palette[index].CVal |= GPIO_PDOR_PDO((((c2 & (1 << 1)) ? (1) : (0)) << 15));
    palette[index].CVal |= GPIO_PDOR_PDO((((c2 & (1 << 2)) ? (1) : (0)) << 16));
    palette[index].CVal |= GPIO_PDOR_PDO((((c2 & (1 << 3)) ? (1) : (0)) << 17));
    palette[index].CVal |= GPIO_PDOR_PDO((((c2 & (1 << 4)) ? (1) : (0)) << 18));
    palette[index].CVal |= GPIO_PDOR_PDO((((c2 & (1 << 5)) ? (1) : (0)) << 19));
    palette[index].DVal |= GPIO_PDOR_PDO((((c2 & (1 << 6)) ? (1) : (0)) << 0));
    palette[index].DVal |= GPIO_PDOR_PDO((((c2 & (1 << 7)) ? (1) : (0)) << 2));
    palette[index].DVal |= GPIO_PDOR_PDO((((c3 & (1 << 0)) ? (1) : (0)) << 3));
    palette[index].DVal |= GPIO_PDOR_PDO((((c3 & (1 << 1)) ? (1) : (0)) << 4));
}

void SetDrawRegion(int x, int y, int width, int height) {
    draw_x = x;
    draw_y = y;
    region_x = x;
    region_y = y;
    region_width = width;
    region_height = height;
}

void FastLCD::WriteAt(const char *str, int x, int y) {
    int i = 0;
    while (str[i] != '\0') {
        WriteCharAt(x, y, str[i]);
        x += CharWidth;
        i++;
    }
}

void FastLCD::WriteAt(int i, int x, int y) {
    char num[50];
    sprintf(num, "%d", i);
    WriteAt(num, x, y);
}

void FastLCD::WriteAt(float f, int x, int y) {
    char num[50];
    int d, r;
    d = (int) f;
    r = (int) ((f - d) * 1000);
    if (f < 0.)
        r = r * -1;
    if (f < 0. && f > -1.)
        sprintf(num, "-%d.%03d", d, r);
    else
        sprintf(num, "%d.%03d", d, r);
    WriteAt(num, x, y);
}

void FastLCD::WriteAt(double d, int x, int y) {
    WriteAt((float) d, x, y);
}

void FastLCD::WriteAt(bool b, int x, int y) {
    if (b) {
        WriteAt("true", x, y);
    } else {
        WriteAt("false", x, y);
    }
}

void FastLCD::WriteAt(char c, int x, int y) {
    char str[1] = {c};
    WriteAt(str, x, y);
}

int abs(int no) {
    if (no < 0) {
        no *= -1;
    }
    return no;
}

void FastLCD::WriteRC(const char *str, int row, int col) {
    int x, y;

    y = row * 17;
    x = col * 12;
    WriteAt(str, x, y);
}

void FastLCD::WriteRC(int i, int row, int col) {
    int x, y;

    y = row * 17;
    x = col * 12;
    WriteAt(i, x, y);
}

void FastLCD::WriteRC(float f, int row, int col) {
    int x, y;

    y = row * 17;
    x = col * 12;
    WriteAt(f, x, y);
}

void FastLCD::WriteRC(double d, int row, int col) {
    int x, y;

    y = row * 17;
    x = col * 12;
    WriteAt(d, x, y);
}

void FastLCD::WriteRC(bool b, int row, int col) {
    int x, y;

    y = row * 17;
    x = col * 12;
    WriteAt(b, x, y);
}

void FastLCD::WriteRC(char c, int row, int col) {
    int x, y;

    y = row * 17;
    x = col * 12;
    WriteAt(c, x, y);
}

void FastLCD::Write(const char *str) {
    int i = 0;
    CheckLine();
    while (str[i] != '\0') {
        WriteChar(current_line, current_char, str[i]);
        NextChar();
        i++;
    }
}

void FastLCD::Write(int i) {
    char num[50];
    sprintf(num, "%d", i);
    Write(num);
}

void FastLCD::Write(float f) {
    char num[50];
    int d, r;
    if (f >= 0) {
        d = (int) f;
        r = (int) ((f - d) * 1000);
        sprintf(num, "%d.%03d", d, r);
    } else {
        f *= -1;
        d = (int) f;
        r = (int) ((f - d) * 1000);
        sprintf(num, "-%d.%03d", d, r);
    }
    Write(num);
}

void FastLCD::Write(double d) {
    Write((float) d);
}

void FastLCD::Write(bool b) {
    if (b) {
        Write("true");
    } else {
        Write("false");
    }
}

void FastLCD::Write(char c) {
    CheckLine();
    WriteChar(current_line, current_char, c);
    NextChar();
}

void FastLCD::WriteLine(const char *str) {
    CheckLine();
    Write(str);
    NextLine();
}

void FastLCD::WriteLine(int i) {
    CheckLine();
    Write(i);
    NextLine();
}

void FastLCD::WriteLine(float f) {
    CheckLine();
    Write(f);
    NextLine();
}

void FastLCD::WriteLine(double d) {
    CheckLine();
    Write(d);
    NextLine();
}

void FastLCD::WriteLine(bool b) {
    CheckLine();
    Write(b);
    NextLine();
}

void FastLCD::WriteLine(char c) {
    CheckLine();
    Write(c);
    NextLine();
}

void WriteIndex(unsigned char index) {
    // CS = 0;
    CLR_CS;

    // RS = 0;
    CLR_RS;

    // RD = 1;
    SET_RD;

    // dataport0 = index;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO((1 << 9) | (1 << 8));
    GPIOC_PDOR &= ~GPIO_PDOR_PDO((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 12) | (1 << 13));

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO((((index & (1 << 0)) ? (1) : (0)) << 9));
    GPIOB_PDOR |= GPIO_PDOR_PDO((((index & (1 << 1)) ? (1) : (0)) << 8));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((index & (1 << 2)) ? (1) : (0)) << 4));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((index & (1 << 3)) ? (1) : (0)) << 5));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((index & (1 << 4)) ? (1) : (0)) << 6));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((index & (1 << 5)) ? (1) : (0)) << 7));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((index & (1 << 6)) ? (1) : (0)) << 12));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((index & (1 << 7)) ? (1) : (0)) << 13));

    // WR = 0;
    CLR_WR;

    // // delay2( 1 )

    // WR = 1;
    SET_WR;

    // CS = 1;
    SET_CS;

    // // dataport0 = 0xff;
    GPIOB_PDOR |= GPIO_PDOR_PDO((1 << 9) | (1 << 8));
    GPIOC_PDOR |= GPIO_PDOR_PDO((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 12) | (1 << 13));

    // RS = 1;
    SET_RS;

    // delay2( 1 );
}

void WriteParameter(unsigned char param) {
    // CS = 0;
    CLR_CS;

    // RD = 1;
    SET_RD;

    // RS = 1;
    SET_RS;

    // dataport0 = data1;
    // clear all bits
    GPIOB_PDOR &= ~GPIO_PDOR_PDO((1 << 9) | (1 << 8));
    GPIOC_PDOR &= ~GPIO_PDOR_PDO((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 12) | (1 << 13));

    // Set index bits
    GPIOB_PDOR |= GPIO_PDOR_PDO((((param & (1 << 0)) ? (1) : (0)) << 9));
    GPIOB_PDOR |= GPIO_PDOR_PDO((((param & (1 << 1)) ? (1) : (0)) << 8));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((param & (1 << 2)) ? (1) : (0)) << 4));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((param & (1 << 3)) ? (1) : (0)) << 5));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((param & (1 << 4)) ? (1) : (0)) << 6));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((param & (1 << 5)) ? (1) : (0)) << 7));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((param & (1 << 6)) ? (1) : (0)) << 12));
    GPIOC_PDOR |= GPIO_PDOR_PDO((((param & (1 << 7)) ? (1) : (0)) << 13));

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

void FastLCD::Clear() {
    current_line = 0;
    current_char = 0;

    memset(LCDBuffer, 0, sizeof(LCDBuffer));
}

void FastLCD::NextLine() {
    if (current_char > 0) {
        current_line++;
        current_char = 0;
    }
}

void FastLCD::CheckLine() {
    if (current_line >= MAX_LINES) {
        current_line = 0;
        Clear();
    }
}

void FastLCD::NextChar() {
    current_char++;
    if (current_char == MAX_COLS) {
        NextLine();
        CheckLine();
    }
}

void _ForePixel() {
    set_pixel(draw_x, draw_y, foreground_palette_index);
    next_pixel();
}

void FastLCD::DrawPixel(int x, int y) {
    SetDrawRegion(x, y, 1, 1);
    _ForePixel();
}

static void Swap(int &a, int &b) {
    int c = a;
    a = b;
    b = c;
}

void FastLCD::DrawHorizontalLine(int y, int x1, int x2) {
    if (x2 < x1)
        Swap(x1, x2);
    int length = x2 - x1 + 1;
    SetDrawRegion(x1, y, length, 1);

    for (int i = 0; i < WIDTH; i++)
        _ForePixel();
}

void FastLCD::DrawVerticalLine(int x, int y1, int y2) {
    if (y2 < y1)
        Swap(y1, y2);
    int length = y2 - y1 + 1;
    SetDrawRegion(x, y1, 1, length);

    for (int i = 0; i < length; i++)
        _ForePixel();
}


void FastLCD::DrawRectangle(int x, int y, int width, int height) {
    DrawLine(x, y, x + width, y, false);
    DrawLine(x + width, y, x + width, y + height, false);
    DrawLine(x + width, y + height, x, y + height, false);
    DrawLine(x, y + height, x, y, false);
}

void LCDDrawRegion(int x, int y, int width, int height) {
    unsigned int pixStartCol, pixEndCol, pixStartRow, pixEndRow;

    pixStartCol = x;
    pixEndCol = pixStartCol + width - 1;

    pixStartRow = y;
    pixEndRow = pixStartRow + height - 1;

    // widx( 0x2a ); //set column adress
    WriteIndex(0x2A);

    // wpat( 0x00 ); //start column number high byte
    WriteParameter((pixStartCol & 0xf00) >> 8);

    // wpat( 0x00 ); //start column number low byte
    WriteParameter(pixStartCol & 0xff);

    // wpat( 0x01 ); //end column number high byte
    WriteParameter((pixEndCol & 0x0f00) >> 8);

    // wpat( 0x3f ); //end column number low byte
    WriteParameter(pixEndCol & 0xff);


    // widx( 0x2b ); //set row address
    WriteIndex(0x2B);

    // wpat( 0x00 ); //start row number high byte
    WriteParameter((pixStartRow & 0x0f00) >> 8);

    // wpat( 0x00 ); //start row number low byte
    WriteParameter(pixStartRow & 0xff);

    // wpat( 0x00 ); //end row number high byte
    WriteParameter((pixEndRow & 0x0f00) >> 8);

    // wpat( 0xef ); //end row number low byte
    WriteParameter(pixEndRow & 0xff);

    // widx( 0x2c );       //enter write mode
    WriteIndex(0x2C);
}

void FastLCD::FillRectangle(int x, int y, int width, int height) {
    SetDrawRegion(x, y, width, height);
    for (int i = 0; i < width * height; i++)
        _ForePixel();
}

/*
 * plotLine(x0, y0, x1, y1)
    dx = abs(x1 - x0)
    sx = x0 < x1 ? 1 : -1
    dy = -abs(y1 - y0)
    sy = y0 < y1 ? 1 : -1
    error = dx + dy

    while true
        plot(x0, y0)
        if x0 == x1 && y0 == y1 break
        e2 = 2 * error
        if e2 >= dy
            if x0 == x1 break
            error = error + dy
            x0 = x0 + sx
        end if
        if e2 <= dx
            if y0 == y1 break
            error = error + dx
            y0 = y0 + sy
        end if
    end while
 */

// Those goobers used float math for line drawing 🤣🤣🤣🤣🤣🤣🤣🤣🤣🤣
void FastLCD::DrawLine(int x0, int y0, int x1, int y1, bool thick) {
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while (true) {
        set_pixel(x0, y0, foreground_palette_index);
        if (thick) {
            set_pixel(x0 + 1, y0, foreground_palette_index);
            set_pixel(x0 - 1, y0, foreground_palette_index);
            set_pixel(x0, y0 + 1, foreground_palette_index);
            set_pixel(x0, y0 - 1, foreground_palette_index);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * error;
        if (e2 >= dy) {
            if (x0 == x1) break;
            error += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            if (y0 == y1) break;
            error += dx;
            y0 += sy;
        }
    }
}

void FastLCD::DrawCircle(int x0, int y0, int radius) {
    // This alogorithm is from wikipedia
    // It's called the "midpoint circle algorithm"
    // or the "Bresenham's circle algorithm"
    // http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
    // See the page for further details
    int f = 1 - radius;
    int ddF_x = 1;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    DrawPixel(x0, y0 + radius);
    DrawPixel(x0, y0 - radius);
    DrawPixel(x0 + radius, y0);
    DrawPixel(x0 - radius, y0);

    while (x < y) {
        // ddF_x == 2 * x + 1;
        // ddF_y == -2 * y;
        // f == x*x + y*y - radius*radius + 2*x - y + 1;
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        DrawPixel(x0 + x, y0 + y);
        DrawPixel(x0 - x, y0 + y);
        DrawPixel(x0 + x, y0 - y);
        DrawPixel(x0 - x, y0 - y);
        DrawPixel(x0 + y, y0 + x);
        DrawPixel(x0 - y, y0 + x);
        DrawPixel(x0 + y, y0 - x);
        DrawPixel(x0 - y, y0 - x);
    }
}

void FastLCD::FillCircle(int x0, int y0, int radius) {
    // This algorithm is a variant on DrawCircle.
    // Rather than draw the points around the circle,
    // We connect them with a series of lines
    // to fill in the circle.

    int f = 1 - radius;
    int ddF_x = 1;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;


    DrawVerticalLine(x0, y0 - radius, y0 + radius);
    DrawHorizontalLine(y0, x0 - radius, x0 + radius);

    while (x < y) {
        // ddF_x == 2 * x + 1;
        // ddF_y == -2 * y;
        // f == x*x + y*y - radius*radius + 2*x - y + 1;
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        DrawHorizontalLine(y0 + x, x0 - y, x0 + y);
        DrawHorizontalLine(y0 - x, x0 - y, x0 + y);
        DrawVerticalLine(x0 + x, y0 - y, y0 + y);
        DrawVerticalLine(x0 - x, y0 - y, y0 + y);
    }
}

void FastLCD::WriteChar(int charRow, int charCol, char charNum) {
    WriteCharAt(2 + charCol * CharWidth, 3 + charRow * CharHeight, charNum);
}

void FastLCD::WriteCharAt(int x, int y, char charNum) {
    if (charNum == '\n') {
        CheckLine();
        NextLine();
        return;
    }

    if (charNum > 125 || charNum < 32) {
        charNum = 32;
    }
    charNum -= 32;

    SetDrawRegion(x, y, CharWidth, CharHeight);

    // Look up the bitmap data out of the fontData table
    // Each entry in the fontData table corresponds
    // to a column of pixels in the 5x7 bitmapped character
    unsigned char charData[5];
    charData[0] = fontData[5 * charNum];
    charData[1] = fontData[5 * charNum + 1];
    charData[2] = fontData[5 * charNum + 2];
    charData[3] = fontData[5 * charNum + 3];
    charData[4] = fontData[5 * charNum + 4];

    // The actual characters here are 12x17. They are double size
    // and have 2 pixels of border on the left, and 3 on the bottom.

    bool pix;

    // Loop through the character data to draw the character
    // prevPix stores whether the previous pixel
    // was a forePixel. This allows the more efficient "Repeat pixel"
    // function when a pixel color is repeated.
    for (int row = 0; row < 7; row++) {
        // 2 pixels of left border
        next_pixel();
        next_pixel();

        // Loop through the columns of the character
        for (int col = 0; col < 5; col++) {
            // get the current pixel from the end of the
            // character data array
            bool pix = charData[col] & 0x01;

            if (pix) {
                _ForePixel();
                _ForePixel();
            } else {
                next_pixel();
                next_pixel();
            }
            //charData[col] >>= 1;
            //WriteData1( n );
        }

        // Now we're at the next line of the display
        // so we need two more pixels of border
        next_pixel();
        next_pixel();

        // Loop through the columns again
        for (int col = 0; col < 5; col++) {
            bool pix = charData[col] & 0x01;
            if (pix) {
                _ForePixel();
                _ForePixel();
            } else {
                next_pixel();
                next_pixel();
            }
            // Now we're done with this column of the character
            // so we bitshit to get the next pixel for this column
            // queued up for the next time through the loop.
            charData[col] >>= 1;
            //WriteData1( n );
        }
    }

    // Draw the three rows of padding at the bottom
    for (int i = 0; i < CharWidth * 3; i++) {
        next_pixel();
    }
}

void FastLCD::LCDDrawPixel(uint8_t palette_index) {
    // Bit 17 is WR, we are clearing it here instead of using CLR_WR
    GPIOB_PCOR = (1 << 9) | (1 << 8) | (1 << 17);
    GPIOC_PCOR = (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16) |
                 (1 << 17) | (1 << 18) | (1 << 19);
    GPIOD_PCOR = (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4);

    GPIOB_PSOR = palette[palette_index].BVal;
    GPIOC_PSOR = palette[palette_index].CVal;
    GPIOD_PSOR = palette[palette_index].DVal;

    SET_WR;
}

void FastLCD::LCDDrawPrepare() {
    // RD=1;
    SET_RD;

    // CS=0;
    CLR_CS;

    // RS=1;
    SET_RS;
}

void FastLCD::LCDDrawEnd() {
    // CS=1;
    SET_CS;

    // RS=0;
    CLR_RS;
}

void FastLCD::DrawScreen() {
    LCDDrawRegion(0, 0, LCD_WIDTH, LCD_HEIGHT);
    LCDDrawPrepare();

    for (unsigned long packed_pixels : LCDBuffer) {
        // 8 pixels per uint32_t
        for (int p = 0; p < 8; p++) {
            uint8_t pixel = packed_pixels & 0b1111;
            LCDDrawPixel(pixel);
            packed_pixels >>= 4;
        }
    }

    LCDDrawEnd();
}
