#ifndef FEHBUZZER_H
#define FEHBUZZER_H

/**
 * @brief Access to the Proteus buzzer
 *
 * Allows the Proteus to create sound
 */
class FEHBuzzer
{
public:

    /**
     * @brief Enumeration of all 88 frequencies on a piano
     *
     * Enumeration of all 88 frequencies on a piano <br/>
     * Letter = note <br/>
     * 's' = sharp <br/>
     * 'f' = flat <br/>
     * Number = octave
     *
     */
    typedef enum
    {
        C8  = 4186,
        B7  = 3951,
        As7 = 3729,
        Bf7 = 3729,
        A7  = 3520,
        Gs7 = 3322,
        Af7 = 3322,
        G7  = 3136,
        Fs7 = 2960,
        Gf7 = 2960,
        F7  = 2794,
        E7  = 2637,
        Ds7 = 2489,
        Ef7 = 2489,
        D7  = 2349,
        Cs7 = 2217,
        Df7 = 2217,
        C7  = 2093,
        B6  = 1976,
        As6 = 1865,
        Bf6 = 1865,
        A6  = 1760,
        Gs6 = 1661,
        Af6 = 1661,
        G6  = 1568,
        Fs6 = 1480,
        Gf6 = 1480,
        F6  = 1397,
        E6  = 1319,
        Ds6 = 1245,
        Ef6 = 1244,
        D6  = 1175,
        Cs6 = 1109,
        Df6 = 1109,
        C6  = 1047,
        B5  = 988,
        As5 = 932,
        Bf5 = 932,
        A5  = 880,
        Gs5 = 831,
        Af5 = 831,
        G5  = 784,
        Fs5 = 740,
        Gf5 = 740,
        F5  = 698,
        E5  = 659,
        Ds5 = 622,
        Ef5 = 622,
        D5  = 587,
        Cs5 = 554,
        Df5 = 554,
        C5  = 523,
        B4  = 494,
        As4 = 466,
        Bf4 = 466,
        A4  = 440,
        Gs4 = 415,
        Af4 = 415,
        G4  = 392,
        Fs4 = 370,
        Gf4 = 370,
        F4  = 349,
        E4  = 330,
        Ds4 = 311,
        Ef4 = 311,
        D4  = 294,
        Cs4 = 277,
        Df4 = 277,
        C4  = 261,
        B3  = 247,
        As3 = 233,
        Bf3 = 233,
        A3  = 220,
        Gs3 = 208,
        Af3 = 208,
        G3  = 196,
        Fs3 = 185,
        Gf3 = 185,
        F3  = 175,
        E3  = 165,
        Ds3 = 156,
        Ef3 = 156,
        D3  = 147,
        Cs3 = 139,
        Df3 = 139,
        C3  = 131,
        B2  = 123,
        As2 = 117,
        Bf2 = 117,
        A2  = 110,
        Gs2 = 104,
        Af2 = 104,
        G2  = 98,
        Fs2 = 92,
        Gf2 = 92,
        F2  = 87,
        E2  = 82,
        Ds2 = 78,
        Ef2 = 78,
        D2  = 73,
        Cs2 = 69,
        Df2 = 69,
        C2  = 65,
        B1  = 62,
        As1 = 58,
        Bf1 = 58,
        A1  = 55,
        Gs1 = 52,
        Af1 = 52,
        G1  = 49,
        Fs1 = 46,
        Gf1 = 46,
        F1  = 44,
        E1  = 41,
        Ds1 = 39,
        Ef1 = 39,
        D1  = 37,
        Cs1 = 35,
        Df1 = 35,
        C1  = 33,
        B0  = 31,
        As0 = 29,
        Bf0 = 29,
        A0  = 28
    } stdnote;

    /**
     * @brief Beeps for 500 miliseconds at a frequency of 1000 Hz
     *
     * Beeps for 500 miliseconds at a frequency of 1000 Hz
     *
     */
    void Beep();

    /**
     * @brief Beeps infinitely at a frequency of 1000 Hz
     *
     * Beeps infinitely at a frequency of 1000 Hz
     *
     */
    void Buzz();

    /**
     * @brief Beeps for a user specified amount of time (in seconds) at a frequency of 1000 Hz
     *
     * Beeps for a user specified amount of time (in seconds) at a frequency of 1000 Hz
     *
     * @param double Amount of time (seconds)
     */
    void Buzz(double);

    /**
     * @brief Beeps for a user specified amount of time (in milliseconds) at a frequency of 1000 Hz
     *
     * Beeps for a user specified amount of time (in milliseconds) at a frequency of 1000 Hz
     *
     * @param int Amount of time (milliseconds)
     */
    void Buzz(int);

    /**
     * @brief Beeps infinitely at a user specfied frequency
     *
     * Beeps infinitely at a user specfied frequency
     *
     * @param int Frequency
     */
    void Tone(int);

    /**
     * @brief Beeps infinitely at a user specfied frequency
     *
     * Beeps infinitely at a user specfied frequency
     *
     * @param stdnote Frequency from stdnote enumeration
     */
    void Tone(stdnote);

    /**
     * @brief Beeps for a user specified amount of time (in milliseconds) at a a user specfied frequency
     *
     * Beeps for a user specified amount of time (in milliseconds) at a a user specfied frequency
     *
     * @param int Frequency
     * @param int Amount of time (milliseconds)
     */
    void Tone(int, int);

    /**
     * @brief Beeps for a user specified amount of time (in seconds) at a a user specfied frequency
     *
     * Beeps for a user specified amount of time (in seconds) at a a user specfied frequency
     *
     * @param int Frequency
     * @param double Amount of time (seconds)
     */
    void Tone(int, double);

    /**
     * @brief Beeps for a user specified amount of time (in milliseconds) at a a user specfied frequency
     *
     * Beeps for a user specified amount of time (in milliseconds) at a a user specfied frequency
     *
     * @param stdnote Frequency from stdnote enumeration
     * @param int Amount of time (milliseconds)
     */
    void Tone(stdnote, int);

    /**
     * @brief Beeps for a user specified amount of time (in seconds) at a a user specfied frequency
     *
     * Beeps for a user specified amount of time (in seconds) at a a user specfied frequency
     *
     * @param stdnote Frequency from stdnote enumeration
     * @param double Amount of time (seconds)
     */
    void Tone(stdnote, double);

    /**
     * @brief Turns off buzzer
     *
     * Turns off buzzer
     *
     */
    void Off();

    /**
     * @brief Construct a new FEHBuzzer object
     *
     * Contains enumeration of all 88 frequencies on a piano
     *
     */
    FEHBuzzer();

};

extern FEHBuzzer Buzzer;
#endif // FEHBUZZER_H
