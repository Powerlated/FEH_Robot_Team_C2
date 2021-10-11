Using the Touchscreen {#touch}
=======

Introduction {#touch_intro}
===
This tutorial assumes you have finished the [Hello, World!](@ref hello_world) tutorial.

Detecting a touch {#touch_detection_1}
===
Modify the code from the Hello World tutorial to the following:

    #include <FEHLCD.h>

    /**
    * Main function.
    */
    int main() {
        float x, y;

        // Clear the screen
        LCD.Clear();

        while (true) {
            // If the screen is touched
            if (LCD.Touch(&x, &y)) {
                // Write Hello, World! to the screen
                LCD.WriteLine("Hello, World!");
            }
        }

        return 0;
    }

This code works by using the LCD.Touch(float, float) method, which detects whether the screen has been pressed. If pressed, it returns true and sets the x and y pointers that were passed in to the location the screen was touched. This will happen infinitely, as the code that checks if the screen was touched is located in a while(true) loop.

Build and run the program, and the Proteus will print "Hello, World!" to the screen every time it is tapped.

Touch buffer issues {#touch_issues}
===
However, this solution is not perfect. Modify the code to the following:

    #include <FEHLCD.h>
    #include <FEHUtility.h>

    /**
    * Main function.
    */
    int main() {
        float x, y;

        // Clear the screen
        LCD.Clear();
        LCD.WriteLine("Waiting 2 seconds...");
        // Sleep for 2 seconds
        Sleep(2.0);

        while (true) {
            // If the screen is touched
            if (LCD.Touch(&x, &y)) {
                // Write Hello, World! to the screen
                LCD.WriteLine("Hello, World!");
            }
        }

        return 0;
    }

This coded uses the Sleep(int) function to wait for 2 seconds before starting to check for a touch.

Build and run the program, and observe what happens if you press the screen while it says "Waiting 2 seconds...". The program will print "Hello, World!" when it is done sleeping, possibly multiple times. This is because the Proteus stores touches in a buffer, which are then removed as LCD.Touch(float, float) is called. In practice, this can be frustrating in both game design and robot design.

Properly detecting touch {#touch_detection_2}
===
To fix this, LCD.ClearTouchBuffer() (_TODO_) can be used. It clears the touch buffer, ensuring that only touches after it is called are recorded. An example is provided:

    #include <FEHLCD.h>
    #include <FEHUtility.h>

    /**
    * Main function.
    */
    int main() {
        float x, y;

        // Clear the screen
        LCD.Clear();
        LCD.WriteLine("Waiting 2 seconds...");
        // Sleep for 2 seconds
        Sleep(2.0);

        while (true) {
            // Clear the touch buffer
            LCD.ClearTouchBuffer();
            // Wait until the screen is touched
            while (!LCD.Touch(&x, &y)) {}
            // Write Hello, World! to the screen
            LCD.WriteLine("Hello, World!");
        }

        return 0;
    }

This code uses a different way to check for a touch. It clears the touch buffer, then enters a while loop that does nothing until the screen is pressed. Once the Proteus exists this loop, indiciating the screen was pressed, "Hello, World!" is printed to the screen and the loop repeats infinitely.

This code exhibits how to properly use LCD.ClearTouchBuffer() to ensure that erroneous touches are discarded.