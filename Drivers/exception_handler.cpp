#include <cstdio>
#include <cinttypes>
#include "Libraries/FEHLCD.h"

void print_register(const char *label, uint32_t value) {
    char str[32];
    sprintf(str, "%08" PRIXPTR, (uintptr_t) value);
    LCD.Write(label);
    LCD.WriteLine(str);
}

extern "C" [[maybe_unused]] [[noreturn]]
void ExceptionHandler(volatile uint32_t *sp, bool is_thread_mode, const char *type_string) {
    volatile uint32_t r0, r1, r2, r3, r12, lr, pc, psr_word;

    r0 = sp[0];  // Register R0
    r1 = sp[1];  // Register R1
    r2 = sp[2];  // Register R2
    r3 = sp[3];  // Register R3
    r12 = sp[4];  // Register R12
    lr = sp[5];  // Link register LR
    pc = sp[6];  // Program counter PC
    psr_word = sp[7]; // Word containing IPSR, EPSR, APSR

    LCD.Clear();
    LCD.SetBackgroundColor(FEHLCD::Black);
    LCD.SetFontColor(0xF88379);
    LCD.WriteLine("An exception occurred");
    LCD.SetFontColor(FEHLCD::White);
    LCD.Write("Type: ");
    LCD.WriteLine(type_string);

    if (is_thread_mode) {
        LCD.WriteLine("In thread mode");
    } else {
        LCD.WriteLine("Not in thread mode");
    }

    print_register("  R0: ", r0);
    print_register("  R1: ", r1);
    print_register("  R2: ", r2);
    print_register("  R3: ", r3);
    print_register(" R12: ", r12);
    print_register("  LR: ", lr);
    print_register("  PC: ", pc);
    print_register("PSRs: ", psr_word);
    print_register("  SP: ", (uint32_t) sp);

    while (true) {
        // Handle power button manually since interrupts are disabled in fault mode
        if ((GPIOD_PDIR & (1 << 12)) == false) {
            GPIOD_PCOR = 1 << 13;
        }
    }
}
