/* CodeWarrior ARM Runtime Support Library
 * Copyright � 2012 Freescale Semiconductors. All rights reserved.
 *
 * $Date: 2012/07/23 06:20:27 $
 * $Revision: 1.5 $
 */


/*
 *	__arm_start.c	-	entry-point for ARM programs
 *
 */
#include <string.h>
#include <stdlib.h>
#include <runtime_configuration.h>
#include "derivative.h"
#include "FEHProteus.h"
#include "FEHUtility.h"
#include "FEHIO.h"
#include "mcg.h"
#include "lptmr.h"

_EWL_BEGIN_EXTERN_C

#if SUPPORT_SEMIHOST_ARGC_ARGV
#define __MAX_CMDLINE_ARGS 10
static char *argv[__MAX_CMDLINE_ARGS] = { 0 };
#else
static char *argv[] = { 0 };
#endif

#if __GNUC__
#define __call_static_initializers __init_cpp
#endif
#if SUPPORT_SEMIHOST_ARGC_ARGV
extern int __argc_argv(char **, int);
#endif /* SUPPORT_SEMIHOST_ARGC_ARGV */

extern void __call_static_initializers(void);
extern int main(int, char **);
void InitFEHProteus();
void InitPowerButton();

#ifdef __VFPV4__
extern void __fp_init(void);
#endif /* __VFPV4__ */

extern void __init_registers();
extern void __init_hardware();
extern void __init_user();

#if defined(__APCS_ROPI)
extern void __init_pic();
#endif

#if defined(__APCS_RWPI)
extern void __init_pid();
#endif

#if defined(__APCS_ROPI) || defined(__APCS_RWPI)
extern void __load_static_base();
#endif

#if defined(__SEMIHOSTING)
extern void __init_semihost(void) _EWL_WEAK;
#endif

#if SUPPORT_ROM_TO_RAM
extern void __copy_rom_sections_to_ram(void);
extern char __S_romp[];
#endif

static void zero_fill_bss(void)
{
	extern char __START_BSS[];
	extern char __END_BSS[];

	memset(__START_BSS, 0, (__END_BSS - __START_BSS));
}

#ifndef __thumb // Thumb version
#error Thumb startup
#endif

// To keep iar debugger happy
void __iar_program_start(void) _EWL_NAKED;
void __thumb_startup(void);
void __iar_program_start()
{
	__thumb_startup();
}


void __thumb_startup(void) _EWL_NAKED;
void __thumb_startup(void)
{
		// Setup registers
		__init_registers();

		// setup hardware
		__init_hardware();


#if defined(__APCS_ROPI) || defined(__APCS_RWPI)
		//	static base register initialization
		__load_static_base();
#endif
#if defined(__APCS_RWPI)
		//	-pid
		//	setup static base for SB relative position independent data
		//	perform runtime relocation
		__init_pid();
#endif
#if defined(__APCS_ROPI)
		//	-pic
		//	perform runtime relocation for position independent code
		__init_pic();
#endif
		//	zero-fill the .bss section
		zero_fill_bss();

#if SUPPORT_ROM_TO_RAM
		if (__S_romp != 0L)
			__copy_rom_sections_to_ram();
#endif


		//      initialize the floating-point library
#ifdef __VFPV4__
		__fp_init();
#endif

		//	call C++ static initializers
		__call_static_initializers();

		// initializations before main, user specific
		__init_user();

#if defined(__SEMIHOSTING)
		// semihost initializations
		__init_semihost();
#endif

		//	call main(argc, &argv)
#if SUPPORT_SEMIHOST_ARGC_ARGV
		exit(main(__argc_argv(argv, __MAX_CMDLINE_ARGS), argv));
#else
        InitFEHProteus();
        //InitPowerButton();
		exit(main(0, argv));
#endif

		//	should never get here
		while (1);

}

void InitFEHProteus()
{
    // Initialize GPIO Clocks
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK );
	// Initialize ADC Clocks
	InitClocks();
	// Initialize ADCs
	InitADCs();
    // Setup clocks
    CoreClockMHz = pll_init( CORE_CLK_MHZ, REF_CLK );
    CoreClockKHz = CoreClockMHz * 1000;
    PeripheralClockKHz = CoreClockKHz / ( ( ( SIM_CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK ) >> 24 ) + 1 );

    InitPowerButton();

    Propeller.Initialize();

    Sleep( 5000 );
}

void InitPowerButton()
{
    //tVectorTable* vect_table;

    // Power button kill pin = D13
    PORT_PCR_REG( PORTD_BASE_PTR, 13 ) = ( 0 | PORT_PCR_MUX( 1 ) );
    GPIOD_PDDR |= GPIO_PDDR_PDD( GPIO_PIN( 13 ) );

    // Set kill pin high to keep the push button controller active
    GPIOD_PDOR |= GPIO_PDOR_PDO( GPIO_PIN( 13 ) );

    // Power button interrupt pin = D12
    // pull up and falling edge interrupt
    PORT_PCR_REG( PORTD_BASE_PTR, 12 ) = ( 0 | PORT_PCR_MUX( 1 ) | PORT_PCR_IRQC( 0xA ) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK );

    // enable interrupt on port D
    //enable_irq( 90 );
    NVICICPR2 = 1 << ( 26 );
    NVICISER2 = 1 << ( 26 );

    //#define VECTOR_106      default_isr     // 0x0000_01A8 106   90     Port control module Pin Detect (Port D)
    //vect_table = (tVectorTable*)SCB_VTOR;
    //vect_table->__fun[ 106 ] = PortDISR;
}

_EWL_END_EXTERN_C
