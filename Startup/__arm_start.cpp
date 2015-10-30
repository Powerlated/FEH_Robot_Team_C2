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
#include "FEHLCD.h"
#include "FEHUtility.h"
#include "FEHIO.h"
#include "mcg.h"
#include "lptmr.h"

_EWL_BEGIN_EXTERN_C

#define USE_88_MHZ_CLOCK

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
static unsigned char pll_init_bl();

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

#ifdef MCU_MK60N512VMD100
#define BSP_CLOCK_SRC                   (50000000ul)       // crystal, oscillator freq
#else
#define BSP_CLOCK_SRC                   (8000000ul)       // crystal, oscillator freq
#endif
#define BSP_REF_CLOCK_SRC               (2000000ul)       // must be 2-4MHz

// original used in BL
//#define BSP_CORE_DIV                    (1)
//#define BSP_BUS_DIV                     (1)
//#define BSP_FLEXBUS_DIV                 (1)
//#define BSP_FLASH_DIV                   (2)

#define BSP_CORE_DIV                    (1)
#define BSP_BUS_DIV                     (2)
#define BSP_FLEXBUS_DIV                 (2)
#define BSP_FLASH_DIV                   (4)
/* BSP_CLOCK_MUL from interval 24 - 55 */
//#define BSP_CLOCK_MUL                   (24)    /* 48MHz */
#define BSP_CLOCK_MUL                   (44)    /* 48MHz */

#define BSP_REF_CLOCK_DIV               (BSP_CLOCK_SRC / BSP_REF_CLOCK_SRC)

void InitFEHProteus()
{
	// Initialize GPIO Clocks
	SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK );

	InitPowerButton();

	// Setup clocks
#ifndef USE_88_MHZ_CLOCK
	CoreClockMHz = pll_init( CORE_CLK_MHZ, REF_CLK );
#else
	pll_init_bl();
	CoreClockMHz = BSP_CLOCK_MUL * 2;
#endif
	CoreClockKHz = CoreClockMHz * 1000;
	PeripheralClockKHz = CoreClockKHz / ( ( ( SIM_CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK ) >> 24 ) + 1 );

	// Initialize ADC and PIT Clocks
	SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK | SIM_SCGC6_PIT_MASK | SIM_SCGC6_RTC_MASK);
	SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK );

	// Turn on Real Time Clock Oscillator
	RTC_CR = RTC_CR_OSCE_MASK;
	// Wait for it to come online
	// Use time_delay_ms (which uses the Low power timer) since Sleep needs the RTC
	time_delay_ms(100);
	
	// Enable Real Time Clock
	RTC_SR &=  ~RTC_SR_TCE_MASK;
	RTC_TSR = 0x0u;
	RTC_SR =  RTC_SR_TCE_MASK;

	// Initialize ADCs
	AnalogInputPin::InitADCs();
	AnalogEncoder::Init();

	Propeller.Initialize();

	// Initialize LCD
	LCD.Initialize();

    Sleep(5000);
}

/*****************************************************************************
* It will configure the MCU to disable STOP and COP Modules.
* It also set the MCG configuration and bus clock frequency.
****************************************************************************/
static unsigned char pll_init_bl()
{
   /*This assumes that the MCG is in default FEI mode out of reset. */

   /* First move to FBE mode */
#ifdef MCU_MK60N512VMD100
   /* Enable external oscillator, RANGE=0, HGO=, EREFS=, LP=, IRCS= */
   MCG_C2 = 0;
#else
   /* Enable external oscillator, RANGE=2, HGO=1, EREFS=1, LP=0, IRCS=0 */
   MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK|MCG_C2_IRCS_MASK;
#endif

   /* Select external oscilator and Reference Divider and clear IREFS to start ext osc
	  CLKS=2, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0 */
   MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);

#ifndef MCU_MK60N512VMD100
   /* wait for oscillator to initialize */
  while (!(MCG_S & MCG_S_OSCINIT_MASK)){};
#endif

  /* wait for Reference clock Status bit to clear */
   while (MCG_S & MCG_S_IREFST_MASK){};

   while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}; /* Wait for clock status bits to show clock source is ext ref clk */

#ifdef MCU_MK60N512VMD100
   MCG_C5 = MCG_C5_PRDIV(BSP_REF_CLOCK_DIV - 1);
#else
   MCG_C5 = MCG_C5_PRDIV(BSP_REF_CLOCK_DIV - 1) | MCG_C5_PLLCLKEN_MASK;
#endif
   /* Ensure MCG_C6 is at the reset default of 0. LOLIE disabled,
	PLL enabled, clk monitor disabled, PLL VCO divider is clear */
   MCG_C6 = 0;

   /* Set system options dividers */

   SIM_CLKDIV1 =   SIM_CLKDIV1_OUTDIV1(BSP_CORE_DIV - 1) | 	/* core/system clock */
				   SIM_CLKDIV1_OUTDIV2(BSP_BUS_DIV - 1)  | 	/* peripheral clock; */
				   SIM_CLKDIV1_OUTDIV3(BSP_FLEXBUS_DIV - 1) |  /* FlexBus clock driven to the external pin (FB_CLK)*/
				   SIM_CLKDIV1_OUTDIV4(BSP_FLASH_DIV - 1);     /* flash clock */

   /* Set the VCO divider and enable the PLL, LOLIE = 0, PLLS = 1, CME = 0, VDIV = */
   //MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(BSP_CLOCK_MUL - 24); /* 2MHz * BSP_CLOCK_MUL */
   MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(BSP_CLOCK_MUL - 24); /* 2MHz * BSP_CLOCK_MUL */

   while (!(MCG_S & MCG_S_PLLST_MASK)){}; /* wait for PLL status bit to set */
   while (!(MCG_S & MCG_S_LOCK_MASK)){}; /* Wait for LOCK bit to set */

   /* Transition into PEE by setting CLKS to 0
   CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0 */
   MCG_C1 &= ~MCG_C1_CLKS_MASK;

   /* Wait for clock status bits to update */
   while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};

   return 0;
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
    NVICICPR2 |= 1 << ( 26 );
    NVICISER2 |= 1 << ( 26 );

    //#define VECTOR_106      default_isr     // 0x0000_01A8 106   90     Port control module Pin Detect (Port D)
    //vect_table = (tVectorTable*)SCB_VTOR;
    //vect_table->__fun[ 106 ] = PortDISR;
}

_EWL_END_EXTERN_C
