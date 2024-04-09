.syntax unified
.cpu cortex-m4
.thumb

.text
.NMI_Name: .asciz "NMI"
.HardFault_Name: .asciz "HardFault"
.MemManage_Name: .asciz "MemManage"
.BusFault_Name: .asciz "BusFault"
.UsageFault_Name: .asciz "UsageFault"

.global NMI_Handler
.global HardFault_Handler
.global MemManage_Handler
.global BusFault_Handler
.global UsageFault_Handler

.thumb_func
NMI_Handler:
    ldr r2, =.NMI_Name
    b .collect_info
.thumb_func
HardFault_Handler:
    ldr r2, =.HardFault_Name
    b .collect_info
.thumb_func
MemManage_Handler:
    ldr r2, =.MemManage_Name
    b .collect_info
.thumb_func
BusFault_Handler:
    ldr r2, =.BusFault_Name
    b .collect_info
.thumb_func
UsageFault_Handler:
    ldr r2, =.UsageFault_Name
    b .collect_info
.thumb_func

// This version is for Cortex M3, Cortex M4 and Cortex M4F
.thumb_func
.collect_info:
    /* Check EXC_RETURN in Link register bit 2. */
    tst lr, #4
    ittee eq
    /* Choose the stack pointer depending on if MSP or PSP was being used */
    mrseq r0, msp
    moveq r1, #0
    mrsne r0, psp
    movne r1, #1

    /* ExceptionHandler arguments
     * volatile uint32_t *sp - R0
     * bool is_thread_mode - R1
     * const char *type_string - R2
     */
    ldr r2, =.HardFault_Name
    bl ExceptionHandler
.end
