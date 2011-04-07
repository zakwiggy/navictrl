
        .include "startup_generic.s"

# reference to external interrupt handlers
#
                .extern SWI_Handler
                .extern Prefetch_Handler
                .extern Abort_Handler
                .extern Undefined_Handler
                .extern FIQ_Handler


	.equ	VectorAddress,		0xFFFFF030	/* VIC Vector address register address. */
	.equ	VectorAddressDaisy,	0xFC000030	/* Daisy VIC Vector address register */


#		
# Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs
#
        .equ    Mode_USR,   0x10
        .equ    Mode_FIQ,   0x11
        .equ    Mode_IRQ,   0x12
        .equ    Mode_SVC,   0x13
        .equ    Mode_ABT,   0x17
        .equ    Mode_UND,   0x1B
        .equ    Mode_SYS,   0x1F
#
        .equ    I_BIT, 0x80        /* when I bit is set, IRQ is disabled */
        .equ    F_BIT, 0x40        /* when F bit is set, FIQ is disabled */
#
#
# System Memory definitions
#
#  Internal RAM definitions
        .equ    RAM_Size,      0x0018000      /* 96K */
        .equ    RAM_Base,      0x4000000

#*************************************************************************
# Control Startup Code Operation
#*************************************************************************
.equ 	SRAM_SETUP  ,   1     /* Enable setup of SRAM */
.equ	FMI_SETUP   ,   0     /* Enable FMI Setup */   /* already done by bootloader */
.equ	CLOCK_SETUP ,   1 	  /* Enable clock setup */
.equ	ETM_SETUP ,   0 	  /* Enable ETM setup */

#*************************************************************************
# Hardware Definitions 
#*************************************************************************

# Flash Memory Interface (FMI) definitions (Flash banks sizes and addresses)
.equ  	FMI_BASE      	,     0x54000000      /* FMI Base Address (non-buffered) */
.equ 	FMI_BBSR_OFS  	,     0x00            /* Boot Bank Size Register */
.equ 	FMI_NBBSR_OFS 	,     0x04            /* Non-boot Bank Size Register	   */
.equ 	FMI_BBADR_OFS 	,     0x0C            /* Boot Bank Base Address Register       #!!! Documentation page 30,*/
.equ 	FMI_NBBADR_OFS 	,     0x10            /* Non-boot Bank Base Address Register   #!!! adresseses do not correspond*/
.equ 	FMI_CR_OFS    	,     0x18            /* Control Register */
.equ	FMI_SR_OFS		,	  0x1C            /* Status Register */
			   

.equ	FMI_CR_Val      ,     0x00000018
.equ	FMI_BBSR_Val    ,     0x00000004  /* 04 */
.equ	FMI_BBADR_Val   ,     0x00000000  /* 00 */
.equ	FMI_NBBSR_Val   ,     0x00000002  /* 02 */
.equ	FMI_NBBADR_Val  ,     0x00080000  /* 80000 */
.equ	FLASH_CFG_Val   ,     0x00001010
.equ	FMI_SR_Val      ,     0x00000003      /* Clear status errors (register not in STR912 manual! */
	

# System Control Unit (SCU) definitions
.equ	SCU_BASE        ,     0x5C002000      /* SCU Base Address (non-buffered)	       */
.equ	SCU_CLKCNTR_OFS ,     0x00            /* Clock Control register Offset		       */
.equ	SCU_PLLCONF_OFS ,     0x04            /* PLL Configuration register Offset		   */
.equ	SCU_SYSTAT_OFS	,	  0x08            /* SCU status register offset                */
.equ	SCU_PCGR0_OFS   ,     0x14            /* Peripheral Clock Gating Register 0 Offset */
.equ	SCU_PCGR1_OFS   ,     0x18            /* Peripheral Clock Gating Register 1 Offset */
.equ	SCU_SCR0_OFS    ,     0x34            /* System Configuration Register 0 Offset	   */


.equ	SCU_CLKCNTR_Val ,     0x00031004      /* Use PLL, external memory ratio/2 */
.equ	SCU_PLLCONF_Val ,     0x000BC019
.equ	SCU_PCGR0_Val   ,     0x00000FFB	  /* Setup ext mem clock, EMI, SRAM, Prefetch Queue/Branch cache, FMI */
.equ	SCU_PCGR1_Val   ,     0x00FEC801	  /* Setup GPIO8, 9 & 4 										      */
.equ 	SCU_SCR0_Val 	,     0x00000196      /* Disable Prefetch Queue and Branch cache, SRAM = 96kb */
.equ	SCU_SYSSTAT_LOCK ,    0x01      	  /* Check for PLL locked 								  */
.equ	SCU_PRR0_OFS    ,     0x1C            /*; Peripheral Reset Register        0 Offset	*/
.equ	SCU_PRR1_OFS    ,     0x20            /* Peripheral Reset Register        1 Offset */


.equ  	P_RESET_SETUP   ,	    1
.equ	SCU_PRR0_Val    ,		0x00001B73
.equ 	SCU_PRR1_Val    ,		0x00FEC801

# APB Bridge 1 & 2 definitions (Peripherals)
.equ	APB0_BUF_BASE   ,     0x48001802      /* APB Bridge 0 Buffered Base Address		 */
.equ	APB0_NBUF_BASE  ,     0x58000000      /* APB Bridge 0 Non-buffered Base Address	 */
.equ	APB1_BUF_BASE   ,     0x4C000000      /* APB Bridge 1 Buffered Base Address		 */
.equ	APB1_NBUF_BASE  ,     0x5C000000      /* APB Bridge 1 Non-buffered Base Address	 */

# ETM Definitions
.equ	IOPORT2_ETM_ENABLE_BASE ,     0x5C00204C
.equ	IOPORT6_ETM_ENABLE_BASE ,     0x5C00205C

.equ	IOPORT2_ETM_ENABLE_VAL  ,     0x0000FFFF
.equ	IOPORT6_ETM_ENABLE_VAL  ,     0x0000FFFF

#*************************************************************************
# Stack definitions
#*************************************************************************

        .equ    UND_Stack_Size,     1*4
        .equ    SVC_Stack_Size,    32*4
        .equ    ABT_Stack_Size,     1*4
        .equ    FIQ_Stack_Size,    32*4
        .equ    IRQ_Stack_Size,    512*4
        .equ    USR_Stack_Size,    512*4
        .equ    Top_Stack,     RAM_Base + RAM_Size

# NOTE: Startup Code must be linked first at Address at which it expects to run.

#*************************************************************************
# STARTUP EXECUTABLE CODE
#*************************************************************************

        .text
		.arm
		.extern main
		.global _app_entry
        .global start_up
		

        .func   start_up
#start_up:

ENTRY:

#*************************************************************************
# Exception Vectors
#*************************************************************************
Vectors:
        LDR     PC, Reset_Addr 		/* 0x0000 */        
        LDR     PC, Undef_Addr 		/* 0x0004 */        
        LDR     PC, SWI_Addr 		/* 0x0008 */        
        LDR     PC, PAbt_Addr 		/* 0x000C */        
        LDR     PC, DAbt_Addr 		/* 0x0010 */        
        NOP 						/* 0x0014 Reserved Vector */
		LDR     PC, IRQ_Addr  		/* 0x0018 wraps around address space to 0xFFFFFF030. Vector from VicVECAddr */
        LDR     PC, FIQ_Addr		/* 0x001C FIQ has no VIC vector slot!	*/

#*************************************************************************
# Interrupt Vectors
#*************************************************************************

Reset_Addr:     .word   Hard_Reset			  /* CPU reset vector and entry point */
Undef_Addr:     .word   _Undef_Handler
SWI_Addr:       .word   _SWI_Handler
PAbt_Addr:      .word   _PAbt_Handler
DAbt_Addr:      .word   _DAbt_Handler
                .word   0                      /* Reserved Address */
IRQ_Addr:       .word   _IRQ_Handler			   /* Does not get used due to "LDR PC, [PC, #-0xFF0]" above */
FIQ_Addr:       .word   _FIQ_Handler			   

# Dummy Interrupt Vector Table (real service routines in INTERRUPT.C)

	_Undef_Handler:  B       UndefinedHandler
	_SWI_Handler:    B       SWIHandler
	_PAbt_Handler:   B       PrefetchHandler
	_DAbt_Handler:   B       AbortHandler
	_IRQ_Handler:	 B		IRQHandler 		
	_FIQ_Handler:    B       FIQHandler



/*******************************************************************************
                       Exception Handlers
*******************************************************************************/

/*******************************************************************************
* Macro Name     : SaveContext
* Description    : This macro used to save the context before entering
                   an exception handler.
* Input          : The range of registers to store.
* Output         : none
*******************************************************************************/

.macro	SaveContext reg1 reg2
		STMFD	sp!,{\reg1-\reg2,lr}	/* Save The workspace plus the current return */
										/* address lr_ mode into the stack */
		MRS		r1, spsr				/* Save the spsr_mode into r1 */
		STMFD	sp!, {r1}				/* Save spsr */
.endm

/*******************************************************************************
* Macro Name     : RestoreContext
* Description    : This macro used to restore the context to return from
                   an exception handler and continue the program execution.
* Input          : The range of registers to restore.
* Output         : none
*******************************************************************************/

.macro	RestoreContext reg1 reg2
		LDMFD	sp!, {r1}				/* Restore the saved spsr_mode into r1 */
		MSR		spsr_cxsf, r1			/* Restore spsr_mode */
		LDMFD	sp!, {\reg1-\reg2,pc}^	/* Return to the instruction following */
										/* the exception interrupt */
.endm

/*******************************************************************************
* Function Name  : IRQHandler
* Description    : This function called when IRQ exception is entered.
* Input          : none
* Output         : none
*******************************************************************************/
IRQHandler:
       SUB    lr, lr, #4				/* Update the link register */
       SaveContext r0, r12				/* Save the workspace plus the current */
										/* return address lr_irq and spsr_irq */
		LDR    r0, =VectorAddress
		LDR    r0, [r0]					/* Read the routine address of VIC0 */
		LDR    r1, =VectorAddressDaisy
		LDR    r1, [r1]				    /* Read the routine address of VIC1 */
		/* Padding between the acknowledge and re-enable of interrupts */
		/* For more details, please refer to the following URL */
		/* http://www.arm.com/support/faqip/3682.html */
		NOP
		NOP
#		MSR		cpsr_c, #Mode_SYS		/* Switch to SYS mode and enable IRQ */
#		STMFD	sp!, {lr}				/* Save the link register. */
		LDR		lr, =ReturnAddress		/* Read the return address. */
		CMP		r0, #0					/* Is VIC0 VAR zero? */
		BEQ		SkipVic0
		BX		r0						/* Branch to the IRQ handler. */
SkipVic0:
		CMP		r1, #0					/* Is VIC1 VAR zero? */
		BEQ		ReturnAddress
		BX		r1						/* Branch to the IRQ handler. */
ReturnAddress:
#		LDMFD	sp!, {lr}				/* Restore the link register. */
#		MSR		cpsr_c, #Mode_IRQ|I_BIT	/* Switch to IRQ mode and disable IRQ */
		LDR		r0, =VectorAddress		/* Write to the VectorAddress to clear the */
		STR		r0, [r0]				/* respective interrupt in the internal interrupt */
		LDR		r1, =VectorAddressDaisy	/* Write to the VectorAddressDaisy to clear the */
		STR		r1, [r1]				/* respective interrupt in the internal interrupt */
		RestoreContext r0, r12			/* Restore the context and return to the program execution. */

/*******************************************************************************
* Function Name  : SWIHandler
* Description    : This function called when SWI instruction executed.
* Input          : none
* Output         : none
*******************************************************************************/

SWIHandler:
		SaveContext r0, r12			/* r0 holds swi number */
		MOV 	r1, sp				/* load regs */
		BL		SWI_Handler
		RestoreContext r0, r12

/*******************************************************************************
* Function Name  : UndefinedHandler
* Description    : This function called when undefined instruction
                   exception is entered.
* Input          : none
* Output         : none
*******************************************************************************/

UndefinedHandler:
		SaveContext r0, r12
		BL		Undefined_Handler
		RestoreContext r0, r12

/*******************************************************************************
* Function Name  : PrefetchAbortHandler
* Description    : This function called when Prefetch Abort
                   exception is entered.
* Input          : none
* Output         : none
*******************************************************************************/

PrefetchHandler:
		SUB		lr, lr, #4			/* Update the link register. */
		SaveContext r0, r12
		BL		Prefetch_Handler
		RestoreContext r0, r12

/*******************************************************************************
* Function Name  : DataAbortHandler
* Description    : This function is called when Data Abort
                   exception is entered.
* Input          : none
* Output         : none
*******************************************************************************/

AbortHandler:
		SUB		lr, lr, #8			/* Update the link register. */
		SaveContext r0, r12
		BL		Abort_Handler
		RestoreContext r0, r12

/*******************************************************************************
* Function Name  : FIQHandler
* Description    : This function is called when FIQ
                   exception is entered.
* Input          : none
* Output         : none
*******************************************************************************/

FIQHandler:
		SUB		lr, lr, #4			/* Update the link register. */
		SaveContext r0, r7
		BL		FIQ_Handler
		RestoreContext r0, r7

	


#*************************************************************************
# Reset Handler Entry Point
#*************************************************************************
Hard_Reset: 
app_entry:
 		 StartupDelay 500000
Start_init_s:

#*************************************************************************
# Setup SRAM Size

                .IF      SRAM_SETUP == 1

                LDR     R0, =SCU_BASE
                LDR     R1, =SCU_SCR0_Val
                STR     R1, [R0, #SCU_SCR0_OFS]
 #               ORR     R1, R1, #0x00000200
 #               STR     R1, [R0, #SCU_SCR0_OFS]

                .ENDIF

#*************************************************************************
# Setup Flash Memory Interface (FMI)

                .IF      FMI_SETUP == 1

                LDR     R0, =FMI_BASE
                LDR     R1, =FMI_BBSR_Val
                STR     R1, [R0, #FMI_BBSR_OFS]
                LDR     R1, =FMI_NBBSR_Val
                STR     R1, [R0, #FMI_NBBSR_OFS]
                LDR     R1, =(FMI_BBADR_Val >> 2)
                STR     R1, [R0, #FMI_BBADR_OFS]
                LDR     R1, =(FMI_NBBADR_Val >> 2)
                STR     R1, [R0, #FMI_NBBADR_OFS]
                LDR     R2, =FMI_CR_Val
                STR     R2, [R0, #FMI_CR_OFS]

    			LDR     R2, =FMI_SR_Val
                STR     R2, [R0, #FMI_SR_OFS]

	            # Write "Write flash configuration" command (60h)
                MOV     R0, R1, LSL #2
                MOV     R1, #0x60
                STRH    R1, [R0, #0]

                # Write "Write flash configuration confirm" command (03h)
                LDR     R2, =(FLASH_CFG_Val >> 2)
                ADD     R0, R0, R2
                MOV     R1, #0x03
                STRH    R1, [R0, #0]

                .ENDIF

#*************************************************************************
# Setup Clock PLL

                .IF      CLOCK_SETUP == 1

                LDR     R0, =SCU_BASE
                LDR     R1, =0x00020002
                STR     R1, [R0, #SCU_CLKCNTR_OFS]    /* Select OSC as clock src */

                NOP     /* Wait for oscillator stabilisation */
                NOP     /* Must be more than 10 oscillator periods */
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
                NOP
/*
                LDR     R1, =0x0003C019               // Disable PLL 
                STR     R1, [R0, #SCU_PLLCONF_OFS]
                LDR     R1, =SCU_PLLCONF_Val 
                STR     R1, [R0, #SCU_PLLCONF_OFS]    // Set new PLL values 

				.IF      (SCU_PLLCONF_Val & 0x8000)	  // See if PLL is being used 

                LDR     R1, =SCU_SYSSTAT_LOCK
PLL_LOCK_LOOP:
				LDR		R2,[R0, #SCU_SYSTAT_OFS]      // Wait for PLL lock 
				ANDS	R2, R2, R1
				BEQ		PLL_LOCK_LOOP

				.ENDIF

                LDR     R1, =SCU_PLLCONF_Val
                STR     R1, [R0, #SCU_PLLCONF_OFS]
                LDR     R1, =SCU_CLKCNTR_Val
                STR     R1, [R0, #SCU_CLKCNTR_OFS]

                LDR     R1, =SCU_PCGR0_Val            // Enable clock gating 
                STR     R1, [R0, #SCU_PCGR0_OFS]
                LDR     R1, =SCU_PCGR1_Val
                STR     R1, [R0, #SCU_PCGR1_OFS]
                .ENDIF
  */
  /*; --- wait states Flash confguration */

        LDR     R6, = 0x00080000            /*;Write a Write Flash Configuration */
        LDR     R7, =0x60                   /*;Register command (60h) to any word*/
        STRH    R7, [R6]                    /*;address in Bank 1.*/
 
 
        LDR     R6, = 0x00083040            /*;Write a Write Flash Configuration  */
        LDR     R7, = 0x3                   /*;Register Confirm command (03h)*/
        STRH    R7, [R6]                    /*;2Wstaites in read,PWD,LVD enabled, */
                                            /*;High BUSCFG.*/

/*; --- PLL configuration      */
    

        LDR     R1, = SCU_PLLCONF_Val               /*;Set PLL ENABLE, to 96Mhz */
        STR     R1, [R0, #SCU_PLLCONF_OFS]
        
       
Wait_Loop:      
 
        LDR     R1,[R0, #SCU_SYSTAT_OFS]   /*;Wait until PLL is Locked*/
		  ANDS    R1, R1, #0x01 
		  BEQ     Wait_Loop
        
        
        LDR     R1, = 0x00020080             /*;Set PLL as clock source after pll */
        STR     R1, [R0, #SCU_CLKCNTR_OFS ] /*;is locked and  FMICLK=RCLK,*/
        .ENDIF                                     /*;PCLK=RCLK/2*/

# Setup Peripheral Reset
                .IF      P_RESET_SETUP == 1
                LDR     R1, =SCU_PRR0_Val
                STR     R1, [R0, #SCU_PRR0_OFS]
                LDR     R1, =SCU_PRR1_Val
                STR     R1, [R0, #SCU_PRR1_OFS]
                .ENDIF

#*************************************************************************
# Embedded Trace Module Setup
#*************************************************************************

                .IF     ETM_SETUP == 1

# Configure IOPORT2 for ETM operation
                LDR     R0, =IOPORT2_ETM_ENABLE_BASE
                LDR     R1, =IOPORT2_ETM_ENABLE_VAL
                STR     R1, [R0, #0]  

# Configure IOPORT6 for ETM operation				
                LDR     R0, =IOPORT6_ETM_ENABLE_BASE
                LDR     R1, =IOPORT6_ETM_ENABLE_VAL
                STR     R1, [R0, #0]  
			  
				.ENDIF


#*************************************************************************
# Compiler Runtime Environment Setup
#*************************************************************************
# Note: R13 = SP

# Setup Stack for each mode
	   			LDR     R0, =Top_Stack

# Set up Fast Interrupt Mode and set FIQ Mode Stack
        		MSR     CPSR_c, #Mode_FIQ|I_BIT|F_BIT
    		    mov     r13, r0                     
    		    sub     r0, r0, #FIQ_Stack_Size

# Set up Interrupt Mode and set IRQ Mode Stack
    		    msr     CPSR_c, #Mode_IRQ|I_BIT|F_BIT
    			mov     r13, r0                     
     		   	sub     r0, r0, #IRQ_Stack_Size

# Set up Abort Mode and set Abort Mode Stack
      			msr     CPSR_c, #Mode_ABT|I_BIT|F_BIT
        		mov     r13, r0                     
        		sub     r0, r0, #ABT_Stack_Size

# Set up Undefined Instruction Mode and set Undef Mode Stack
        		msr     CPSR_c, #Mode_UND|I_BIT|F_BIT
        		mov     r13, r0                     
        		sub     r0, r0, #UND_Stack_Size

# 	Set up Supervisor Mode and set Supervisor Mode Stack
        		msr     CPSR_c, #Mode_SVC|I_BIT|F_BIT
        		mov     r13, r0                     
        		sub     r0, r0, #SVC_Stack_Size

# 	Set up User Mode and set User Mode Stack
        		msr     CPSR_c, #Mode_USR /* #Mode_USR */  /* Leave interrupts enabled in user mode                 */
        		mov     r13, r0             /* Note: interrupts will not happen until VIC is enabled */         

#  Setup a default Stack Limit (when compiled with "-mapcs-stack-check")
        		SUB     SL, SP, #1<<10         /* 1kB */

# Initialise current CPU status to prevent unwanted interrupts
#				msr		CPSR_c,#0xD3

#*************************************************************************
# Initialise RAM For Compiler Variables
#*************************************************************************

            	copy_section2 data, _etext, __data_start__, _edata

#*************************************************************************
# Clear .bss section
#*************************************************************************

                clear_section bss, __bss_start__, __bss_end__
                clear_section bss2, __bss2_start__, __bss2_end__

#*************************************************************************
# Enter the C code
#*************************************************************************
# Jump to main()

	    B       main


        .size   _startup, . - _startup
        .endfunc

#*************************************************************************
# END
#*************************************************************************	
        .end
