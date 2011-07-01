
;CodeVisionAVR C Compiler V2.05.0 Standard
;(C) Copyright 1998-2010 Pavel Haiduc, HP InfoTech s.r.l.
;http://www.hpinfotech.com

;Chip type                : ATxmega128A1
;Program type             : Application
;Clock frequency          : 32,000000 MHz
;Memory model             : Small
;Optimize for             : Size
;(s)printf features       : float, width, precision
;(s)scanf features        : int, width
;External RAM size        : 0
;Data Stack size          : 1024 byte(s)
;Heap size                : 0 byte(s)
;Promote 'char' to 'int'  : Yes
;'char' is unsigned       : Yes
;8 bit enums              : Yes
;global 'const' stored in FLASH: Yes
;Enhanced core instructions    : On
;Smart register allocation     : On
;Automatic register allocation : On

	#pragma AVRPART ADMIN PART_NAME ATxmega128A1
	#pragma AVRPART MEMORY PROG_FLASH 135168
	#pragma AVRPART MEMORY EEPROM 2048
	#pragma AVRPART MEMORY INT_SRAM SIZE 16383
	#pragma AVRPART MEMORY INT_SRAM START_ADDR 0x2000

	#define CALL_SUPPORTED 1

	.LISTMAC
	.EQU CCP=0x34
	.EQU RAMPD=0x38
	.EQU RAMPX=0x39
	.EQU RAMPY=0x3A
	.EQU RAMPZ=0x3B
	.EQU EIND=0x3C
	.EQU WDT_CTRL=0x80
	.EQU PMIC_CTRL=0xA2
	.EQU NVM_ADDR0=0X01C0
	.EQU NVM_ADDR1=NVM_ADDR0+1
	.EQU NVM_ADDR2=NVM_ADDR1+1
	.EQU NVM_DATA0=NVM_ADDR0+4
	.EQU NVM_CMD=NVM_ADDR0+0xA
	.EQU NVM_CTRLA=NVM_ADDR0+0xB
	.EQU NVM_CTRLB=NVM_ADDR0+0xC
	.EQU NVM_STATUS=NVM_ADDR0+0xF
	.EQU PORTCFG_MPCMASK=0xB0
	.EQU PORTH_DIR=0x6E0
	.EQU PORTH_OUT=0x6E4
	.EQU PORTH_PIN0CTRL=0x6F0
	.EQU PORTJ_DIR=0x700
	.EQU PORTJ_OUT=0x704
	.EQU PORTJ_PIN0CTRL=0x710
	.EQU PORTK_DIR=0x720
	.EQU PORTK_OUT=0x724
	.EQU PORTK_PIN0CTRL=0x730
	.EQU EBI_CTRL=0x0440
	.EQU EBI_SDRAMCTRLA=EBI_CTRL+1
	.EQU EBI_REFRESHL=EBI_CTRL+4
	.EQU EBI_REFRESHH=EBI_CTRL+5
	.EQU EBI_INITDLYL=EBI_CTRL+6
	.EQU EBI_INITDLYH=EBI_CTRL+7
	.EQU EBI_SDRAMCTRLB=EBI_CTRL+8
	.EQU EBI_SDRAMCTRLC=EBI_CTRL+9
	.EQU EBI_CS0_CTRLA=EBI_CTRL+0x10
	.EQU EBI_CS0_CTRLB=EBI_CS0_CTRLA+1
	.EQU EBI_CS0_BASEADDRL=EBI_CS0_CTRLA+2
	.EQU EBI_CS0_BASEADDRH=EBI_CS0_CTRLA+3
	.EQU EBI_CS1_CTRLA=EBI_CTRL+0x14
	.EQU EBI_CS1_CTRLB=EBI_CS1_CTRLA+1
	.EQU EBI_CS1_BASEADDRL=EBI_CS1_CTRLA+2
	.EQU EBI_CS1_BASEADDRH=EBI_CS1_CTRLA+3
	.EQU EBI_CS2_CTRLA=EBI_CTRL+0x18
	.EQU EBI_CS2_CTRLB=EBI_CS2_CTRLA+1
	.EQU EBI_CS2_BASEADDRL=EBI_CS2_CTRLA+2
	.EQU EBI_CS2_BASEADDRH=EBI_CS2_CTRLA+3
	.EQU EBI_CS3_CTRLA=EBI_CTRL+0x1C
	.EQU EBI_CS3_CTRLB=EBI_CS3_CTRLA+1
	.EQU EBI_CS3_BASEADDRL=EBI_CS3_CTRLA+2
	.EQU EBI_CS3_BASEADDRH=EBI_CS3_CTRLA+3
	.EQU EBI_SDCAS_bp=3
	.EQU EBI_SDCAS_bm=8
	.EQU EBI_SDROW_bp=2
	.EQU EBI_SDROW_bm=4
	.EQU EBI_SDCOL_bp=0
	.EQU EBI_MRDLY_bp=6
	.EQU EBI_ROWCYCDLY_bp=3
	.EQU EBI_RPDLY_bp=0
	.EQU EBI_WRDLY_bp=6
	.EQU EBI_ESRDLY_bp=3
	.EQU EBI_ROWCOLDLY_bp=0
	.EQU EBI_CS_SDSREN_bm=4
	.EQU EBI_CS_ASPACE_256B_gc=0x00<<2
	.EQU EBI_CS_ASPACE_512B_gc=0x01<<2
	.EQU EBI_CS_ASPACE_1KB_gc=0x02<<2
	.EQU EBI_CS_ASPACE_2KB_gc=0x03<<2
	.EQU EBI_CS_ASPACE_4KB_gc=0x04<<2
	.EQU EBI_CS_ASPACE_8KB_gc=0x05<<2
	.EQU EBI_CS_ASPACE_16KB_gc=0x06<<2
	.EQU EBI_CS_ASPACE_32KB_gc=0x07<<2
	.EQU EBI_CS_ASPACE_64KB_gc=0x08<<2
	.EQU EBI_CS_ASPACE_128KB_gc=0x09<<2
	.EQU EBI_CS_ASPACE_256KB_gc=0x0A<<2
	.EQU EBI_CS_ASPACE_512KB_gc=0x0B<<2
	.EQU EBI_CS_ASPACE_1MB_gc=0x0C<<2
	.EQU EBI_CS_ASPACE_2MB_gc=0x0D<<2
	.EQU EBI_CS_ASPACE_4MB_gc=0x0E<<2
	.EQU EBI_CS_ASPACE_8MB_gc=0x0F<<2
	.EQU EBI_CS_ASPACE_16M_gc=0x10<<2
	.EQU EBI_CS_MODE_DISABLED_gc=0x00
	.EQU EBI_CS_MODE_SRAM_gc=0x01
	.EQU EBI_CS_MODE_LPC_gc=0x02
	.EQU EBI_CS_MODE_SDRAM_gc=0x03
	.EQU EBI_SDDATAW_4BIT_gc=0x00<<6
	.EQU EBI_SDDATAW_8BIT_gc=0x01<<6
	.EQU EBI_LPCMODE_ALE1_gc=0x00<<4
	.EQU EBI_LPCMODE_ALE12_gc=0x02<<4
	.EQU EBI_SRMODE_ALE1_gc=0x00<<2
	.EQU EBI_SRMODE_ALE2_gc=0x01<<2
	.EQU EBI_SRMODE_ALE12_gc=0x02<<2
	.EQU EBI_SRMODE_NOALE_gc=0x03<<2
	.EQU EBI_IFMODE_DISABLED_gc=0x00
	.EQU EBI_IFMODE_3PORT_gc=0x01
	.EQU EBI_IFMODE_4PORT_gc=0x02
	.EQU EBI_IFMODE_2PORT_gc=0x03
	.EQU EBI_SDCOL_8BIT_gc=0x00
	.EQU EBI_SDCOL_9BIT_gc=0x01
	.EQU EBI_SDCOL_10BIT_gc=0x02
	.EQU EBI_SDCOL_11BIT_gc=0x03
	.EQU SPL=0x3D
	.EQU SPH=0x3E
	.EQU SREG=0x3F
	.EQU GPIO0=0x00
	.EQU GPIO1=0x01
	.EQU GPIO2=0x02
	.EQU GPIO3=0x03
	.EQU GPIO4=0x04
	.EQU GPIO5=0x05
	.EQU GPIO6=0x06
	.EQU GPIO7=0x07
	.EQU GPIO8=0x08
	.EQU GPIO9=0x09
	.EQU GPIO10=0x0A
	.EQU GPIO11=0x0B
	.EQU GPIO12=0x0C
	.EQU GPIO13=0x0D
	.EQU GPIO14=0x0E
	.EQU GPIO15=0x0F

	.DEF R0X0=R0
	.DEF R0X1=R1
	.DEF R0X2=R2
	.DEF R0X3=R3
	.DEF R0X4=R4
	.DEF R0X5=R5
	.DEF R0X6=R6
	.DEF R0X7=R7
	.DEF R0X8=R8
	.DEF R0X9=R9
	.DEF R0XA=R10
	.DEF R0XB=R11
	.DEF R0XC=R12
	.DEF R0XD=R13
	.DEF R0XE=R14
	.DEF R0XF=R15
	.DEF R0X10=R16
	.DEF R0X11=R17
	.DEF R0X12=R18
	.DEF R0X13=R19
	.DEF R0X14=R20
	.DEF R0X15=R21
	.DEF R0X16=R22
	.DEF R0X17=R23
	.DEF R0X18=R24
	.DEF R0X19=R25
	.DEF R0X1A=R26
	.DEF R0X1B=R27
	.DEF R0X1C=R28
	.DEF R0X1D=R29
	.DEF R0X1E=R30
	.DEF R0X1F=R31

	.EQU __SRAM_START=0x2000
	.EQU __SRAM_END=0x3FFF
	.EQU __DSTACK_SIZE=0x0400
	.EQU __HEAP_SIZE=0x0000
	.EQU __CLEAR_SRAM_SIZE=__SRAM_END-__SRAM_START+1

	.MACRO __CPD1N
	CPI  R30,LOW(@0)
	LDI  R26,HIGH(@0)
	CPC  R31,R26
	LDI  R26,BYTE3(@0)
	CPC  R22,R26
	LDI  R26,BYTE4(@0)
	CPC  R23,R26
	.ENDM

	.MACRO __CPD2N
	CPI  R26,LOW(@0)
	LDI  R30,HIGH(@0)
	CPC  R27,R30
	LDI  R30,BYTE3(@0)
	CPC  R24,R30
	LDI  R30,BYTE4(@0)
	CPC  R25,R30
	.ENDM

	.MACRO __CPWRR
	CP   R@0,R@2
	CPC  R@1,R@3
	.ENDM

	.MACRO __CPWRN
	CPI  R@0,LOW(@2)
	LDI  R30,HIGH(@2)
	CPC  R@1,R30
	.ENDM

	.MACRO __ADDB1MN
	SUBI R30,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDB2MN
	SUBI R26,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDW1MN
	SUBI R30,LOW(-@0-(@1))
	SBCI R31,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW2MN
	SUBI R26,LOW(-@0-(@1))
	SBCI R27,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	SBCI R22,BYTE3(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1N
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	SBCI R22,BYTE3(-@0)
	SBCI R23,BYTE4(-@0)
	.ENDM

	.MACRO __ADDD2N
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	SBCI R24,BYTE3(-@0)
	SBCI R25,BYTE4(-@0)
	.ENDM

	.MACRO __SUBD1N
	SUBI R30,LOW(@0)
	SBCI R31,HIGH(@0)
	SBCI R22,BYTE3(@0)
	SBCI R23,BYTE4(@0)
	.ENDM

	.MACRO __SUBD2N
	SUBI R26,LOW(@0)
	SBCI R27,HIGH(@0)
	SBCI R24,BYTE3(@0)
	SBCI R25,BYTE4(@0)
	.ENDM

	.MACRO __ANDBMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ANDWMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ANDI R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ANDD1N
	ANDI R30,LOW(@0)
	ANDI R31,HIGH(@0)
	ANDI R22,BYTE3(@0)
	ANDI R23,BYTE4(@0)
	.ENDM

	.MACRO __ANDD2N
	ANDI R26,LOW(@0)
	ANDI R27,HIGH(@0)
	ANDI R24,BYTE3(@0)
	ANDI R25,BYTE4(@0)
	.ENDM

	.MACRO __ORBMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ORWMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ORI  R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ORD1N
	ORI  R30,LOW(@0)
	ORI  R31,HIGH(@0)
	ORI  R22,BYTE3(@0)
	ORI  R23,BYTE4(@0)
	.ENDM

	.MACRO __ORD2N
	ORI  R26,LOW(@0)
	ORI  R27,HIGH(@0)
	ORI  R24,BYTE3(@0)
	ORI  R25,BYTE4(@0)
	.ENDM

	.MACRO __DELAY_USB
	LDI  R24,LOW(@0)
__DELAY_USB_LOOP:
	DEC  R24
	BRNE __DELAY_USB_LOOP
	.ENDM

	.MACRO __DELAY_USW
	LDI  R24,LOW(@0)
	LDI  R25,HIGH(@0)
__DELAY_USW_LOOP:
	SBIW R24,1
	BRNE __DELAY_USW_LOOP
	.ENDM

	.MACRO __GETD1S
	LDD  R30,Y+@0
	LDD  R31,Y+@0+1
	LDD  R22,Y+@0+2
	LDD  R23,Y+@0+3
	.ENDM

	.MACRO __GETD2S
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	LDD  R24,Y+@0+2
	LDD  R25,Y+@0+3
	.ENDM

	.MACRO __PUTD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R31
	STD  Y+@0+2,R22
	STD  Y+@0+3,R23
	.ENDM

	.MACRO __PUTD2S
	STD  Y+@0,R26
	STD  Y+@0+1,R27
	STD  Y+@0+2,R24
	STD  Y+@0+3,R25
	.ENDM

	.MACRO __PUTDZ2
	STD  Z+@0,R26
	STD  Z+@0+1,R27
	STD  Z+@0+2,R24
	STD  Z+@0+3,R25
	.ENDM

	.MACRO __CLRD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R30
	STD  Y+@0+2,R30
	STD  Y+@0+3,R30
	.ENDM

	.MACRO __POINTB1MN
	LDI  R30,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW1MN
	LDI  R30,LOW(@0+(@1))
	LDI  R31,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTD1M
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __POINTW1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	LDI  R22,BYTE3(2*@0+(@1))
	LDI  R23,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTB2MN
	LDI  R26,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW2MN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTBRM
	LDI  R@0,LOW(@1)
	.ENDM

	.MACRO __POINTWRM
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __POINTBRMN
	LDI  R@0,LOW(@1+(@2))
	.ENDM

	.MACRO __POINTWRMN
	LDI  R@0,LOW(@2+(@3))
	LDI  R@1,HIGH(@2+(@3))
	.ENDM

	.MACRO __POINTWRFN
	LDI  R@0,LOW(@2*2+(@3))
	LDI  R@1,HIGH(@2*2+(@3))
	.ENDM

	.MACRO __GETD1N
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __GETD2N
	LDI  R26,LOW(@0)
	LDI  R27,HIGH(@0)
	LDI  R24,BYTE3(@0)
	LDI  R25,BYTE4(@0)
	.ENDM

	.MACRO __GETB1MN
	LDS  R30,@0+(@1)
	.ENDM

	.MACRO __GETB1HMN
	LDS  R31,@0+(@1)
	.ENDM

	.MACRO __GETW1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	.ENDM

	.MACRO __GETD1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	LDS  R22,@0+(@1)+2
	LDS  R23,@0+(@1)+3
	.ENDM

	.MACRO __GETBRMN
	LDS  R@0,@1+(@2)
	.ENDM

	.MACRO __GETWRMN
	LDS  R@0,@2+(@3)
	LDS  R@1,@2+(@3)+1
	.ENDM

	.MACRO __GETWRZ
	LDD  R@0,Z+@2
	LDD  R@1,Z+@2+1
	.ENDM

	.MACRO __GETD2Z
	LDD  R26,Z+@0
	LDD  R27,Z+@0+1
	LDD  R24,Z+@0+2
	LDD  R25,Z+@0+3
	.ENDM

	.MACRO __GETB2MN
	LDS  R26,@0+(@1)
	.ENDM

	.MACRO __GETW2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	.ENDM

	.MACRO __GETD2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	LDS  R24,@0+(@1)+2
	LDS  R25,@0+(@1)+3
	.ENDM

	.MACRO __PUTB1MN
	STS  @0+(@1),R30
	.ENDM

	.MACRO __PUTW1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	.ENDM

	.MACRO __PUTD1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	STS  @0+(@1)+2,R22
	STS  @0+(@1)+3,R23
	.ENDM

	.MACRO __PUTB1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMWRB
	.ENDM

	.MACRO __PUTW1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMWRW
	.ENDM

	.MACRO __PUTD1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMWRD
	.ENDM

	.MACRO __PUTBR0MN
	STS  @0+(@1),R0
	.ENDM

	.MACRO __PUTBMRN
	STS  @0+(@1),R@2
	.ENDM

	.MACRO __PUTWMRN
	STS  @0+(@1),R@2
	STS  @0+(@1)+1,R@3
	.ENDM

	.MACRO __PUTBZR
	STD  Z+@1,R@0
	.ENDM

	.MACRO __PUTWZR
	STD  Z+@2,R@0
	STD  Z+@2+1,R@1
	.ENDM

	.MACRO __GETW1R
	MOV  R30,R@0
	MOV  R31,R@1
	.ENDM

	.MACRO __GETW2R
	MOV  R26,R@0
	MOV  R27,R@1
	.ENDM

	.MACRO __GETWRN
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __PUTW1R
	MOV  R@0,R30
	MOV  R@1,R31
	.ENDM

	.MACRO __PUTW2R
	MOV  R@0,R26
	MOV  R@1,R27
	.ENDM

	.MACRO __ADDWRN
	SUBI R@0,LOW(-@2)
	SBCI R@1,HIGH(-@2)
	.ENDM

	.MACRO __ADDWRR
	ADD  R@0,R@2
	ADC  R@1,R@3
	.ENDM

	.MACRO __SUBWRN
	SUBI R@0,LOW(@2)
	SBCI R@1,HIGH(@2)
	.ENDM

	.MACRO __SUBWRR
	SUB  R@0,R@2
	SBC  R@1,R@3
	.ENDM

	.MACRO __ANDWRN
	ANDI R@0,LOW(@2)
	ANDI R@1,HIGH(@2)
	.ENDM

	.MACRO __ANDWRR
	AND  R@0,R@2
	AND  R@1,R@3
	.ENDM

	.MACRO __ORWRN
	ORI  R@0,LOW(@2)
	ORI  R@1,HIGH(@2)
	.ENDM

	.MACRO __ORWRR
	OR   R@0,R@2
	OR   R@1,R@3
	.ENDM

	.MACRO __EORWRR
	EOR  R@0,R@2
	EOR  R@1,R@3
	.ENDM

	.MACRO __GETWRS
	LDD  R@0,Y+@2
	LDD  R@1,Y+@2+1
	.ENDM

	.MACRO __PUTBSR
	STD  Y+@1,R@0
	.ENDM

	.MACRO __PUTWSR
	STD  Y+@2,R@0
	STD  Y+@2+1,R@1
	.ENDM

	.MACRO __MOVEWRR
	MOV  R@0,R@2
	MOV  R@1,R@3
	.ENDM

	.MACRO __INWR
	IN   R@0,@2
	IN   R@1,@2+1
	.ENDM

	.MACRO __OUTWR
	OUT  @2+1,R@1
	OUT  @2,R@0
	.ENDM

	.MACRO __CALL1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	ICALL
	.ENDM

	.MACRO __CALL1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	CALL __GETW1PF
	ICALL
	.ENDM

	.MACRO __CALL2EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	CALL __EEPROMRDW
	ICALL
	.ENDM

	.MACRO __GETW1STACK
	IN   R26,SPL
	IN   R27,SPH
	ADIW R26,@0+1
	LD   R30,X+
	LD   R31,X
	.ENDM

	.MACRO __GETD1STACK
	IN   R26,SPL
	IN   R27,SPH
	ADIW R26,@0+1
	LD   R30,X+
	LD   R31,X+
	LD   R22,X
	.ENDM

	.MACRO __NBST
	BST  R@0,@1
	IN   R30,SREG
	LDI  R31,0x40
	EOR  R30,R31
	OUT  SREG,R30
	.ENDM


	.MACRO __PUTB1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RNS
	MOVW R26,R@0
	ADIW R26,@1
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	CALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	CALL __PUTDP1
	.ENDM


	.MACRO __GETB1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R30,Z
	.ENDM

	.MACRO __GETB1HSX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	.ENDM

	.MACRO __GETW1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z+
	LD   R23,Z
	MOVW R30,R0
	.ENDM

	.MACRO __GETB2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R26,X
	.ENDM

	.MACRO __GETW2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	.ENDM

	.MACRO __GETD2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R1,X+
	LD   R24,X+
	LD   R25,X
	MOVW R26,R0
	.ENDM

	.MACRO __GETBRSX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	LD   R@0,Z
	.ENDM

	.MACRO __GETWRSX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	LD   R@0,Z+
	LD   R@1,Z
	.ENDM

	.MACRO __GETBRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	LD   R@0,X
	.ENDM

	.MACRO __GETWRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	LD   R@0,X+
	LD   R@1,X
	.ENDM

	.MACRO __LSLW8SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	CLR  R30
	.ENDM

	.MACRO __PUTB1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __CLRW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __CLRD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R30
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __PUTB2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z,R26
	.ENDM

	.MACRO __PUTW2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z,R27
	.ENDM

	.MACRO __PUTD2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z+,R27
	ST   Z+,R24
	ST   Z,R25
	.ENDM

	.MACRO __PUTBSRX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	ST   Z,R@0
	.ENDM

	.MACRO __PUTWSRX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	ST   Z+,R@0
	ST   Z,R@1
	.ENDM

	.MACRO __PUTB1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __MULBRR
	MULS R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRRU
	MUL  R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRR0
	MULS R@0,R@1
	.ENDM

	.MACRO __MULBRRU0
	MUL  R@0,R@1
	.ENDM

	.MACRO __MULBNWRU
	LDI  R26,@2
	MUL  R26,R@0
	MOVW R30,R0
	MUL  R26,R@1
	ADD  R31,R0
	.ENDM

;NAME DEFINITIONS FOR GLOBAL VARIABLES ALLOCATED TO REGISTERS
	.DEF _dir=R3
	.DEF _NumOfRev=R4
	.DEF _Counter_OverFlows=R2
	.DEF _LED=R7
	.DEF _CountMode=R6
	.DEF _counter_state1=R9
	.DEF _counter_state2=R8
	.DEF _q_test_sig_Port=R10
	.DEF _test_lineCount=R13

;GPIO0-GPIO15 INITIALIZATION VALUES
	.EQU __GPIO0_INIT=0x7D
	.EQU __GPIO1_INIT=0x01
	.EQU __GPIO2_INIT=0x00
	.EQU __GPIO3_INIT=0x00
	.EQU __GPIO4_INIT=0x00
	.EQU __GPIO5_INIT=0x00
	.EQU __GPIO6_INIT=0x00
	.EQU __GPIO7_INIT=0x00
	.EQU __GPIO8_INIT=0x00
	.EQU __GPIO9_INIT=0x00
	.EQU __GPIO10_INIT=0x00
	.EQU __GPIO11_INIT=0x00
	.EQU __GPIO12_INIT=0x00
	.EQU __GPIO13_INIT=0x00
	.EQU __GPIO14_INIT=0x00
	.EQU __GPIO15_INIT=0x00

;GLOBAL REGISTER VARIABLES INITIALIZATION VALUES
	.EQU __R2_INIT=0x00
	.EQU __R3_INIT=0x00
	.EQU __R4_INIT=0x00
	.EQU __R5_INIT=0x00
	.EQU __R6_INIT=0x01
	.EQU __R7_INIT=0x00
	.EQU __R8_INIT=0x00
	.EQU __R9_INIT=0x00
	.EQU __R10_INIT=0x00
	.EQU __R11_INIT=0x00
	.EQU __R12_INIT=0x00
	.EQU __R13_INIT=0x00
	.EQU __R14_INIT=0x00

	.CSEG
	.ORG 0x00

;START OF CODE MARKER
__START_OF_CODE:

;INTERRUPT VECTORS
	JMP  __RESET
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  _handler_TCC0_ERR_vect
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  _handler_TCE0_CCA_vect
	JMP  _handler_TCE0_CCB_vect
	JMP  _handler_TCE0_CCC_vect
	JMP  _handler_TCE0_CCD_vect
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  _tcd0_overflow_isr
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  _portk_int0_isr
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00
	JMP  0x00

_0x3:
	.DB  0xA,0xD7,0x23,0x3C
_0x4:
	.DB  0x0,0x0,0x80,0x3F
_0x5:
	.DB  0x6F,0x12,0x83,0x3A
_0x6:
	.DB  0x0,0x0,0x80,0x3F
_0x7:
	.DB  0x6F,0x12,0x83,0x3A
_0x8:
	.DB  0x0,0x0,0x80,0x3E
_0x9:
	.DB  0xDB,0xF,0xC9,0x3F
_0x2000060:
	.DB  0x1
_0x2000000:
	.DB  0x2D,0x4E,0x41,0x4E,0x0,0x49,0x4E,0x46
	.DB  0x0
_0x202000B:
	.DB  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0
	.DB  0x0,0x0
_0x20A0000:
	.DB  0x2D,0x4E,0x41,0x4E,0x0

__GLOBAL_INI_TBL:
	.DW  0x04
	.DW  _h
	.DW  _0x3*2

	.DW  0x04
	.DW  _w_3
	.DW  _0x4*2

	.DW  0x04
	.DW  _w_4
	.DW  _0x5*2

	.DW  0x04
	.DW  _w_8
	.DW  _0x6*2

	.DW  0x04
	.DW  _w_9
	.DW  _0x7*2

	.DW  0x04
	.DW  _home_x
	.DW  _0x8*2

	.DW  0x04
	.DW  _home_theta
	.DW  _0x9*2

	.DW  0x01
	.DW  __seed_G100
	.DW  _0x2000060*2

_0xFFFFFFFF:
	.DW  0

__RESET:
	CLI
	CLR  R30
	OUT  RAMPD,R30
	OUT  RAMPX,R30
	OUT  RAMPY,R30

;MEMORY MAPPED EEPROM ACCESS IS USED
	LDS  R31,NVM_CTRLB
	ORI  R31,0x08
	STS  NVM_CTRLB,R31

;INTERRUPT VECTORS ARE PLACED
;AT THE START OF FLASH
	LDI  R31,0xD8
	OUT  CCP,R31
	STS  PMIC_CTRL,R30

;NO EXTERNAL RAM IS USED
	STS  EBI_CTRL,R30

;DISABLE WATCHDOG
	LDS  R26,WDT_CTRL
	CBR  R26,2
	SBR  R26,1
	OUT  CCP,R31
	STS  WDT_CTRL,R26

;CLEAR SRAM
	LDI  R24,LOW(__CLEAR_SRAM_SIZE)
	LDI  R25,HIGH(__CLEAR_SRAM_SIZE)
	LDI  R26,LOW(__SRAM_START)
	LDI  R27,HIGH(__SRAM_START)
__CLEAR_SRAM:
	ST   X+,R30
	SBIW R24,1
	BRNE __CLEAR_SRAM

;GLOBAL VARIABLES INITIALIZATION
	LDI  R30,LOW(__GLOBAL_INI_TBL*2)
	LDI  R31,HIGH(__GLOBAL_INI_TBL*2)
__GLOBAL_INI_NEXT:
	LPM  R24,Z+
	LPM  R25,Z+
	SBIW R24,0
	BREQ __GLOBAL_INI_END
	LPM  R26,Z+
	LPM  R27,Z+
	LPM  R0,Z+
	LPM  R1,Z+
	MOVW R22,R30
	MOVW R30,R0
__GLOBAL_INI_LOOP:
	LPM  R0,Z+
	ST   X+,R0
	SBIW R24,1
	BRNE __GLOBAL_INI_LOOP
	MOVW R30,R22
	RJMP __GLOBAL_INI_NEXT
__GLOBAL_INI_END:

	OUT  RAMPZ,R24

;GPIO0-GPIO15 INITIALIZATION
	LDI  R30,__GPIO0_INIT
	OUT  GPIO0,R30
	LDI  R30,__GPIO1_INIT
	OUT  GPIO1,R30
	LDI  R30,__GPIO2_INIT
	OUT  GPIO2,R30
	;__GPIO3_INIT = __GPIO2_INIT
	OUT  GPIO3,R30
	;__GPIO4_INIT = __GPIO2_INIT
	OUT  GPIO4,R30
	;__GPIO5_INIT = __GPIO2_INIT
	OUT  GPIO5,R30
	;__GPIO6_INIT = __GPIO2_INIT
	OUT  GPIO6,R30
	;__GPIO7_INIT = __GPIO2_INIT
	OUT  GPIO7,R30
	;__GPIO8_INIT = __GPIO2_INIT
	OUT  GPIO8,R30
	;__GPIO9_INIT = __GPIO2_INIT
	OUT  GPIO9,R30
	;__GPIO10_INIT = __GPIO2_INIT
	OUT  GPIO10,R30
	;__GPIO11_INIT = __GPIO2_INIT
	OUT  GPIO11,R30
	;__GPIO12_INIT = __GPIO2_INIT
	OUT  GPIO12,R30
	;__GPIO13_INIT = __GPIO2_INIT
	OUT  GPIO13,R30
	;__GPIO14_INIT = __GPIO2_INIT
	OUT  GPIO14,R30
	;__GPIO15_INIT = __GPIO2_INIT
	OUT  GPIO15,R30

;GLOBAL REGISTER VARIABLES INITIALIZATION
	LDI  R30,__R6_INIT
	MOV  R6,R30
	LDI  R30,__R10_INIT
	MOV  R10,R30
	;__R11_INIT = __R10_INIT
	MOV  R11,R30
	;__R13_INIT = __R10_INIT
	MOV  R13,R30

;HARDWARE STACK POINTER INITIALIZATION
	LDI  R30,LOW(__SRAM_END-__HEAP_SIZE)
	OUT  SPL,R30
	LDI  R30,HIGH(__SRAM_END-__HEAP_SIZE)
	OUT  SPH,R30

;DATA STACK POINTER INITIALIZATION
	LDI  R28,LOW(__SRAM_START+__DSTACK_SIZE)
	LDI  R29,HIGH(__SRAM_START+__DSTACK_SIZE)

	JMP  _main

	.ESEG
	.ORG 0

	.DSEG
	.ORG 0x2400

	.CSEG
;/* This file has been prepared for Doxygen automatic documentation generation. */
;/*! \file *********************************************************************
; *
; * \brief  Event Detector Project. This application program would read various kind of system's output. It can be
; * interfaced with at maximum 5 quadrature encoders, 5 analog signals from CT process sensors. It would also estimate
; * the full state of the system using available outputs. We would use first order low pass filters to estimate the speed
; * variables from output position states. In case of high noise we would use EKF filtering to estimate the speed variables.
; * The quadrature decoding would be done using dedicaded external HCTL2032 IC and also using built-in quadrature decoder of
; * ATxmega128A1. The program would sample the process @ 1 kHz and would compute the full state and then evaluate the event-generation
; * rule and then would generate the event/interrupt to trigger the communication of state vector to wireless mote on SPI channel and mote
; * would transmit it on the network.
; * This applicstion consists of four main modules. Module # 1: Sensor Interface Module, Module # 2: State Estimation Module,
; * Module # 3: Event Generation Module  Module #4: Controller
; * We have used one timer to read sensors periodically @ 1kHz and then we would estimate speeds (full state vector) and test the event-generation
; * rule. All four operations would be done inside timer TCDO overflow ISR. In case of event we would trigger the serial communication from inside the timer
; * overflow interrupt. When the mote would receive the serial data, it would immediately transmit it to the network!
; *
; * \par Application note:
; *      See NetCon Wiki Page
; *      HCTL2032 Datasheet
; *      ATxmega128A1 Datasheet
; *
; * \par Documentation
; *
; * \author
; *      Faisal Altaf: http://www.ee.kth.se
; *      Reglerteknik
; *      Support email: faltaf@kth.se
; *
; * $Revision: 0$
; * $Date: 2010-12-7 13:36 +0100 (on, 7 December 2010) $
; *
; * Copyright (c) 2010, Reglerteknik, KTH-Royal Institute of Technology. All rights reserved.
; *
; * Redistribution and use in source and binary forms, with or without
; * modification, are permitted provided that the following conditions are met:
; *
; * 1. Redistributions of source code must retain the above copyright notice,
; * this list of conditions and the following disclaimer.
; *
; * 2. Redistributions in binary form must reproduce the above copyright notice,
; * this list of conditions and the following disclaimer in the documentation
; * and/or other materials provided with the distribution.
; *
; * 3. The name of KTH may not be used to endorse or promote products derived
; * from this software without specific prior written permission.
;
; *****************************************************************************/
;
;
;/* we are using the one provided by codevisionavr instead of that provided by atmel.
;copied from \inc directory to project directory but modified it to include F_CPU macro
;which we copied from atmel's avr_compiler.h*/
;#include "avr_compiler.h"
;
;
;/* either copy the hctl_driver.lib, qdec_driver.lib and qdec_signal_generator.lib files from the  main project directory and put in .\lib directory
;inside cavr2 directory or add the alternate paths to additional directories in the Project|Configure|C Compiler|Paths|Library paths menu*. The header files
;hctl_driver.h, qdec_driver.h and qdec_signal_generator.h are present in main project directory.*/
;
;#include "hctl2032_driver.h"
;#include "qdec_driver.h"
;#include "qdec_signal_generator.h"
;#include "delay.h"
;#include "math.h"
;
;/*#define _ATXMEGA_DEVICE_*/
;/* use the ATxmega128A1 USARTF0 for getchar and putchar functions. The following is required
;because by default putchar send data on USARTC0 */
;#define _ATXMEGA_USART_ USARTF0
;#include "stdio.h"
;
;
;/* To select between 2 MHz and 32 MHz clock source*/
;#define SYSTEM_CLOCK_32MHZ
;/*To select between 19200 and 115200bps baud rate*/
;#define SYSTEM_CLOCK_32MHZ_115200
;
;/* To select between Periodic TTC and ETC*/
;#define CONTROLLER_ETC
;
;/* To select between two settings of controller. One settings is good in terms of nicer inter-event times
;and other settings are good for more robust control performance but inter-event times may not be that nice */
;// Comment (using /* */) the following for controller settings for nicer control performance
;/*#define NICE_INTER_EVENT_TIMES*/
;
;
;
;
;/**************** GLOBAL VARIABLES ****************
;***************************************************
;***************************************************/
;
;/*! \brief Direction of the output signal*/
;bool    dir  = 0; // 0 = CW or count-up (Count-up would start from BOT (0x0000) value)=, 1 = CCW or count down (Count down would start from TOP value (4*lineCount - 1))
;
;/*! \brief Declare your global variables here */
;unsigned int NumOfRev = 0;
;unsigned char Counter_OverFlows=0;
;unsigned char LED = 0xFF;
;bit LED_Flag=1;
;unsigned char CountMode = 1;  // Count mode for HCTL2032 Quad Decoder
;unsigned char counter_state1 = 0x00; // Counter state i.e. is it normal, overflow or underflow
;unsigned char counter_state2 = 0x00;
;
;signed long int Pos_Count_HCTL_1 = 0x00000000;
;signed long int Pos_Count_HCTL_2 = 0x00000000;
;signed long int Pos_Count_HCTL_3 = 0x00000000;
;signed long int Pos_Count_HCTL_4 = 0x00000000;
;
;unsigned long int Temp_Pos_Count_HCTL_1 = 0x00000000;
;unsigned long int Temp_Pos_Count_HCTL_2 = 0x00000000;
;unsigned long int Temp_Pos_Count_HCTL_3 = 0x00000000;
;unsigned long int Temp_Pos_Count_HCTL_4 = 0x00000000;
;
;/***** Estimation Variables and Constants *****/
;float h = 0.01; // 10 ms sampling Time for ETC

	.DSEG
;const float Td = 0.03; // Derivative Prediction Horizon
;const float Ti = 0.1; // Integration Time
;const float Tt1 = 0.05; // Tracking Time for Trolley
;const float Tt2 = 0.05; // Tracking Time for Arm
;const char N = 10; // Low Pass PF Parameter for Derivative Action
;const char N1 = 5;//5; // Low Pass PF Parameter for Derivative Action for Payload
;//const float PI = 3.1416;
;float ad = 0;
;float bd = 0;
;float ad1 = 0;
;float bd1 = 0;
;float bi1 = 0;
;float bi2 = 0;
;float a01 = 0;
;float a02 = 0;
;
;/***** System Parameters *****/
;const float r_x = 0.0379127; // Trolley Pully Radius
;const unsigned int PPR1 = 4096; // Encoder Pulses Per Revolution for Encoder 1 (Payload X Angle)
;const unsigned int PPR2 = 4096; // Encoder Pulses Per Revolution for Encoder 2 (Payload Y Angle)
;const unsigned int PPR3 = 4096; // Encoder Pulses Per Revolution for Encoder 3 (Trolley Position)
;const unsigned int PPR4 = 4000; // Encoder Pulses Per Revolution for Encoder 4 (Arm Position)
;const unsigned int PPR5 = 4096; // Encoder Pulses Per Revolution for Encoder 5 (Lift Line Length)
;
;
;/***** System States initialization *****/
;float x_1 = 0.0;
;float x_2 = 0.0;
;float x_3 = 0.0;
;float x_4 = 0.0;
;float x_a1 = 0.0;
;float x_5 = 0.0;
;float x_6 = 0.0;
;float x_7 = 0.0;
;float x_8 = 0.0;
;float x_a2 = 0.0;
;float x_1_old = 0.0; // x_1 at last time step
;float x_2_old = 0.0; // x_2 at last time step
;float x_3_old = 0.0; // x_3 at last time step
;float x_4_old = 0.0; // x_4 at last time step
;float x_5_old = 0.0; // x_5 at last time step
;float x_6_old = 0.0; // x_6 at last time step
;float x_7_old = 0.0; // x_7 at last time step
;float x_8_old = 0.0; // x_8 at last time step
;float x_a1_old = 0.0; // x_a1 at last time step
;float x_a2_old = 0.0; // x_a2 at last time step
;float Phi = 0.0; // \Phi(k-1)=\frac{1}{1+0.0138x_1(k-1)^2}
;
;
;
;/*** System States at Last Trigger ***/
;
;float x_1_tk = 0.0;
;float x_2_tk = 0.0;
;float x_3_tk = 0.0;
;float x_4_tk = 0.0;
;float x_a1_tk = 0.0;
;float x_5_tk = 0.0;
;float x_6_tk = 0.0;
;float x_7_tk = 0.0;
;float x_8_tk = 0.0;
;float x_a2_tk = 0.0;
;
;/********** Event Generation Rule (EGR) Variables **********/
;/** Measurement Error Norm**/
;float norm_e = 0.0;
;/** Shifted State **/
;float norm_epsilon = 0.0;
;
;
;/************************************************************
;**************************************************************
;**************************************************************
;**************************************************************/
;
;/*************** EGR and CONTROLLER Parameters **************
;*************************************************************
;*************************************************************/
;// Conditional compilation. It Depends on whether we want nicer inter-event times
;// or more robust control performance. Comment macro 'NICE_INTER_EVENT_TIMES' (see above) to get robust
;// control performance but then inter-event times will not be that nice
;#ifdef NICE_INTER_EVENT_TIMES
;// For Achieving Nice inter-event Timings (exponential behavior) use following parameters
;// but then control performance will be compromised and payload oscillation may become big sometime
;const float sigma = 0.01; //0.035;//0.007;//0.035;//0.075;//0.0075;//0.035;//0.007;//0.007;//0.004;//0.0004;//0.0000001;//0.00005;//0.0008;//0.035;//0.075;//0.035;//0.0175;//0.075;//0.0080;//0.075;
;const float delta = 0.005; //0.005;//0.005;//0.005;//0.0001;//0.0;//0.005;//0.0001;//0.001;//0.01;//0.0005;//0.005;//0.001;//0.005; //0.01;
;const float taumina = 0.04;
;
;/***** Weighting Matrix *****/
;// Weights for Trolley States
;const float w_1 = 1;        // Weight for x_1
;const float w_2 = 1;        // Weight for x_2
;
;// Weights for Payload alpha angle
;// Enable Following weights for Nicer inter-event times but control performance will be compromised
;const float w_3_d = 0.1;//0.01;//0.0001;//0.1; //0.01;//0.0001; // we want to change w_3, w_4 , w_8 and w_9 during operation
;float w_3 = w_3_d;  //0.0001;//0.00000001;//0.000001;//0.01;//0.00001;//0.1;//0.001;//0.01;//1;//0.0001;//0.01;//0.1;//0.001;//0.0001;//0.00001;//0.0001;//0.005;//0.5;//1;  // Weight for x_3
;const float w_4_d = 1;//0.1;//0.01;//0.0001;//1; //0.01;//0.0001;
;float w_4 = w_4_d;  //0.0001;//0.00000001;//0.01;//0.00001;//0.1;//0.001;//0.01;//1;//0.0001;//0.01;//0.1;//0.001;//0.0001;//0.00001;//0.0001;//0.005;//1;  // Weight for x_4
;
;// Weights for Integrator State x_a1
;const float w_5 = 1;        // Weight for x_a1
;
;// Weights for Arm States
;const float w_6 = 1;        // Weight for x_5
;const float w_7 = 1;       // Weight for x_6
;
;// Weights for Payload Beta States
;const float w_8_d = 0.02;//0.002;//0.00002;//0.02; //0.002;//0.00002;
;float w_8 = w_8_d;  //0.00002;//0.00000001;//0.000001;//0.1;//0.001;//0.01;//1;//0.00002;//0.01;//0.1;//0.001;//0.00002;//0.000001;//0.00002;//0.0002;//0.02;//0.2;//1;  // Weight for x_7
;const float w_9_d = 1;//0.02;//0.002;//0.00002;//1; //0.1//0.002;//0.00002;
;float w_9 = w_9_d;  //0.00002;//0.00000001;//0.000001;//0.1;//0.001;//0.01;//1;//0.00002;//0.01;//0.1;//0.001;//0.00002;//0.000001;//0.00002;//0.0002;//0.5;//0.002;//0.02;//0.2;//1;  // Weight for x_8
;
;// Weights for Integrator State x_a2
;const float w_10 = 1;      // Weight for x_a2
;
;
;/****** Controller Parameters/Gains ******/
;// For Nicer Inter-Event-Times
;const float K11 = -5.7015; //Trolley Position Gain
;const float K12 =  0.2;    //1.0;//0.1;//1.0;//4.3408; // Trollery Speed Gain ,,, it is too high and creating some problems especially when speed estimate is crude. May work well when using EKF but not with crude estimate!!
;const float K13 = -2.5;    //-2.0;//-2.5;//-2.9673;// // X-Angle Position Gain
;const float K14 = -0.1498; // X-Angle Speed Gain
;const float K15 = 2.125;   //2.25;//2.0;//2.5317;//3.5;//2.5317; // Integral State (x_{a1}) Gain  , For real crane, 1.1
;const float K21 = -2.7525; // Arm Position Gain
;const float K22 = 0.1138;  // Arm Speed Gain
;const float K23 = 2.5;     //2.0;//2.5;//3.0;//3.5477; // Y-Angle Position Gain
;const float K24 = 0.1716;  // Y-Angle Speed Gain
;const float K25 = 0.8;     //1.1358; // Integral State (x_{a2}) Gain
;
;#else
;
;// For Achieving Nice Control Performance use following parameters
;// but then the inter-event times will not be that nice and would not exhibit exponential behavior
;const float sigma = 0.008;//0.01;//0.001;//0.003;//0.007;//0.01; // Use for better control performance but it would give more events
;const float delta = 0.05;//0.1;//0.01;//0.001;//0.0001;//0.000001;// 0.0;//0.000000001;//0.000001;//0.00001;//0.0001;//0.005;//0.005;//0.005;//0.005;//0.0001;//0.0;//0.005;//0.0001;//0.001;//0.01;//0.0005;//0.005;//0.001;//0.005; //0.01;
;const float taumina = 0.04;
;
;/***** Weighting Matrix *****/
;// Weights for Trolley States
;const float w_1 = 1;//0.1;//0.01;//0.1;//1;        // Weight for x_1
;const float w_2 = 1;//0.1;//1;        // Weight for x_2
;
;// Weights for Payload alpha angle
;// Enable Following for Nicer Control Performance
;const float w_3_d = 1.0;//0.1;//0.001;//0.01;//0.001;//1;//0.1;//0.001;//0.00001;//0.0001;//0.001; //0.0001; // Use for better control performance of payload alpha angle but it would give more events
;float w_3 = w_3_d;  //0.0001;
;const float w_4_d = 0.001;//0.01;//0.01;//0.001;//1;//0.1;//0.001; //0.0001; // Use for better control performance of payload alpha angle but it would give more events
;float w_4 = w_4_d;  //0.0001;
;
;
;// Weights for Integrator State x_a1
;const float w_5 = 1;        // Weight for x_a1
;
;// Weights for Arm States
;const float w_6 = 1;//0.1;//0.001;//0.01;//0.1;//1;        // Weight for x_5
;const float w_7 = 1;//0.1;//1;        // Weight for x_6
;
;// Weights for Payload Beta States
;const float w_8_d = 1.0;//0.1;//0.001;//0.01;//0.001;//0.01;//1;//0.1;//0.0002;//0.000001;//0.00001;//0.0002; //0.0000002; // Use for better control performance of payload beta angle but it would give more events
;float w_8 = w_8_d;  //0.00002;
;const float w_9_d = 0.001;//0.01;//0.01;//0.001;//1;//0.1;//0.0002; //0.00002; // Use for better control performance of payload beta angle but it would give more events
;float w_9 = w_9_d; //0.00002;
;
;// Weights for Integrator State x_a2
;const float w_10 = 1;      // Weight for x_a2
;
;
;/****** Controller Parameters/Gains ******/
;// For Robust Controller Performance
;const float K11 = -5.7015; //Trolley Position Gain
;const float K12 =  0.2;//0.1;//0.2;//1.0;//0.1;//1.0;//4.3408; // Trollery Speed Gain ,,, it is too high and creating some problems especially when speed estimate is crude. May work well when using EKF but not with crude estimate!!
;const float K13 = -2.5;//-2.9673;// // X-Angle Position Gain
;const float K14 = -0.1498; // X-Angle Speed Gain
;const float K15 = 2.125;//2.5317;//3.5;//2.5317; // Integral State (x_{a1}) Gain  , For real crane, 1.1
;const float K21 = -2.7525; // Arm Position Gain
;const float K22 = 0.1138; // Arm Speed Gain
;const float K23 = 2.5;//3.5477;//2.5;//3.0;//3.5477; // Y-Angle Position Gain
;const float K24 = 0.1716; // Y-Angle Speed Gain
;const float K25 = 0.8;//1.1358; // Integral State (x_{a2}) Gain
;
;#endif
;
;
;/**** Reference Signals ****/
;float r_1 = 0.0;//0.3;
;float r_2 = 0.0;//PI;
;/*** Homing after encoder reset ***/
;float home_x = 0.25; // bring the trolley to this position initially
;float home_theta = PI*0.5; // bring the Arm to this position initially
;
;/*** Control Signals ***/
;// Control Signals after saturation
;float u_1 = 0.0;
;float u_2 = 0.0;
;float u_1_old = 0.0; // Control Signal for trolley at previous step
;float u_2_old = 0.0; // Control Signal for trolley at previous step
;float u_11 = 0.0; // control signal without integrator
;
;
;// Computed Control Signals (before saturation)
;float v_1 = 0.0;
;float v_2 = 0.0;
;
;// Start Flag
;bit start_flag = 0;
;
;//Switch States
;bit SW0_pressed = 1;
;bit SW1_pressed = 1;
;bit SW2_pressed = 1;
;bit SW3_pressed = 1;
;bit SW4_pressed = 1;
;
;// Decision Variable to Select Between Periodic and and ETC
;bit Periodic_ON = 0; // Select ETC initially
;
;// Number of Times ERG is violated
;unsigned long iterations = 0;
;unsigned long iterations_old = 0;
;bit first_transmission = 1;
;
;// Parameter For Reference Signal Generation
;unsigned long count_ref = 0;
;bit flag_ref = 0; // signal to integrator that new reference command is available
;
;
;// System Clocks initialization for 2MHz Clock
;void system_clocks_init(void)
; 0000 015B {

	.CSEG
; 0000 015C unsigned char n,s;
; 0000 015D 
; 0000 015E // Optimize for speed
; 0000 015F #pragma optsize-
; 0000 0160 // Save interrupts enabled/disabled state
; 0000 0161 s=SREG;
;	n -> R17
;	s -> R16
; 0000 0162 // Disable interrupts
; 0000 0163 #asm("cli")
; 0000 0164 
; 0000 0165 // Internal 2 MHz RC oscillator initialization
; 0000 0166 // Enable the internal 2 MHz RC oscillator
; 0000 0167 OSC.CTRL|=OSC_RC2MEN_bm;
; 0000 0168 
; 0000 0169 // System Clock prescaler A division factor: 1
; 0000 016A // System Clock prescalers B & C division factors: B:1, C:1
; 0000 016B // ClkPer4: 32000,000 kHz
; 0000 016C // ClkPer2: 32000,000 kHz
; 0000 016D // ClkPer:  32000,000 kHz
; 0000 016E // ClkCPU:  32000,000 kHz
; 0000 016F n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
; 0000 0170     CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
; 0000 0171 CCP=CCP_IOREG_gc;
; 0000 0172 CLK.PSCTRL=n;
; 0000 0173 
; 0000 0174 // Disable the autocalibration of the internal 2 MHz RC oscillator
; 0000 0175 DFLLRC2M.CTRL&= ~DFLL_ENABLE_bm;
; 0000 0176 
; 0000 0177 // Wait for the internal 2 MHz RC oscillator to stabilize
; 0000 0178 while ((OSC.STATUS & OSC_RC2MRDY_bm)==0);
; 0000 0179 
; 0000 017A // Select the system clock source: 2 MHz Internal RC Osc.
; 0000 017B n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC2M_gc;
; 0000 017C CCP=CCP_IOREG_gc;
; 0000 017D CLK.CTRL=n;
; 0000 017E 
; 0000 017F 
; 0000 0180 // Disable the unused oscillators: 32 MHz, 32 kHz, external clock/crystal oscillator, PLL
; 0000 0181 OSC.CTRL&= ~(OSC_RC32MEN_bm | OSC_RC32KEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);
; 0000 0182 
; 0000 0183 // Peripheral Clock output: Disabled
; 0000 0184 PORTCFG.CLKEVOUT=(PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;
; 0000 0185 
; 0000 0186 // Restore interrupts enabled/disabled state
; 0000 0187 SREG=s;
; 0000 0188 // Restore optimization for size if needed
; 0000 0189 #pragma optsize_default
; 0000 018A }
;
;// System Clocks initialization for 32MHz clock
;void system_clocks_init_32mhz(void)
; 0000 018E {
_system_clocks_init_32mhz:
; 0000 018F unsigned char n,s;
; 0000 0190 
; 0000 0191 // Optimize for speed
; 0000 0192 #pragma optsize-
; 0000 0193 // Save interrupts enabled/disabled state
; 0000 0194 s=SREG;
	ST   -Y,R17
	ST   -Y,R16
;	n -> R17
;	s -> R16
	IN   R16,63
; 0000 0195 // Disable interrupts
; 0000 0196 #asm("cli")
	cli
; 0000 0197 
; 0000 0198 // Internal 32 kHz RC oscillator initialization
; 0000 0199 // Enable the internal 32 kHz RC oscillator
; 0000 019A OSC.CTRL|=OSC_RC32KEN_bm;
	LDS  R30,80
	ORI  R30,4
	STS  80,R30
; 0000 019B // Wait for the internal 32 kHz RC oscillator to stabilize
; 0000 019C while ((OSC.STATUS & OSC_RC32KRDY_bm)==0);
_0xD:
	LDS  R30,81
	ANDI R30,LOW(0x4)
	BREQ _0xD
; 0000 019D 
; 0000 019E // Internal 32 MHz RC oscillator initialization
; 0000 019F // Enable the internal 32 MHz RC oscillator
; 0000 01A0 OSC.CTRL|=OSC_RC32MEN_bm;
	LDS  R30,80
	ORI  R30,2
	STS  80,R30
; 0000 01A1 
; 0000 01A2 // System Clock prescaler A division factor: 1
; 0000 01A3 // System Clock prescalers B & C division factors: B:1, C:1
; 0000 01A4 // ClkPer4: 32000.000 kHz
; 0000 01A5 // ClkPer2: 32000.000 kHz
; 0000 01A6 // ClkPer:  32000.000 kHz
; 0000 01A7 // ClkCPU:  32000.000 kHz
; 0000 01A8 n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
; 0000 01A9 	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	LDS  R30,65
	ANDI R30,LOW(0x80)
	MOV  R17,R30
; 0000 01AA CCP=CCP_IOREG_gc;
	LDI  R30,LOW(216)
	OUT  0x34,R30
; 0000 01AB CLK.PSCTRL=n;
	STS  65,R17
; 0000 01AC 
; 0000 01AD // Internal 32 MHz RC osc. calibration reference clock source: 32.768 kHz Internal Osc.
; 0000 01AE OSC.DFLLCTRL&= ~(OSC_RC32MCREF_bm | OSC_RC2MCREF_bm);
	LDS  R30,86
	ANDI R30,LOW(0xFC)
	STS  86,R30
; 0000 01AF // Enable the autocalibration of the internal 32 MHz RC oscillator
; 0000 01B0 DFLLRC32M.CTRL|=DFLL_ENABLE_bm;
	LDS  R30,96
	ORI  R30,1
	STS  96,R30
; 0000 01B1 
; 0000 01B2 // Wait for the internal 32 MHz RC oscillator to stabilize
; 0000 01B3 while ((OSC.STATUS & OSC_RC32MRDY_bm)==0);
_0x10:
	LDS  R30,81
	ANDI R30,LOW(0x2)
	BREQ _0x10
; 0000 01B4 
; 0000 01B5 // Select the system clock source: 32 MHz Internal RC Osc.
; 0000 01B6 n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC32M_gc;
	LDS  R30,64
	ANDI R30,LOW(0xF8)
	ORI  R30,1
	MOV  R17,R30
; 0000 01B7 CCP=CCP_IOREG_gc;
	LDI  R30,LOW(216)
	OUT  0x34,R30
; 0000 01B8 CLK.CTRL=n;
	STS  64,R17
; 0000 01B9 
; 0000 01BA // Disable the unused oscillators: 2 MHz, external clock/crystal oscillator, PLL
; 0000 01BB OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);
	LDS  R30,80
	ANDI R30,LOW(0xE6)
	STS  80,R30
; 0000 01BC 
; 0000 01BD // Peripheral Clock output: Disabled
; 0000 01BE PORTCFG.CLKEVOUT=(PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;
	LDS  R30,180
	ANDI R30,LOW(0xFC)
	STS  180,R30
; 0000 01BF 
; 0000 01C0 // Restore interrupts enabled/disabled state
; 0000 01C1 SREG=s;
	OUT  0x3F,R16
; 0000 01C2 // Restore optimization for size if needed
; 0000 01C3 #pragma optsize_default
; 0000 01C4 }
	LD   R16,Y+
	LD   R17,Y+
	RET
;
;// Disable a Timer/Counter type 0
;void tc0_disable(TC0_t *ptc)
; 0000 01C8 {
_tc0_disable:
; 0000 01C9 // Timer/Counter off
; 0000 01CA ptc->CTRLA=(ptc->CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_OFF_gc;
;	*ptc -> Y+0
	LD   R26,Y
	LDD  R27,Y+1
	LD   R30,X
	ANDI R30,LOW(0xF0)
	ST   X,R30
; 0000 01CB // Issue a reset command
; 0000 01CC ptc->CTRLFSET=TC_CMD_RESET_gc;
	ADIW R26,9
	LDI  R30,LOW(12)
	ST   X,R30
; 0000 01CD }
	ADIW R28,2
	RET
;
;// Disable an USART
;void usart_disable(USART_t *pu)
; 0000 01D1 {
; 0000 01D2 // Rx and Tx are off
; 0000 01D3 pu->CTRLB=0;
;	*pu -> Y+0
; 0000 01D4 // Ensure that all interrupts generated by the USART are off
; 0000 01D5 pu->CTRLA=0;
; 0000 01D6 }
;
;// USARTF0 initialization
;void usartf0_init(void)
; 0000 01DA {
_usartf0_init:
; 0000 01DB // Note: the correct PORTF direction for the RxD, TxD and XCK signals
; 0000 01DC // is configured in the ports_init function
; 0000 01DD 
; 0000 01DE // Transmitter is enabled
; 0000 01DF // Set TxD=1
; 0000 01E0 PORTF.OUTSET=0x08;
	LDI  R30,LOW(8)
	STS  1701,R30
; 0000 01E1 
; 0000 01E2 // Communication mode: Asynchronous USART
; 0000 01E3 // Data bits: 8
; 0000 01E4 // Stop bits: 1
; 0000 01E5 // Parity: Disabled
; 0000 01E6 USARTF0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	LDI  R30,LOW(3)
	STS  2981,R30
; 0000 01E7 
; 0000 01E8 // Receive complete interrupt: Disabled
; 0000 01E9 // Transmit complete interrupt: Disabled
; 0000 01EA // Data register empty interrupt: Disabled
; 0000 01EB USARTF0.CTRLA=(USARTF0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
; 0000 01EC     USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	LDS  R30,2979
	ANDI R30,LOW(0xC0)
	STS  2979,R30
; 0000 01ED 
; 0000 01EE // If We are using 32MHz Clock
; 0000 01EF #ifdef SYSTEM_CLOCK_32MHZ && SYSTEM_CLOCK_32MHZ_115200
; 0000 01F0 // Required Baud rate: 115200
; 0000 01F1 // Real Baud Rate: 115211.5 (x1 Mode), Error: 0.0 %
; 0000 01F2 USARTF0.BAUDCTRLA=0x2E;
	LDI  R30,LOW(46)
	STS  2982,R30
; 0000 01F3 USARTF0.BAUDCTRLB=((0x09 << USART_BSCALE_bp) & USART_BSCALE_gm) | 0x08;
	LDI  R30,LOW(152)
	STS  2983,R30
; 0000 01F4 #elif  SYSTEM_CLOCK_32MHZ
; 0000 01F5 // Required Baud rate: 19200
; 0000 01F6 // Real Baud Rate: 19196.2, Error: 0.0 %
; 0000 01F7 USARTF0.BAUDCTRLA=0xE5;
; 0000 01F8 USARTF0.BAUDCTRLB=((0x0B << USART_BSCALE_bp) & USART_BSCALE_gm) | 0x0C;
; 0000 01F9 #else
; 0000 01FA // If we are using 2MHz Clock
; 0000 01FB // Required Baud rate: 19200
; 0000 01FC // Real Baud Rate: 19196.2 (x2 Mode), Error: 0.0 %
; 0000 01FD USARTF0.BAUDCTRLA=0x03;
; 0000 01FE USARTF0.BAUDCTRLB=((0x09 << USART_BSCALE_bp) & USART_BSCALE_gm) | 0x06;
; 0000 01FF #endif
; 0000 0200 
; 0000 0201 #ifdef SYSTEM_CLOCK_32MHZ
; 0000 0202 // Receiver: Off
; 0000 0203 // Transmitter: On
; 0000 0204 // Double transmission speed mode: Off
; 0000 0205 // Multi-processor communication mode: Off
; 0000 0206 USARTF0.CTRLB=(USARTF0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
; 0000 0207 	USART_TXEN_bm;
	LDS  R30,2980
	ANDI R30,LOW(0xE0)
	ORI  R30,8
	STS  2980,R30
; 0000 0208 #else
; 0000 0209 // Receiver: Off
; 0000 020A // Transmitter: On
; 0000 020B // Double transmission speed mode: On
; 0000 020C // Multi-processor communication mode: Off
; 0000 020D USARTF0.CTRLB=(USARTF0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
; 0000 020E 	USART_TXEN_bm | USART_CLK2X_bm;
; 0000 020F #endif
; 0000 0210 }
	RET
;
;// Write a character to the USARTF0 Transmitter
;#pragma used+
;void putchar_usartf0(char c)
; 0000 0215 {
_putchar_usartf0:
; 0000 0216 while ((USARTF0.STATUS & USART_DREIF_bm) == 0);
;	c -> Y+0
_0x13:
	LDS  R30,2977
	ANDI R30,LOW(0x20)
	BREQ _0x13
; 0000 0217 USARTF0.DATA=c;
	LD   R30,Y
	STS  2976,R30
; 0000 0218 }
	ADIW R28,1
	RET
;#pragma used-
;
;/** Initialize Timer/Counter TCD0**/
;void tcd0_init(void)
; 0000 021D {
_tcd0_init:
; 0000 021E unsigned char s;
; 0000 021F //unsigned char n;
; 0000 0220 
; 0000 0221 // Note: the correct PORTC direction for the Compare Channels outputs
; 0000 0222 // is configured in the ports_init function
; 0000 0223 
; 0000 0224 // Save interrupts enabled/disabled state
; 0000 0225 s=SREG;
	ST   -Y,R17
;	s -> R17
	IN   R17,63
; 0000 0226 // Disable interrupts
; 0000 0227 #asm("cli")
	cli
; 0000 0228 
; 0000 0229 // Disable and reset the timer/counter just to be sure
; 0000 022A tc0_disable(&TCD0);
	LDI  R30,LOW(2304)
	LDI  R31,HIGH(2304)
	ST   -Y,R31
	ST   -Y,R30
	RCALL _tc0_disable
; 0000 022B 
; 0000 022C // Note that we want to set sampling time of 10 ms.
; 0000 022D // so these settings are according to that for 2MHz clock
; 0000 022E // for 32MHz use Clock/64
; 0000 022F #ifdef SYSTEM_CLOCK_32MHZ
; 0000 0230 // if 32 mhz clock is being used
; 0000 0231 TCD0.CTRLA=(TCD0.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV64_gc;
	LDS  R30,2304
	ANDI R30,LOW(0xF0)
	ORI  R30,LOW(0x5)
	STS  2304,R30
; 0000 0232 //TCD0.CTRLA=(TCD0.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV1024_gc;   // temporary change to get longer sampling interval
; 0000 0233 #else
; 0000 0234 // if 2 mhz clock is being used. Clock source: Peripheral Clock/4.
; 0000 0235 TCD0.CTRLA=(TCD0.CTRLA & (~TC0_CLKSEL_gm)) | TC_CLKSEL_DIV4_gc;
; 0000 0236 #endif
; 0000 0237 
; 0000 0238 // Overflow interrupt: Low Level
; 0000 0239 // Error interrupt: Disabled
; 0000 023A TCD0.INTCTRLA=(TCD0.INTCTRLA & (~(TC0_ERRINTLVL_gm | TC0_OVFINTLVL_gm))) | TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_LO_gc;
	LDS  R30,2310
	ANDI R30,LOW(0xF0)
	ORI  R30,1
	STS  2310,R30
; 0000 023B 
; 0000 023C // Mode: Dual Slope PWM Operation, Overflow Int./Event on TOP and Bottom
; 0000 023D // Compare/Capture on channel A: Off
; 0000 023E // Compare/Capture on channel B: Off
; 0000 023F // Compare/Capture on channel C: Off
; 0000 0240 // Compare/Capture on channel D: Off
; 0000 0241 TCD0.CTRLB=(TCD0.CTRLB & (~(TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC0_WGMODE_gm))) | TC_WGMODE_NORMAL_gc;
	LDS  R30,2305
	ANDI R30,LOW(0x8)
	STS  2305,R30
; 0000 0242 
; 0000 0243 // Clear the interrupt flags
; 0000 0244 TCD0.INTFLAGS=TCD0.INTFLAGS;
	LDS  R30,2316
	STS  2316,R30
; 0000 0245 
; 0000 0246 // Set counter register
; 0000 0247 TCD0.CNT=0x0000;
	CALL SUBOPT_0x0
; 0000 0248 // Set period register
; 0000 0249 TCD0.PER= 0x1388; //Clock/(4*5000) = 100 Hz => h = 10 ms  , for 32Mhz (clock/(64*5000) = 100 Hz
; 0000 024A // Set channel A Compare/Capture register
; 0000 024B TCD0.CCA=0x0000;
	LDI  R30,LOW(0)
	LDI  R31,HIGH(0)
	STS  2344,R30
	STS  2344+1,R31
; 0000 024C // Set channel B Compare/Capture register
; 0000 024D TCD0.CCB=0x0000;
	STS  2346,R30
	STS  2346+1,R31
; 0000 024E // Set channel C Compare/Capture register
; 0000 024F TCD0.CCC=0x0000;
	STS  2348,R30
	STS  2348+1,R31
; 0000 0250 // Set channel D Compare/Capture register
; 0000 0251 TCD0.CCD=0x0000;
	STS  2350,R30
	STS  2350+1,R31
; 0000 0252 
; 0000 0253 // Restore interrupts enabled/disabled state
; 0000 0254 SREG=s;
	OUT  0x3F,R17
; 0000 0255 }
	LD   R17,Y+
	RET
;
;/**** Timer/counter TCD0 overflow/Underflow interrupt service routine *****/
;// All Four Functions: Encoder Reading, Speed Estimation, Event-Generation Rule and Control Computation
;// are being computed inside this Interrupt Service Routine
;interrupt [TCD0_OVF_vect] void tcd0_overflow_isr(void)
; 0000 025B {
_tcd0_overflow_isr:
	CALL SUBOPT_0x1
; 0000 025C /*status registered is not stored automatically when entering ISR and restored when leaving ISR. See pg. 9 in device manual*/
; 0000 025D 
; 0000 025E // write your code here
; 0000 025F unsigned char * p;
; 0000 0260 unsigned char i;
; 0000 0261 //float x_test = 0;
; 0000 0262 
; 0000 0263 // Measure the control delay
; 0000 0264 //PORTF.OUTTGL = PIN6_bm;
; 0000 0265 
; 0000 0266 // To Control Inter-Event Time
; 0000 0267 iterations +=1;
;	*p -> R16,R17
;	i -> R19
	CALL SUBOPT_0x2
	__ADDD1N 1
	STS  _iterations,R30
	STS  _iterations+1,R31
	STS  _iterations+2,R22
	STS  _iterations+3,R23
; 0000 0268 
; 0000 0269 /*** Payload X Angle Data in Encoder Counts ***/
; 0000 026A /***** First HCTL2032: Encoder 1- Read and Send Position Count from QDEC of Second HCTL2032 *****/
; 0000 026B Temp_Pos_Count_HCTL_3 = HCTL2032_Read_Count_Data(&PORTD, &PORTE, false);
	CALL SUBOPT_0x3
	LDI  R30,LOW(0)
	ST   -Y,R30
	CALL _HCTL2032_Read_Count_Data
	STS  _Temp_Pos_Count_HCTL_3,R30
	STS  _Temp_Pos_Count_HCTL_3+1,R31
	STS  _Temp_Pos_Count_HCTL_3+2,R22
	STS  _Temp_Pos_Count_HCTL_3+3,R23
; 0000 026C // Scale the encoder count between  -2147483648 t0 2147483647
; 0000 026D Pos_Count_HCTL_3 = (signed long)Temp_Pos_Count_HCTL_3;
	STS  _Pos_Count_HCTL_3,R30
	STS  _Pos_Count_HCTL_3+1,R31
	STS  _Pos_Count_HCTL_3+2,R22
	STS  _Pos_Count_HCTL_3+3,R23
; 0000 026E 
; 0000 026F /*** Payload Y Angle Data in Encoder Counts***/
; 0000 0270 /****** First HCTL2032: Encoder 2- Read and Send Position Count from QDEC of First HCTL2032 ******/
; 0000 0271 Temp_Pos_Count_HCTL_4 = HCTL2032_Read_Count_Data(&PORTD, &PORTE, true);
	CALL SUBOPT_0x3
	LDI  R30,LOW(1)
	ST   -Y,R30
	CALL _HCTL2032_Read_Count_Data
	STS  _Temp_Pos_Count_HCTL_4,R30
	STS  _Temp_Pos_Count_HCTL_4+1,R31
	STS  _Temp_Pos_Count_HCTL_4+2,R22
	STS  _Temp_Pos_Count_HCTL_4+3,R23
; 0000 0272 // Scale the encoder count between  -2147483648 t0 2147483647
; 0000 0273 Pos_Count_HCTL_4 = (signed long)Temp_Pos_Count_HCTL_4;
	STS  _Pos_Count_HCTL_4,R30
	STS  _Pos_Count_HCTL_4+1,R31
	STS  _Pos_Count_HCTL_4+2,R22
	STS  _Pos_Count_HCTL_4+3,R23
; 0000 0274 
; 0000 0275 /*** Trolley Position Data in Encoder Counts ***/
; 0000 0276 /* Second HCTL2032: Encoder 3- Read and Send Position Count from QDEC of Second HCTL2032 */
; 0000 0277 Temp_Pos_Count_HCTL_1 = HCTL2032_Read_Count_Data(&PORTH, &PORTC, false);
	CALL SUBOPT_0x4
	LDI  R30,LOW(0)
	ST   -Y,R30
	CALL _HCTL2032_Read_Count_Data
	STS  _Temp_Pos_Count_HCTL_1,R30
	STS  _Temp_Pos_Count_HCTL_1+1,R31
	STS  _Temp_Pos_Count_HCTL_1+2,R22
	STS  _Temp_Pos_Count_HCTL_1+3,R23
; 0000 0278 // Scale the encoder count between  -2147483648 t0 2147483647
; 0000 0279 Pos_Count_HCTL_1 = (signed long)Temp_Pos_Count_HCTL_1;
	STS  _Pos_Count_HCTL_1,R30
	STS  _Pos_Count_HCTL_1+1,R31
	STS  _Pos_Count_HCTL_1+2,R22
	STS  _Pos_Count_HCTL_1+3,R23
; 0000 027A 
; 0000 027B /*** Arm Position Data in Encoder Counts ***/
; 0000 027C /* Second HCTL2032: Encoder 4- Read and Send Position Count from QDEC of Second HCTL2032 */
; 0000 027D Temp_Pos_Count_HCTL_2 = HCTL2032_Read_Count_Data(&PORTH, &PORTC, true);
	CALL SUBOPT_0x4
	LDI  R30,LOW(1)
	ST   -Y,R30
	CALL _HCTL2032_Read_Count_Data
	STS  _Temp_Pos_Count_HCTL_2,R30
	STS  _Temp_Pos_Count_HCTL_2+1,R31
	STS  _Temp_Pos_Count_HCTL_2+2,R22
	STS  _Temp_Pos_Count_HCTL_2+3,R23
; 0000 027E // Scale the encoder count between  -2147483648 t0 2147483647
; 0000 027F Pos_Count_HCTL_2 = (signed long)Temp_Pos_Count_HCTL_2;
	STS  _Pos_Count_HCTL_2,R30
	STS  _Pos_Count_HCTL_2+1,R31
	STS  _Pos_Count_HCTL_2+2,R22
	STS  _Pos_Count_HCTL_2+3,R23
; 0000 0280 
; 0000 0281 
; 0000 0282 /********** Trolley and Arm Position Data in meter and radians *************/
; 0000 0283 
; 0000 0284 /* Calculate Trolley Position 'x_1' in meters */
; 0000 0285 x_1 = (float)Pos_Count_HCTL_3*2.0*PI*r_x/(float)PPR3;
	LDS  R30,_Pos_Count_HCTL_3
	LDS  R31,_Pos_Count_HCTL_3+1
	LDS  R22,_Pos_Count_HCTL_3+2
	LDS  R23,_Pos_Count_HCTL_3+3
	CALL SUBOPT_0x5
	__GETD2N 0x3D1B4A59
	CALL __MULF12
	CALL SUBOPT_0x6
	CALL SUBOPT_0x7
; 0000 0286 
; 0000 0287 /* Calculate Arm Position 'x_5' in radians*/
; 0000 0288 x_5 = (float)Pos_Count_HCTL_4*2.0*PI/(float)PPR4;
	LDS  R30,_Pos_Count_HCTL_4
	LDS  R31,_Pos_Count_HCTL_4+1
	LDS  R22,_Pos_Count_HCTL_4+2
	LDS  R23,_Pos_Count_HCTL_4+3
	CALL SUBOPT_0x5
	MOVW R26,R30
	MOVW R24,R22
	__GETD1N 0x457A0000
	CALL __DIVF21
	CALL SUBOPT_0x8
; 0000 0289 
; 0000 028A /********** Payload Angle Data ***********/
; 0000 028B 
; 0000 028C /* Calculate X-Angle 'x_3' in Radians */
; 0000 028D x_3 = (float)Pos_Count_HCTL_1*2.0*PI/(float)PPR1;
	LDS  R30,_Pos_Count_HCTL_1
	LDS  R31,_Pos_Count_HCTL_1+1
	LDS  R22,_Pos_Count_HCTL_1+2
	LDS  R23,_Pos_Count_HCTL_1+3
	CALL SUBOPT_0x5
	CALL SUBOPT_0x6
	CALL SUBOPT_0x9
; 0000 028E // Invert Position (see thesis page 41 for description)
; 0000 028F //x_3 = -x_3; // not required if we are reading encoders with correct phase on ED
; 0000 0290 
; 0000 0291 /* Calculate Y-Angle 'x_7' in Radians */
; 0000 0292 x_7 = (float)Pos_Count_HCTL_2*2.0*PI/(float)PPR2;
	LDS  R30,_Pos_Count_HCTL_2
	LDS  R31,_Pos_Count_HCTL_2+1
	LDS  R22,_Pos_Count_HCTL_2+2
	LDS  R23,_Pos_Count_HCTL_2+3
	CALL SUBOPT_0x5
	CALL SUBOPT_0x6
	CALL SUBOPT_0xA
; 0000 0293 
; 0000 0294 /***** Error Filtering, Encoder data may have glitch which can be disastrous for control ****/
; 0000 0295 // Temorary Solution: Since error is always very high number so it can be detected from valid data
; 0000 0296 // if error occurs then keep the previous state
; 0000 0297 // Permanent Solution: Hardware Redesign is required withj MCU on board
; 0000 0298 // Problem is occuring due to voltage level translator
; 0000 0299 
; 0000 029A if(fabs(x_1) > 0.6) // Maximum range for trolley is 0.5m
	CALL SUBOPT_0xB
	CALL SUBOPT_0xC
	__GETD1N 0x3F19999A
	CALL __CMPF12
	BREQ PC+2
	BRCC PC+3
	JMP  _0x16
; 0000 029B x_1 = x_1_old;
	CALL SUBOPT_0xD
	CALL SUBOPT_0x7
; 0000 029C if(fabs(x_3) > 0.9) // Maximum range for Payload X angle is 0.8 radians
_0x16:
	CALL SUBOPT_0xE
	CALL SUBOPT_0xC
	CALL SUBOPT_0xF
	BREQ PC+2
	BRCC PC+3
	JMP  _0x17
; 0000 029D x_3 = x_3_old;
	CALL SUBOPT_0x10
	CALL SUBOPT_0x9
; 0000 029E if(fabs(x_5) > 6.2832) // Maximum range for Arm is plus minus PI
_0x17:
	CALL SUBOPT_0x11
	CALL SUBOPT_0xC
	__GETD1N 0x40C90FF9
	CALL __CMPF12
	BREQ PC+2
	BRCC PC+3
	JMP  _0x18
; 0000 029F x_5 = x_5_old;
	LDS  R30,_x_5_old
	LDS  R31,_x_5_old+1
	LDS  R22,_x_5_old+2
	LDS  R23,_x_5_old+3
	CALL SUBOPT_0x8
; 0000 02A0 if(fabs(x_7) > 0.9) // Maximum range for Payload Y Angle is 0.8 radians
_0x18:
	CALL SUBOPT_0x12
	CALL SUBOPT_0xC
	CALL SUBOPT_0xF
	BREQ PC+2
	BRCC PC+3
	JMP  _0x19
; 0000 02A1 x_7 = x_7_old;
	CALL SUBOPT_0x13
	CALL SUBOPT_0xA
; 0000 02A2 
; 0000 02A3 
; 0000 02A4 /*********** Estimate Speed States ***********/
; 0000 02A5 /********* EULER APPROXIMATION **********/
; 0000 02A6 // ad and bd may be different for different states but for time being we are using the same for all.
; 0000 02A7 // Consider using different in case estimation doesnt work.
; 0000 02A8 /* Trolley Speed m/sec*/
; 0000 02A9 //x_2 = ad1*x_2 + bd1*(x_1 - x_1_old);
; 0000 02AA 
; 0000 02AB /*X-Angle Speed rad/sec*/
; 0000 02AC //x_4 = ad1*x_4 + bd1*(x_3 - x_3_old);
; 0000 02AD // Invert Speed (see thesis page 41)
; 0000 02AE //x_4 = x_4; // not required if we are reading encoders on ED
; 0000 02AF 
; 0000 02B0 /*Arm Speed rad/sec*/
; 0000 02B1 //x_6 = ad*x_6 + bd*(x_5 - x_5_old);
; 0000 02B2 
; 0000 02B3 /*Y-Angle Speed rad/sec */
; 0000 02B4 //x_8 = ad1*x_8 + bd1*(x_7 - x_7_old);
; 0000 02B5 
; 0000 02B6 /********** Nonlinear MODEL BASED SPEED ESTIMATION (See Faisal's thesis for Model Details) ***********/
; 0000 02B7 
; 0000 02B8 // Varying Parameter
; 0000 02B9 Phi = 1.0/(1.0+0.0138*x_1_old*x_1_old);
_0x19:
	CALL SUBOPT_0xD
	__GETD2N 0x3C621965
	CALL __MULF12
	LDS  R26,_x_1_old
	LDS  R27,_x_1_old+1
	LDS  R24,_x_1_old+2
	LDS  R25,_x_1_old+3
	CALL SUBOPT_0x14
	CALL SUBOPT_0x15
	STS  _Phi,R30
	STS  _Phi+1,R31
	STS  _Phi+2,R22
	STS  _Phi+3,R23
; 0000 02BA // Trolley Speed in m/sec
; 0000 02BB x_2 = (1.0-41.667*h)*x_2_old+h*(-0.8464*x_3_old+5.8808*u_1_old);
	CALL SUBOPT_0x16
	__GETD2N 0x4226AB02
	CALL SUBOPT_0x14
	CALL SUBOPT_0x17
	LDS  R26,_x_2_old
	LDS  R27,_x_2_old+1
	LDS  R24,_x_2_old+2
	LDS  R25,_x_2_old+3
	CALL __MULF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x10
	__GETD2N 0xBF58ADAC
	CALL __MULF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x18
	__GETD2N 0x40BC2F83
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x19
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	STS  _x_2,R30
	STS  _x_2+1,R31
	STS  _x_2+2,R22
	STS  _x_2+3,R23
; 0000 02BC //// Payload X angle speed in radians/sec
; 0000 02BD x_4 = x_4_old + h*(-69.4450*x_2_old-17.7607*x_3_old+9.8013*u_1_old);
	LDS  R30,_x_2_old
	LDS  R31,_x_2_old+1
	LDS  R22,_x_2_old+2
	LDS  R23,_x_2_old+3
	__GETD2N 0xC28AE3D7
	CALL __MULF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x10
	__GETD2N 0x418E15EA
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x17
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x18
	__GETD2N 0x411CD220
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x19
	LDS  R26,_x_4_old
	LDS  R27,_x_4_old+1
	LDS  R24,_x_4_old+2
	LDS  R25,_x_4_old+3
	CALL __ADDF12
	STS  _x_4,R30
	STS  _x_4+1,R31
	STS  _x_4+2,R22
	STS  _x_4+3,R23
; 0000 02BE // Arm Speed in radians/sec
; 0000 02BF x_6 = (1.0 - 17.3765*h*Phi)*x_6_old + h*Phi*(0.0618*x_1_old*x_7_old+11.912*u_2_old);
	CALL SUBOPT_0x16
	__GETD2N 0x418B0312
	CALL __MULF12
	CALL SUBOPT_0x1A
	CALL SUBOPT_0x14
	CALL SUBOPT_0x17
	LDS  R26,_x_6_old
	LDS  R27,_x_6_old+1
	LDS  R24,_x_6_old+2
	LDS  R25,_x_6_old+3
	CALL __MULF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	LDS  R30,_Phi
	LDS  R31,_Phi+1
	LDS  R22,_Phi+2
	LDS  R23,_Phi+3
	CALL SUBOPT_0x1B
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0xD
	__GETD2N 0x3D7D21FF
	CALL SUBOPT_0x1C
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x1D
	__GETD2N 0x413E978D
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	STS  _x_6,R30
	STS  _x_6+1,R31
	STS  _x_6+2,R22
	STS  _x_6+3,R23
; 0000 02C0 // Payload Y angle speed in radians/sec
; 0000 02C1 x_8 = x_8_old + h*(-16.35*x_7_old-Phi*x_1_old*(-28.9608*x_6_old + 0.1030*x_1_old*x_7_old + 19.8533*u_2_old));
	CALL SUBOPT_0x13
	__GETD2N 0xC182CCCD
	CALL __MULF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0xD
	CALL SUBOPT_0x1A
	CALL __MULF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	LDS  R30,_x_6_old
	LDS  R31,_x_6_old+1
	LDS  R22,_x_6_old+2
	LDS  R23,_x_6_old+3
	__GETD2N 0xC1E7AFB8
	CALL __MULF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0xD
	__GETD2N 0x3DD2F1AA
	CALL SUBOPT_0x1C
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x1D
	__GETD2N 0x419ED38F
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x17
	CALL SUBOPT_0x1B
	LDS  R26,_x_8_old
	LDS  R27,_x_8_old+1
	LDS  R24,_x_8_old+2
	LDS  R25,_x_8_old+3
	CALL __ADDF12
	STS  _x_8,R30
	STS  _x_8+1,R31
	STS  _x_8+2,R22
	STS  _x_8+3,R23
; 0000 02C2 
; 0000 02C3 /**** Thresholding Estimates in case something goes wrong, we are thresholding because we know the maximum possible speeds *******/
; 0000 02C4 if(x_2 > 0.15) // Speed Gain for Trolley = 0.14114
	CALL SUBOPT_0x1E
	__GETD1N 0x3E19999A
	CALL __CMPF12
	BREQ PC+2
	BRCC PC+3
	JMP  _0x1A
; 0000 02C5 x_2 = 0.15;
	RJMP _0x136
; 0000 02C6 else if(x_2 < -0.15)
_0x1A:
	CALL SUBOPT_0x1E
	__GETD1N 0xBE19999A
	CALL __CMPF12
	BRSH _0x1C
; 0000 02C7 x_2 = -0.15;
_0x136:
	STS  _x_2,R30
	STS  _x_2+1,R31
	STS  _x_2+2,R22
	STS  _x_2+3,R23
; 0000 02C8 
; 0000 02C9 if(x_6 > 0.8) // Speed Gain for Arm = 0.68552
_0x1C:
	CALL SUBOPT_0x1F
	__GETD1N 0x3F4CCCCD
	CALL __CMPF12
	BREQ PC+2
	BRCC PC+3
	JMP  _0x1D
; 0000 02CA x_6 = 0.8;
	RJMP _0x137
; 0000 02CB else if(x_2 < -0.8)
_0x1D:
	CALL SUBOPT_0x1E
	__GETD1N 0xBF4CCCCD
	CALL __CMPF12
	BRSH _0x1F
; 0000 02CC x_6 = -0.8;
_0x137:
	STS  _x_6,R30
	STS  _x_6+1,R31
	STS  _x_6+2,R22
	STS  _x_6+3,R23
; 0000 02CD 
; 0000 02CE 
; 0000 02CF 
; 0000 02D0 /*** Put Limit on Event Generation ***/
; 0000 02D1 // Reset Payload Angles
; 0000 02D2 // Due to Quantization Effect because Encoders resolution is 0.0015 radians
; 0000 02D3 // Allow for 5mm deviation from zero,, s = r*alpha => alpha = 0.005/0.6
; 0000 02D4 //if(fabs(x_3) < 0.015) //  0.0083, 0.005 , 0.0
; 0000 02D5 if(fabs(x_3) < 0.0)
_0x1F:
	CALL SUBOPT_0xE
	CALL __PUTPARD1
	CALL _fabs
	TST  R23
	BRPL _0x20
; 0000 02D6 {
; 0000 02D7 w_3 = 1;
	CALL SUBOPT_0x20
	CALL SUBOPT_0x21
; 0000 02D8 w_4 = 1;
	CALL SUBOPT_0x20
	RJMP _0x138
; 0000 02D9 }
; 0000 02DA else  // otherwise use default weights
_0x20:
; 0000 02DB {
; 0000 02DC w_3 = w_3_d;
	CALL SUBOPT_0x20
	CALL SUBOPT_0x21
; 0000 02DD w_4 = w_4_d;
	__GETD1N 0x3A83126F
_0x138:
	STS  _w_4,R30
	STS  _w_4+1,R31
	STS  _w_4+2,R22
	STS  _w_4+3,R23
; 0000 02DE }
; 0000 02DF //if(fabs(x_7) < 0.015) //  0.0083 , 0.005, 0.0
; 0000 02E0 if(fabs(x_7) < 0.0)
	CALL SUBOPT_0x12
	CALL SUBOPT_0x22
	TST  R23
	BRPL _0x22
; 0000 02E1 {
; 0000 02E2 w_8 = 1;
	CALL SUBOPT_0x23
; 0000 02E3 w_9 = 1;
	CALL SUBOPT_0x20
	RJMP _0x139
; 0000 02E4 }
; 0000 02E5 else  // otherwise use default weights
_0x22:
; 0000 02E6 {
; 0000 02E7 w_8 = w_8_d;
	CALL SUBOPT_0x23
; 0000 02E8 w_9 = w_9_d;
	__GETD1N 0x3A83126F
_0x139:
	STS  _w_9,R30
	STS  _w_9+1,R31
	STS  _w_9+2,R22
	STS  _w_9+3,R23
; 0000 02E9 }
; 0000 02EA 
; 0000 02EB 
; 0000 02EC /**** Generate Reference Signal ****/
; 0000 02ED count_ref +=1;
	LDS  R30,_count_ref
	LDS  R31,_count_ref+1
	LDS  R22,_count_ref+2
	LDS  R23,_count_ref+3
	__ADDD1N 1
	STS  _count_ref,R30
	STS  _count_ref+1,R31
	STS  _count_ref+2,R22
	STS  _count_ref+3,R23
; 0000 02EE //if(count_ref == 3000)
; 0000 02EF if(count_ref == (unsigned long)((float)20/h))
	CALL SUBOPT_0x16
	__GETD2N 0x41A00000
	CALL SUBOPT_0x24
	BRNE _0x24
; 0000 02F0 {
; 0000 02F1 r_1 = 0.40;
	__GETD1N 0x3ECCCCCD
	CALL SUBOPT_0x25
; 0000 02F2 r_2 = 2.3562; // 135 degree
	__GETD1N 0x4016CBFB
	CALL SUBOPT_0x26
; 0000 02F3 flag_ref = 1; // signal to integrator that new reference command is available
; 0000 02F4 }
; 0000 02F5 //if(count_ref == 6000)
; 0000 02F6 if(count_ref == (unsigned long)((float)35/h))
_0x24:
	CALL SUBOPT_0x16
	__GETD2N 0x420C0000
	CALL SUBOPT_0x24
	BRNE _0x27
; 0000 02F7 {
; 0000 02F8 r_1 = 0.3;
	__GETD1N 0x3E99999A
	CALL SUBOPT_0x27
; 0000 02F9 r_2 = home_theta;
	CALL SUBOPT_0x26
; 0000 02FA flag_ref = 1; // signal to integrator that new reference command is available
; 0000 02FB }
; 0000 02FC 
; 0000 02FD 
; 0000 02FE 
; 0000 02FF /*******  Computation of Event Generation Rule (EGR) ********/
; 0000 0300 
; 0000 0301 if(Periodic_ON == 0 && start_flag)    // IF ETC Selected
_0x27:
	LDI  R26,0
	SBIC 0x0,7
	LDI  R26,1
	CPI  R26,LOW(0x0)
	BRNE _0x2B
	SBIC 0x0,1
	RJMP _0x2C
_0x2B:
	RJMP _0x2A
_0x2C:
; 0000 0302 {
; 0000 0303     //  norm(e) >= sigma*norm(epsilon) + delta
; 0000 0304     // Compute Norms
; 0000 0305     // Shifted State Norm
; 0000 0306     // x_a1* = -K11*r_1 , x_a2* = -K21*r_2
; 0000 0307     norm_epsilon = sqrt((x_1 - r_1)*(x_1 - r_1) + x_2*x_2 + w_3*x_3*x_3 + w_4*x_4*x_4 + (x_a1 + K11*r_1)*(x_a1 + K11*r_1) + (x_5 - r_2)*(x_5 - r_2) + x_6*x_6 + w_8*x_7*x_7 + w_9*x_8*x_8 + (x_a2 + K21*r_2)*(x_a2 + K21*r_2));
	LDS  R26,_r_1
	LDS  R27,_r_1+1
	LDS  R24,_r_1+2
	LDS  R25,_r_1+3
	CALL SUBOPT_0x28
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x29
	CALL SUBOPT_0x1E
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0xE
	LDS  R26,_w_3
	LDS  R27,_w_3+1
	LDS  R24,_w_3+2
	LDS  R25,_w_3+3
	CALL __MULF12
	LDS  R26,_x_3
	LDS  R27,_x_3+1
	LDS  R24,_x_3+2
	LDS  R25,_x_3+3
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2A
	LDS  R26,_w_4
	LDS  R27,_w_4+1
	LDS  R24,_w_4+2
	LDS  R25,_w_4+3
	CALL __MULF12
	LDS  R26,_x_4
	LDS  R27,_x_4+1
	LDS  R24,_x_4+2
	LDS  R25,_x_4+3
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2B
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2B
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2C
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2C
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2D
	CALL SUBOPT_0x1F
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x12
	LDS  R26,_w_8
	LDS  R27,_w_8+1
	LDS  R24,_w_8+2
	LDS  R25,_w_8+3
	CALL __MULF12
	LDS  R26,_x_7
	LDS  R27,_x_7+1
	LDS  R24,_x_7+2
	LDS  R25,_x_7+3
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2E
	LDS  R26,_w_9
	LDS  R27,_w_9+1
	LDS  R24,_w_9+2
	LDS  R25,_w_9+3
	CALL __MULF12
	LDS  R26,_x_8
	LDS  R27,_x_8+1
	LDS  R24,_x_8+2
	LDS  R25,_x_8+3
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2F
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x2F
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x30
	STS  _norm_epsilon,R30
	STS  _norm_epsilon+1,R31
	STS  _norm_epsilon+2,R22
	STS  _norm_epsilon+3,R23
; 0000 0308 
; 0000 0309     // Measurement Error Norm, Left Hand Side of EGR
; 0000 030A     norm_e = sqrt((x_1 - x_1_tk)*(x_1 - x_1_tk) + (x_2 - x_2_tk)*(x_2 - x_2_tk) + (x_3 - x_3_tk)*(x_3 - x_3_tk) + (x_4 - x_4_tk)*(x_4 - x_4_tk) + (x_a1 - x_a1_tk)*(x_a1 - x_a1_tk) + (x_5 - x_5_tk)*(x_5 - x_5_tk) + (x_6 - x_6_tk)*(x_6 - x_6_tk) + (x_7 - x_7_tk)*(x_7 - x_7_tk) + (x_8 - x_8_tk)*(x_8 - x_8_tk) + (x_a2 - x_a2_tk)*(x_a2 - x_a2_tk));
	LDS  R26,_x_1_tk
	LDS  R27,_x_1_tk+1
	LDS  R24,_x_1_tk+2
	LDS  R25,_x_1_tk+3
	CALL SUBOPT_0x28
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x31
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x31
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x32
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x32
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x33
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x33
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x34
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x34
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x35
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x35
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x36
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x36
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x37
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x37
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x38
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x38
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x39
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x39
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __MULF12
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x30
	STS  _norm_e,R30
	STS  _norm_e+1,R31
	STS  _norm_e+2,R22
	STS  _norm_e+3,R23
; 0000 030B 
; 0000 030C     if((norm_e >= (sigma*norm_epsilon + delta)) && start_flag) // uncomment for ETC
	LDS  R30,_norm_epsilon
	LDS  R31,_norm_epsilon+1
	LDS  R22,_norm_epsilon+2
	LDS  R23,_norm_epsilon+3
	__GETD2N 0x3C03126F
	CALL __MULF12
	__GETD2N 0x3D4CCCCD
	CALL __ADDF12
	LDS  R26,_norm_e
	LDS  R27,_norm_e+1
	LDS  R24,_norm_e+2
	LDS  R25,_norm_e+3
	CALL __CMPF12
	BRLO _0x2E
	SBIC 0x0,1
	RJMP _0x2F
_0x2E:
	RJMP _0x2D
_0x2F:
; 0000 030D     //if(start_flag) // uncomment for periodic TTC
; 0000 030E     {
; 0000 030F 
; 0000 0310         //iterations += 1;
; 0000 0311 
; 0000 0312         //if(iterations == 3)
; 0000 0313         //if(iterations > 0)
; 0000 0314         {
; 0000 0315 
; 0000 0316             /*** Compute the control Signal ***/
; 0000 0317             // Controller For Trolley
; 0000 0318             v_1 = K11*x_1 + K12*x_2 + K13*(x_3) + K14*(x_4) + x_a1;
	CALL SUBOPT_0x3A
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3B
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3C
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3D
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x3E
; 0000 0319             // Note that x_3 and x_4 are not negated opposed to what was suggested in thesis because while hacking crane
; 0000 031A             // we did wirig in way such that signals gets inverted and are already on the form as mentioned in thesis
; 0000 031B 
; 0000 031C             // Controller For Arm
; 0000 031D             v_2 = K21*x_5 + K22*x_6 + K23*x_7 + K24*x_8 + x_a2;
	CALL SUBOPT_0x3F
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x40
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x41
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x42
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x43
; 0000 031E 
; 0000 031F             // Saturate the output control signal
; 0000 0320             // Saturate Trolley Control Signal
; 0000 0321             if(v_1 > 1.0)
	BREQ PC+2
	BRCC PC+3
	JMP  _0x30
; 0000 0322             u_1 = 1.0;
	CALL SUBOPT_0x20
	RJMP _0x13A
; 0000 0323             else if(v_1 < -1.0)
_0x30:
	CALL SUBOPT_0x44
	BRSH _0x32
; 0000 0324             u_1 = -1.0;
	CALL SUBOPT_0x45
	RJMP _0x13A
; 0000 0325             else
_0x32:
; 0000 0326             u_1 = v_1;
	CALL SUBOPT_0x46
_0x13A:
	STS  _u_1,R30
	STS  _u_1+1,R31
	STS  _u_1+2,R22
	STS  _u_1+3,R23
; 0000 0327 
; 0000 0328             // Saturate Arm Control Signal
; 0000 0329             if(v_2 > 1.0)
	CALL SUBOPT_0x47
	CALL SUBOPT_0x20
	CALL __CMPF12
	BREQ PC+2
	BRCC PC+3
	JMP  _0x34
; 0000 032A             u_2 = 1.0;
	CALL SUBOPT_0x20
	RJMP _0x13B
; 0000 032B             else if(v_2 < -1.0)
_0x34:
	CALL SUBOPT_0x48
	BRSH _0x36
; 0000 032C             u_2 = -1.0;
	CALL SUBOPT_0x45
	RJMP _0x13B
; 0000 032D             else
_0x36:
; 0000 032E             u_2 = v_2;
	CALL SUBOPT_0x49
_0x13B:
	STS  _u_2,R30
	STS  _u_2+1,R31
	STS  _u_2+2,R22
	STS  _u_2+3,R23
; 0000 032F 
; 0000 0330             // Flag for Start of Transmission to measure loop Delay
; 0000 0331             //PORTF.OUTTGL = PIN6_bm;
; 0000 0332 
; 0000 0333             /***** SEND THE DATA OVER SERIAL RS232 *****/
; 0000 0334 
; 0000 0335             // Transmit only if it is first transmission or the diference between two consecutive is greater than 30ms
; 0000 0336             if(((iterations - iterations_old) >= 3) || first_transmission)
	LDS  R26,_iterations_old
	LDS  R27,_iterations_old+1
	LDS  R24,_iterations_old+2
	LDS  R25,_iterations_old+3
	CALL SUBOPT_0x2
	CALL __SUBD12
	__CPD1N 0x3
	BRSH _0x39
	SBIS 0x1,0
	RJMP _0x38
_0x39:
; 0000 0337             {
; 0000 0338             // SEND CONTROL DATA IN RAW FROM
; 0000 0339             //Send Control Signal For Trolley, u_1
; 0000 033A             p = (unsigned char *) & u_1;
	__POINTWRM 16,17,_u_1
; 0000 033B             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x3C:
	CPI  R19,4
	BRSH _0x3D
; 0000 033C             {
; 0000 033D              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 033E             }
	SUBI R19,-1
	RJMP _0x3C
_0x3D:
; 0000 033F             // Send Control Signal For Arm, u_2
; 0000 0340             p = (unsigned char *) & u_2;
	__POINTWRM 16,17,_u_2
; 0000 0341             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x3F:
	CPI  R19,4
	BRSH _0x40
; 0000 0342             {
; 0000 0343              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0344             }
	SUBI R19,-1
	RJMP _0x3F
_0x40:
; 0000 0345             // SEND REFERENCE DATA IN RAW FROM
; 0000 0346             //Send Reference Signal For Trolley, r_1
; 0000 0347             p = (unsigned char *) & r_1;
	__POINTWRM 16,17,_r_1
; 0000 0348             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x42:
	CPI  R19,4
	BRSH _0x43
; 0000 0349             {
; 0000 034A              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 034B             }
	SUBI R19,-1
	RJMP _0x42
_0x43:
; 0000 034C             // Send Reference Signal For Arm, r_2
; 0000 034D             p = (unsigned char *) & r_2;
	__POINTWRM 16,17,_r_2
; 0000 034E             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x45:
	CPI  R19,4
	BRSH _0x46
; 0000 034F             {
; 0000 0350              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0351             }
	SUBI R19,-1
	RJMP _0x45
_0x46:
; 0000 0352 
; 0000 0353             // SEND Trolley and Arm Position DATA IN RAW FORM (Just for Debug Purposes)
; 0000 0354             //Send Trolley Data, x_1 and x_2
; 0000 0355             p = (unsigned char *) & x_1;
	__POINTWRM 16,17,_x_1
; 0000 0356             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x48:
	CPI  R19,4
	BRSH _0x49
; 0000 0357             {
; 0000 0358              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0359             }
	SUBI R19,-1
	RJMP _0x48
_0x49:
; 0000 035A             // Send Trolley Speed, x_2
; 0000 035B             p = (unsigned char *) & x_2;
	__POINTWRM 16,17,_x_2
; 0000 035C             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x4B:
	CPI  R19,4
	BRSH _0x4C
; 0000 035D             {
; 0000 035E              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 035F             }
	SUBI R19,-1
	RJMP _0x4B
_0x4C:
; 0000 0360 
; 0000 0361             //Send Arm Data, x_5 and x_6
; 0000 0362             p = (unsigned char *) & x_5;
	__POINTWRM 16,17,_x_5
; 0000 0363             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x4E:
	CPI  R19,4
	BRSH _0x4F
; 0000 0364             {
; 0000 0365              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0366             }
	SUBI R19,-1
	RJMP _0x4E
_0x4F:
; 0000 0367             // Send Arm Speed, x_6
; 0000 0368             p = (unsigned char *) & x_6;
	__POINTWRM 16,17,_x_6
; 0000 0369             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x51:
	CPI  R19,4
	BRSH _0x52
; 0000 036A             {
; 0000 036B              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 036C             }
	SUBI R19,-1
	RJMP _0x51
_0x52:
; 0000 036D             iterations_old = iterations;
	CALL SUBOPT_0x2
	STS  _iterations_old,R30
	STS  _iterations_old+1,R31
	STS  _iterations_old+2,R22
	STS  _iterations_old+3,R23
; 0000 036E             first_transmission = 0;
	CBI  0x1,0
; 0000 036F             // Store the state value at the time of transmission of currrent trigger/event
; 0000 0370             x_1_tk = x_1;
	CALL SUBOPT_0xB
	STS  _x_1_tk,R30
	STS  _x_1_tk+1,R31
	STS  _x_1_tk+2,R22
	STS  _x_1_tk+3,R23
; 0000 0371             x_2_tk = x_2;
	CALL SUBOPT_0x29
	STS  _x_2_tk,R30
	STS  _x_2_tk+1,R31
	STS  _x_2_tk+2,R22
	STS  _x_2_tk+3,R23
; 0000 0372             x_3_tk = x_3;
	CALL SUBOPT_0xE
	STS  _x_3_tk,R30
	STS  _x_3_tk+1,R31
	STS  _x_3_tk+2,R22
	STS  _x_3_tk+3,R23
; 0000 0373             x_4_tk = x_4;
	CALL SUBOPT_0x2A
	STS  _x_4_tk,R30
	STS  _x_4_tk+1,R31
	STS  _x_4_tk+2,R22
	STS  _x_4_tk+3,R23
; 0000 0374             x_a1_tk = x_a1;
	CALL SUBOPT_0x4B
	STS  _x_a1_tk,R30
	STS  _x_a1_tk+1,R31
	STS  _x_a1_tk+2,R22
	STS  _x_a1_tk+3,R23
; 0000 0375             x_5_tk = x_5;
	CALL SUBOPT_0x11
	STS  _x_5_tk,R30
	STS  _x_5_tk+1,R31
	STS  _x_5_tk+2,R22
	STS  _x_5_tk+3,R23
; 0000 0376             x_6_tk = x_6;
	CALL SUBOPT_0x2D
	STS  _x_6_tk,R30
	STS  _x_6_tk+1,R31
	STS  _x_6_tk+2,R22
	STS  _x_6_tk+3,R23
; 0000 0377             x_7_tk = x_7;
	CALL SUBOPT_0x12
	STS  _x_7_tk,R30
	STS  _x_7_tk+1,R31
	STS  _x_7_tk+2,R22
	STS  _x_7_tk+3,R23
; 0000 0378             x_8_tk = x_8;
	CALL SUBOPT_0x2E
	STS  _x_8_tk,R30
	STS  _x_8_tk+1,R31
	STS  _x_8_tk+2,R22
	STS  _x_8_tk+3,R23
; 0000 0379             x_a2_tk = x_a2;
	CALL SUBOPT_0x4C
	STS  _x_a2_tk,R30
	STS  _x_a2_tk+1,R31
	STS  _x_a2_tk+2,R22
	STS  _x_a2_tk+3,R23
; 0000 037A             }
; 0000 037B 
; 0000 037C         }
_0x38:
; 0000 037D 
; 0000 037E     };
_0x2D:
; 0000 037F 
; 0000 0380 }
; 0000 0381 
; 0000 0382 
; 0000 0383 
; 0000 0384 /******* Compute Periodic Controller if it is selected *******/
; 0000 0385 else if(Periodic_ON == 1 && start_flag)  // If periodic Selected
	RJMP _0x55
_0x2A:
	SBIS 0x0,7
	RJMP _0x57
	SBIC 0x0,1
	RJMP _0x58
_0x57:
	RJMP _0x56
_0x58:
; 0000 0386 {
; 0000 0387     /*** Compute the control Signal ***/
; 0000 0388     // Controller For Trolley
; 0000 0389     v_1 = K11*x_1 + K12*x_2 + K13*(x_3) + K14*(x_4) + x_a1;
	CALL SUBOPT_0x3A
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3B
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3C
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3D
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x3E
; 0000 038A     // Note that x_3 and x_4 are not negated opposed to what was suggested in thesis because while hacking crane
; 0000 038B     // we did wirig in way such that signals gets inverted and are already on the form as mentioned in thesis
; 0000 038C     u_11 = K11*x_1 + K12*x_2 + K13*(x_3) + K14*(x_4);
	CALL SUBOPT_0x3A
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3B
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3C
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x3D
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	STS  _u_11,R30
	STS  _u_11+1,R31
	STS  _u_11+2,R22
	STS  _u_11+3,R23
; 0000 038D     // Controller For Arm
; 0000 038E     v_2 = K21*x_5 + K22*x_6 + K23*x_7 + K24*x_8 + x_a2;
	CALL SUBOPT_0x3F
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x40
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x41
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x42
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL SUBOPT_0x43
; 0000 038F 
; 0000 0390     // Saturate the output control signal
; 0000 0391     // Saturate Trolley Control Signal
; 0000 0392     if(v_1 > 1.0)
	BREQ PC+2
	BRCC PC+3
	JMP  _0x59
; 0000 0393     u_1 = 1.0;
	CALL SUBOPT_0x20
	RJMP _0x13C
; 0000 0394     else if(v_1 < -1.0)
_0x59:
	CALL SUBOPT_0x44
	BRSH _0x5B
; 0000 0395     u_1 = -1.0;
	CALL SUBOPT_0x45
	RJMP _0x13C
; 0000 0396     else
_0x5B:
; 0000 0397     u_1 = v_1;
	CALL SUBOPT_0x46
_0x13C:
	STS  _u_1,R30
	STS  _u_1+1,R31
	STS  _u_1+2,R22
	STS  _u_1+3,R23
; 0000 0398 
; 0000 0399     // Saturate Arm Control Signal
; 0000 039A     if(v_2 > 1.0)
	CALL SUBOPT_0x47
	CALL SUBOPT_0x20
	CALL __CMPF12
	BREQ PC+2
	BRCC PC+3
	JMP  _0x5D
; 0000 039B     u_2 = 1.0;
	CALL SUBOPT_0x20
	RJMP _0x13D
; 0000 039C     else if(v_2 < -1.0)
_0x5D:
	CALL SUBOPT_0x48
	BRSH _0x5F
; 0000 039D     u_2 = -1.0;
	CALL SUBOPT_0x45
	RJMP _0x13D
; 0000 039E     else
_0x5F:
; 0000 039F     u_2 = v_2;
	CALL SUBOPT_0x49
_0x13D:
	STS  _u_2,R30
	STS  _u_2+1,R31
	STS  _u_2+2,R22
	STS  _u_2+3,R23
; 0000 03A0 
; 0000 03A1     // Flag for Start of Transmission to measure loop Delay
; 0000 03A2     //PORTF.OUTTGL = PIN6_bm;
; 0000 03A3 
; 0000 03A4     /***** SEND THE DATA OVER SERIAL RS232 *****/
; 0000 03A5 
; 0000 03A6     // Trigger to tell we gonna send the data
; 0000 03A7     // used just for testing purpose
; 0000 03A8     PORTF.OUTTGL = PIN6_bm;
	LDI  R30,LOW(64)
	STS  1703,R30
; 0000 03A9 
; 0000 03AA     // SEND CONTROL DATA IN RAW FROM
; 0000 03AB     //Send Control Signal For Trolley, u_1
; 0000 03AC     p = (unsigned char *) & u_1;
	__POINTWRM 16,17,_u_1
; 0000 03AD     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x62:
	CPI  R19,4
	BRSH _0x63
; 0000 03AE     {
; 0000 03AF      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03B0     }
	SUBI R19,-1
	RJMP _0x62
_0x63:
; 0000 03B1     // Send Control Signal For Arm, u_2
; 0000 03B2     p = (unsigned char *) & u_2;
	__POINTWRM 16,17,_u_2
; 0000 03B3     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x65:
	CPI  R19,4
	BRSH _0x66
; 0000 03B4     {
; 0000 03B5      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03B6     }
	SUBI R19,-1
	RJMP _0x65
_0x66:
; 0000 03B7 
; 0000 03B8 
; 0000 03B9     // SEND REFERENCE DATA IN RAW FROM
; 0000 03BA     //Send Reference Signal For Trolley, r_1
; 0000 03BB     p = (unsigned char *) & r_1;
	__POINTWRM 16,17,_r_1
; 0000 03BC     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x68:
	CPI  R19,4
	BRSH _0x69
; 0000 03BD     {
; 0000 03BE      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03BF     }
	SUBI R19,-1
	RJMP _0x68
_0x69:
; 0000 03C0     // Send Reference Signal For Arm, r_2
; 0000 03C1     p = (unsigned char *) & r_2;
	__POINTWRM 16,17,_r_2
; 0000 03C2     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x6B:
	CPI  R19,4
	BRSH _0x6C
; 0000 03C3     {
; 0000 03C4      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03C5     }
	SUBI R19,-1
	RJMP _0x6B
_0x6C:
; 0000 03C6 
; 0000 03C7     // SEND Trolley and Arm Position DATA IN RAW FORM (Just for Visual Fedback)
; 0000 03C8     //Send Trolley Data, x_1 and x_2
; 0000 03C9     p = (unsigned char *) & x_1;
	__POINTWRM 16,17,_x_1
; 0000 03CA     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x6E:
	CPI  R19,4
	BRSH _0x6F
; 0000 03CB     {
; 0000 03CC      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03CD     }
	SUBI R19,-1
	RJMP _0x6E
_0x6F:
; 0000 03CE     // Send Trolley Speed, x_2
; 0000 03CF     p = (unsigned char *) & x_2;
	__POINTWRM 16,17,_x_2
; 0000 03D0     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x71:
	CPI  R19,4
	BRSH _0x72
; 0000 03D1     {
; 0000 03D2      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03D3     }
	SUBI R19,-1
	RJMP _0x71
_0x72:
; 0000 03D4 
; 0000 03D5     //Send Arm Data, x_5 and x_6
; 0000 03D6     //x_test = (float)Pos_Count_HCTL_1;
; 0000 03D7     p = (unsigned char *) & x_5;
	__POINTWRM 16,17,_x_5
; 0000 03D8     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x74:
	CPI  R19,4
	BRSH _0x75
; 0000 03D9     {
; 0000 03DA      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03DB     }
	SUBI R19,-1
	RJMP _0x74
_0x75:
; 0000 03DC     // Send Arm Speed, x_6
; 0000 03DD     p = (unsigned char *) & x_6;
	__POINTWRM 16,17,_x_6
; 0000 03DE     for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x77:
	CPI  R19,4
	BRSH _0x78
; 0000 03DF     {
; 0000 03E0      putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 03E1     }
	SUBI R19,-1
	RJMP _0x77
_0x78:
; 0000 03E2 
; 0000 03E3 };
_0x56:
_0x55:
; 0000 03E4 
; 0000 03E5 /*********** Estimate Integrator State ***********/
; 0000 03E6 // Note that bi1 = K15*h , bi2 = K25*h , a01 = h/Tt1 and a02 = h/Tt2
; 0000 03E7 if(start_flag == 1)
	SBIS 0x0,1
	RJMP _0x79
; 0000 03E8 {
; 0000 03E9 if((fabs(r_1 - x_1) > 0.005*r_1) || flag_ref ) // epsilon = 0.005  ,,,0.01
	CALL SUBOPT_0x4D
	CALL SUBOPT_0x22
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	LDS  R30,_r_1
	LDS  R31,_r_1+1
	LDS  R22,_r_1+2
	LDS  R23,_r_1+3
	CALL SUBOPT_0x4E
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __CMPF12
	BREQ PC+4
	BRCS PC+3
	JMP  _0x7B
	SBIS 0x1,1
	RJMP _0x7A
_0x7B:
; 0000 03EA x_a1 = x_a1_old + bi1*(r_1 - x_1) + 0.2*(u_1 - v_1); // 1.5 ,,,0.6
	CALL SUBOPT_0x4D
	LDS  R26,_bi1
	LDS  R27,_bi1+1
	LDS  R24,_bi1+2
	LDS  R25,_bi1+3
	CALL __MULF12
	LDS  R26,_x_a1_old
	LDS  R27,_x_a1_old+1
	LDS  R24,_x_a1_old+2
	LDS  R25,_x_a1_old+3
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	LDS  R26,_v_1
	LDS  R27,_v_1+1
	LDS  R24,_v_1+2
	LDS  R25,_v_1+3
	CALL SUBOPT_0x4F
	CALL SUBOPT_0x50
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	STS  _x_a1,R30
	STS  _x_a1+1,R31
	STS  _x_a1+2,R22
	STS  _x_a1+3,R23
; 0000 03EB if((fabs(r_2 - x_5) > 0.005*r_2) || flag_ref ) // there was a mistake here x_2 instead of x_5 was written. epsilon = 0.005,,,0.01
_0x7A:
	CALL SUBOPT_0x51
	CALL SUBOPT_0x22
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	LDS  R30,_r_2
	LDS  R31,_r_2+1
	LDS  R22,_r_2+2
	LDS  R23,_r_2+3
	CALL SUBOPT_0x4E
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __CMPF12
	BREQ PC+4
	BRCS PC+3
	JMP  _0x7E
	SBIS 0x1,1
	RJMP _0x7D
_0x7E:
; 0000 03EC x_a2 = x_a2_old + bi2*(r_2 - x_5) + 0.2*(u_2 - v_2);  //1.5 ,,0.3,,, 0.6 ,,,0.6
	CALL SUBOPT_0x51
	LDS  R26,_bi2
	LDS  R27,_bi2+1
	LDS  R24,_bi2+2
	LDS  R25,_bi2+3
	CALL __MULF12
	LDS  R26,_x_a2_old
	LDS  R27,_x_a2_old+1
	LDS  R24,_x_a2_old+2
	LDS  R25,_x_a2_old+3
	CALL __ADDF12
	PUSH R23
	PUSH R22
	PUSH R31
	PUSH R30
	CALL SUBOPT_0x47
	CALL SUBOPT_0x52
	CALL SUBOPT_0x50
	POP  R26
	POP  R27
	POP  R24
	POP  R25
	CALL __ADDF12
	STS  _x_a2,R30
	STS  _x_a2+1,R31
	STS  _x_a2+2,R22
	STS  _x_a2+3,R23
; 0000 03ED }
_0x7D:
; 0000 03EE 
; 0000 03EF /***** Store Current State for use in next (plant) sampling step *******/
; 0000 03F0 x_1_old = x_1;
_0x79:
	CALL SUBOPT_0xB
	STS  _x_1_old,R30
	STS  _x_1_old+1,R31
	STS  _x_1_old+2,R22
	STS  _x_1_old+3,R23
; 0000 03F1 x_2_old = x_2;
	CALL SUBOPT_0x29
	STS  _x_2_old,R30
	STS  _x_2_old+1,R31
	STS  _x_2_old+2,R22
	STS  _x_2_old+3,R23
; 0000 03F2 x_3_old = x_3;
	CALL SUBOPT_0xE
	STS  _x_3_old,R30
	STS  _x_3_old+1,R31
	STS  _x_3_old+2,R22
	STS  _x_3_old+3,R23
; 0000 03F3 x_4_old = x_4;
	CALL SUBOPT_0x2A
	STS  _x_4_old,R30
	STS  _x_4_old+1,R31
	STS  _x_4_old+2,R22
	STS  _x_4_old+3,R23
; 0000 03F4 x_5_old = x_5;
	CALL SUBOPT_0x11
	STS  _x_5_old,R30
	STS  _x_5_old+1,R31
	STS  _x_5_old+2,R22
	STS  _x_5_old+3,R23
; 0000 03F5 x_6_old = x_6;
	CALL SUBOPT_0x2D
	STS  _x_6_old,R30
	STS  _x_6_old+1,R31
	STS  _x_6_old+2,R22
	STS  _x_6_old+3,R23
; 0000 03F6 x_7_old = x_7;
	CALL SUBOPT_0x12
	STS  _x_7_old,R30
	STS  _x_7_old+1,R31
	STS  _x_7_old+2,R22
	STS  _x_7_old+3,R23
; 0000 03F7 x_8_old = x_8;
	CALL SUBOPT_0x2E
	STS  _x_8_old,R30
	STS  _x_8_old+1,R31
	STS  _x_8_old+2,R22
	STS  _x_8_old+3,R23
; 0000 03F8 u_1_old = u_1;
	CALL SUBOPT_0x4F
	STS  _u_1_old,R30
	STS  _u_1_old+1,R31
	STS  _u_1_old+2,R22
	STS  _u_1_old+3,R23
; 0000 03F9 u_2_old = u_2;
	CALL SUBOPT_0x52
	STS  _u_2_old,R30
	STS  _u_2_old+1,R31
	STS  _u_2_old+2,R22
	STS  _u_2_old+3,R23
; 0000 03FA x_a1_old = x_a1;
	CALL SUBOPT_0x4B
	STS  _x_a1_old,R30
	STS  _x_a1_old+1,R31
	STS  _x_a1_old+2,R22
	STS  _x_a1_old+3,R23
; 0000 03FB x_a2_old = x_a2;
	CALL SUBOPT_0x4C
	STS  _x_a2_old,R30
	STS  _x_a2_old+1,R31
	STS  _x_a2_old+2,R22
	STS  _x_a2_old+3,R23
; 0000 03FC 
; 0000 03FD flag_ref = 0;  // Bring the flag down to make it ready for next reference update
	CBI  0x1,1
; 0000 03FE 
; 0000 03FF //PORTF.OUTTGL = PIN6_bm;
; 0000 0400 
; 0000 0401 }
	RJMP _0x13E
;
;// PORTK interrupt 0 service routine
;interrupt [PORTK_INT0_vect] void portk_int0_isr(void)
; 0000 0405 {
_portk_int0_isr:
	CALL SUBOPT_0x1
; 0000 0406 // write your code here
; 0000 0407 unsigned char * p;
; 0000 0408 unsigned char i;
; 0000 0409 
; 0000 040A // Reset all controller states
; 0000 040B u_1 = 0;
;	*p -> R16,R17
;	i -> R19
	CALL SUBOPT_0x53
; 0000 040C u_2 = 0;
; 0000 040D v_1 = 0;
	CALL SUBOPT_0x54
; 0000 040E v_2 = 0;
; 0000 040F x_a1 = 0;
	CALL SUBOPT_0x55
; 0000 0410 x_a1_old = 0;
; 0000 0411 x_a1_tk = 0;
; 0000 0412 x_a2 = 0;
; 0000 0413 x_a2_old = 0;
; 0000 0414 x_a2_tk = 0;
; 0000 0415 start_flag = 0;
	CBI  0x0,1
; 0000 0416 
; 0000 0417 //Added Later
; 0000 0418 u_1_old = 0;
	LDI  R30,LOW(0)
	CALL SUBOPT_0x56
; 0000 0419 u_2_old = 0;
; 0000 041A 
; 0000 041B 
; 0000 041C // SEND DATA IN RAW FROM
; 0000 041D //Send Control Signal For Trolley, u_1
; 0000 041E p = (unsigned char *) & u_1;
	__POINTWRM 16,17,_u_1
; 0000 041F for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x85:
	CPI  R19,4
	BRSH _0x86
; 0000 0420 {
; 0000 0421 putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0422 }
	SUBI R19,-1
	RJMP _0x85
_0x86:
; 0000 0423 
; 0000 0424 // Send Control Signal For Arm, u_2
; 0000 0425 p = (unsigned char *) & u_2;
	__POINTWRM 16,17,_u_2
; 0000 0426 for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x88:
	CPI  R19,4
	BRSH _0x89
; 0000 0427 {
; 0000 0428 putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0429 }
	SUBI R19,-1
	RJMP _0x88
_0x89:
; 0000 042A 
; 0000 042B // SEND REFERENCE DATA IN RAW FROM
; 0000 042C //Send Reference Signal For Trolley, r_1
; 0000 042D p = (unsigned char *) & r_1;
	__POINTWRM 16,17,_r_1
; 0000 042E for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x8B:
	CPI  R19,4
	BRSH _0x8C
; 0000 042F {
; 0000 0430  putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0431 }
	SUBI R19,-1
	RJMP _0x8B
_0x8C:
; 0000 0432 // Send Reference Signal For Arm, r_2
; 0000 0433 p = (unsigned char *) & r_2;
	__POINTWRM 16,17,_r_2
; 0000 0434 for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x8E:
	CPI  R19,4
	BRSH _0x8F
; 0000 0435 {
; 0000 0436  putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0437 }
	SUBI R19,-1
	RJMP _0x8E
_0x8F:
; 0000 0438 
; 0000 0439 // SEND Trolley and Arm Position DATA as well to maintain the same data length
; 0000 043A //Send Trolley Data, x_1 and x_2
; 0000 043B p = (unsigned char *) & x_1;
	__POINTWRM 16,17,_x_1
; 0000 043C for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x91:
	CPI  R19,4
	BRSH _0x92
; 0000 043D {
; 0000 043E  putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 043F }
	SUBI R19,-1
	RJMP _0x91
_0x92:
; 0000 0440 // Send Trolley Speed, x_2
; 0000 0441 p = (unsigned char *) & x_2;
	__POINTWRM 16,17,_x_2
; 0000 0442 for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x94:
	CPI  R19,4
	BRSH _0x95
; 0000 0443 {
; 0000 0444  putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0445 }
	SUBI R19,-1
	RJMP _0x94
_0x95:
; 0000 0446 
; 0000 0447 //Send Arm Data, x_5 and x_6
; 0000 0448 p = (unsigned char *) & x_5;
	__POINTWRM 16,17,_x_5
; 0000 0449 for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x97:
	CPI  R19,4
	BRSH _0x98
; 0000 044A {
; 0000 044B  putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 044C }
	SUBI R19,-1
	RJMP _0x97
_0x98:
; 0000 044D // Send Arm Speed, x_6
; 0000 044E p = (unsigned char *) & x_6;
	__POINTWRM 16,17,_x_6
; 0000 044F for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x9A:
	CPI  R19,4
	BRSH _0x9B
; 0000 0450 {
; 0000 0451  putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0452 }
	SUBI R19,-1
	RJMP _0x9A
_0x9B:
; 0000 0453 
; 0000 0454 }
_0x13E:
	CALL __LOADLOCR4
	ADIW R28,4
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	LD   R25,Y+
	LD   R24,Y+
	LD   R23,Y+
	LD   R22,Y+
	LD   R15,Y+
	LD   R1,Y+
	LD   R0,Y+
	RETI
;
;// Ports initialization
;void ports_init(void)
; 0000 0458 {
_ports_init:
; 0000 0459 // PORTF initialization
; 0000 045A // OUT register
; 0000 045B PORTF.OUT=0x08;
	LDI  R30,LOW(8)
	STS  1700,R30
; 0000 045C // Bit0: Input
; 0000 045D // Bit1: Input
; 0000 045E // Bit2: Input
; 0000 045F // Bit3: Output
; 0000 0460 // Bit4: Input
; 0000 0461 // Bit5: Input
; 0000 0462 // Bit6: Input
; 0000 0463 // Bit7: Input
; 0000 0464 PORTF.DIR=0x08;
	STS  1696,R30
; 0000 0465 // Bit0 Output/Pull configuration: Totempole/No
; 0000 0466 // Bit0 Input/Sense configuration: Sense both edges
; 0000 0467 // Bit0 inverted: Off
; 0000 0468 // Bit0 slew rate limitation: Off
; 0000 0469 PORTF.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	LDI  R30,LOW(0)
	STS  1712,R30
; 0000 046A // Bit1 Output/Pull configuration: Totempole/No
; 0000 046B // Bit1 Input/Sense configuration: Sense both edges
; 0000 046C // Bit1 inverted: Off
; 0000 046D // Bit1 slew rate limitation: Off
; 0000 046E PORTF.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	STS  1713,R30
; 0000 046F // Bit2 Output/Pull configuration: Totempole/No
; 0000 0470 // Bit2 Input/Sense configuration: Sense both edges
; 0000 0471 // Bit2 inverted: Off
; 0000 0472 // Bit2 slew rate limitation: Off
; 0000 0473 PORTF.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	STS  1714,R30
; 0000 0474 // Bit3 Output/Pull configuration: Totempole/No
; 0000 0475 // Bit3 Input/Sense configuration: Sense both edges
; 0000 0476 // Bit3 inverted: Off
; 0000 0477 // Bit3 slew rate limitation: Off
; 0000 0478 PORTF.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	STS  1715,R30
; 0000 0479 // Bit4 Output/Pull configuration: Totempole/No
; 0000 047A // Bit4 Input/Sense configuration: Sense both edges
; 0000 047B // Bit4 inverted: Off
; 0000 047C // Bit4 slew rate limitation: Off
; 0000 047D PORTF.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	STS  1716,R30
; 0000 047E // Bit5 Output/Pull configuration: Totempole/No
; 0000 047F // Bit5 Input/Sense configuration: Sense both edges
; 0000 0480 // Bit5 inverted: Off
; 0000 0481 // Bit5 slew rate limitation: Off
; 0000 0482 PORTF.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	STS  1717,R30
; 0000 0483 // Bit6 Output/Pull configuration: Totempole/No
; 0000 0484 // Bit6 Input/Sense configuration: Sense both edges
; 0000 0485 // Bit6 inverted: Off
; 0000 0486 // Bit6 slew rate limitation: Off
; 0000 0487 PORTF.PIN6CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	STS  1718,R30
; 0000 0488 // Bit7 Output/Pull configuration: Totempole/No
; 0000 0489 // Bit7 Input/Sense configuration: Sense both edges
; 0000 048A // Bit7 inverted: Off
; 0000 048B // Bit7 slew rate limitation: Off
; 0000 048C PORTF.PIN7CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	STS  1719,R30
; 0000 048D // Interrupt 0 level: Disabled
; 0000 048E // Interrupt 1 level: Disabled
; 0000 048F PORTF.INTCTRL=(PORTF.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
; 0000 0490     PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	LDS  R30,1705
	ANDI R30,LOW(0xF0)
	STS  1705,R30
; 0000 0491 // Bit0 pin change interrupt 0: Off
; 0000 0492 // Bit1 pin change interrupt 0: Off
; 0000 0493 // Bit2 pin change interrupt 0: Off
; 0000 0494 // Bit3 pin change interrupt 0: Off
; 0000 0495 // Bit4 pin change interrupt 0: Off
; 0000 0496 // Bit5 pin change interrupt 0: Off
; 0000 0497 // Bit6 pin change interrupt 0: Off
; 0000 0498 // Bit7 pin change interrupt 0: Off
; 0000 0499 PORTF.INT0MASK=0x00;
	LDI  R30,LOW(0)
	STS  1706,R30
; 0000 049A // Bit0 pin change interrupt 1: Off
; 0000 049B // Bit1 pin change interrupt 1: Off
; 0000 049C // Bit2 pin change interrupt 1: Off
; 0000 049D // Bit3 pin change interrupt 1: Off
; 0000 049E // Bit4 pin change interrupt 1: Off
; 0000 049F // Bit5 pin change interrupt 1: Off
; 0000 04A0 // Bit6 pin change interrupt 1: Off
; 0000 04A1 // Bit7 pin change interrupt 1: Off
; 0000 04A2 PORTF.INT1MASK=0x00;
	STS  1707,R30
; 0000 04A3 
; 0000 04A4 // Set Pin6 as output for signalling start of transmisson of packet
; 0000 04A5 // this is meant to compute the loop delay
; 0000 04A6 PORTF.DIRSET = PIN6_bm; // Set PIN6 as output
	LDI  R30,LOW(64)
	STS  1697,R30
; 0000 04A7 PORTF.OUTCLR = PIN6_bm; // set it zero initially
	STS  1702,R30
; 0000 04A8 
; 0000 04A9 // Pin configuration for Switch Port
; 0000 04AA //Configure all as Input
; 0000 04AB PORTK.DIRCLR = 0xFF;
	LDI  R30,LOW(255)
	STS  1826,R30
; 0000 04AC //Enable Interrupt 0 on PIN2
; 0000 04AD PORTK.PIN2CTRL = (PORTK.PIN2CTRL & (~(PORT_ISC_gm | PORT_OPC_gm))) | PORT_ISC_FALLING_gc | PORT_OPC_PULLUP_gc;
	LDS  R30,1842
	ANDI R30,LOW(0xC0)
	ORI  R30,LOW(0x1A)
	STS  1842,R30
; 0000 04AE /* Mask Interrupt 0 for PIN7*/
; 0000 04AF PORTK.INT0MASK |= PIN2_bm;
	LDS  R30,1834
	ORI  R30,4
	STS  1834,R30
; 0000 04B0 /* Enable the interrupt 0 and mark it as high level*/
; 0000 04B1 PORTK.INTCTRL = (PORTK.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) | PORT_INT1LVL_OFF_gc | PORT_INT0LVL_HI_gc;
	LDS  R30,1833
	ANDI R30,LOW(0xF0)
	ORI  R30,LOW(0x3)
	STS  1833,R30
; 0000 04B2 }
	RET
;
;void estimation_param_init(void)
; 0000 04B5 {
_estimation_param_init:
; 0000 04B6 /* Calculate Estimation Parameters */
; 0000 04B7 bi1 = K15*h;
	CALL SUBOPT_0x16
	__GETD2N 0x40080000
	CALL __MULF12
	STS  _bi1,R30
	STS  _bi1+1,R31
	STS  _bi1+2,R22
	STS  _bi1+3,R23
; 0000 04B8 bi2 = K25*h;
	CALL SUBOPT_0x16
	__GETD2N 0x3F4CCCCD
	CALL __MULF12
	STS  _bi2,R30
	STS  _bi2+1,R31
	STS  _bi2+2,R22
	STS  _bi2+3,R23
; 0000 04B9 a01 = h/Tt1;
	CALL SUBOPT_0x57
	STS  _a01,R30
	STS  _a01+1,R31
	STS  _a01+2,R22
	STS  _a01+3,R23
; 0000 04BA a02 = h/Tt2;
	CALL SUBOPT_0x57
	STS  _a02,R30
	STS  _a02+1,R31
	STS  _a02+2,R22
	STS  _a02+3,R23
; 0000 04BB ad = 1.0/(1.0 + (float)N*h);
	CALL SUBOPT_0x58
	CALL SUBOPT_0x15
	STS  _ad,R30
	STS  _ad+1,R31
	STS  _ad+2,R22
	STS  _ad+3,R23
; 0000 04BC bd = (float)N/(1.0 + (float)N*h);
	CALL SUBOPT_0x58
	CALL __ADDF12
	__GETD2N 0x41200000
	CALL __DIVF21
	STS  _bd,R30
	STS  _bd+1,R31
	STS  _bd+2,R22
	STS  _bd+3,R23
; 0000 04BD 
; 0000 04BE ad1 = 1.0/(1.0 + (float)N1*h);
	CALL SUBOPT_0x59
	CALL SUBOPT_0x15
	STS  _ad1,R30
	STS  _ad1+1,R31
	STS  _ad1+2,R22
	STS  _ad1+3,R23
; 0000 04BF bd1 = (float)N1/(1.0 + (float)N1*h);
	CALL SUBOPT_0x59
	CALL __ADDF12
	__GETD2N 0x40A00000
	CALL __DIVF21
	STS  _bd1,R30
	STS  _bd1+1,R31
	STS  _bd1+2,R22
	STS  _bd1+3,R23
; 0000 04C0 }
	RET
;
;void reset_controller(void)
; 0000 04C3 {
_reset_controller:
; 0000 04C4 
; 0000 04C5 // RESET INTEGRATOR STATE and All Controller States
; 0000 04C6 x_a1 = 0.0;
	CALL SUBOPT_0x55
; 0000 04C7 x_a1_old = 0.0;
; 0000 04C8 x_a1_tk = 0.0;
; 0000 04C9 x_a2 = 0.0;
; 0000 04CA x_a2_old = 0.0;
; 0000 04CB x_a2_tk = 0.0;
; 0000 04CC x_1_old = 0;
	CALL SUBOPT_0x5A
; 0000 04CD x_2_old = 0;
; 0000 04CE x_3_old = 0;
; 0000 04CF x_4_old = 0;
; 0000 04D0 x_5_old = 0;
; 0000 04D1 x_6_old = 0;
; 0000 04D2 x_7_old = 0;
; 0000 04D3 x_8_old = 0;
; 0000 04D4 
; 0000 04D5 u_1 = 0;
; 0000 04D6 u_2 = 0;
; 0000 04D7 u_1_old = 0;
	CALL SUBOPT_0x56
; 0000 04D8 u_2_old = 0;
; 0000 04D9 v_1 = 0;
	LDI  R30,LOW(0)
	CALL SUBOPT_0x54
; 0000 04DA v_2 = 0;
; 0000 04DB }
	RET
;
;
;void main( void )
; 0000 04DF {
_main:
; 0000 04E0     unsigned char * p;
; 0000 04E1     unsigned char i;
; 0000 04E2     // System clocks initialization
; 0000 04E3     #ifdef SYSTEM_CLOCK_32MHZ
; 0000 04E4     system_clocks_init_32mhz();
;	*p -> R16,R17
;	i -> R19
	CALL _system_clocks_init_32mhz
; 0000 04E5     #else
; 0000 04E6     system_clocks_init();
; 0000 04E7     #endif
; 0000 04E8 
; 0000 04E9     /** Enable Quadrature Decoder (QD) Board **/
; 0000 04EA     PORTJ.DIRSET = EN_max3002;
	LDI  R30,LOW(1)
	STS  1793,R30
; 0000 04EB     PORTJ.OUTSET = EN_max3002;
	STS  1797,R30
; 0000 04EC 
; 0000 04ED 
; 0000 04EE 
; 0000 04EF     // USARTF0 initialization
; 0000 04F0     usartf0_init();
	CALL _usartf0_init
; 0000 04F1 
; 0000 04F2 
; 0000 04F3     // Timer/Counter TCD0 initialization
; 0000 04F4     tcd0_init();
	CALL _tcd0_init
; 0000 04F5 
; 0000 04F6     // Port F initialization for serial transmission and PORTK for hardware external interrupt
; 0000 04F7     ports_init();
	RCALL _ports_init
; 0000 04F8 
; 0000 04F9 
; 0000 04FA     /* Calculate Estimation Parameters */
; 0000 04FB     estimation_param_init();
	RCALL _estimation_param_init
; 0000 04FC 
; 0000 04FD 
; 0000 04FE     /** Setup for Reading Encoder Data from QD (Quadrature Decoder Board)***/
; 0000 04FF     /* Setup HCTL2032 Quadrature Decoder for Encoder 1 and 2 */
; 0000 0500     HCTL2032_Total_Setup(&PORTD,                        /* The port to use for 8 bit data.*/
; 0000 0501                       &PORTE,                           /*The port to use for controlling HCTL2032.*/
; 0000 0502                       &PORTJ,                           /*The Port which would read the signals from HCTL2032. These signals are kind of reporting signals
; 0000 0503                                                         which tells about internal state of HCTL2032 like overflow/underflow, count updated etc*/
; 0000 0504                       CountMode);                       /*Which count mode. CountMode = 1 => 4x, CountMode = 2 => 2x and CountMode = 3 => 1x.*/
	CALL SUBOPT_0x3
	LDI  R30,LOW(1792)
	LDI  R31,HIGH(1792)
	ST   -Y,R31
	ST   -Y,R30
	ST   -Y,R6
	CALL _HCTL2032_Total_Setup
; 0000 0505 
; 0000 0506      /* Setup HCTL2032 Quadrature Decoder for Encoder 3 and 4 */
; 0000 0507 
; 0000 0508     HCTL2032_Total_Setup(&PORTH,                        /* The port to use for 8 bit data.*/
; 0000 0509                       &PORTC,                           /*The port to use for controlling HCTL2032.*/
; 0000 050A                       &PORTK,                           /*The Port which would read the signals from HCTL2032. These signals are kind of reporting signals
; 0000 050B                                                         which tells about internal state of HCTL2032 like overflow/underflow, count updated etc*/
; 0000 050C                       CountMode);                       /*Which count mode. CountMode = 1 => 4x, CountMode = 2 => 2x and CountMode = 3 => 1x.*/
	CALL SUBOPT_0x4
	LDI  R30,LOW(1824)
	LDI  R31,HIGH(1824)
	ST   -Y,R31
	ST   -Y,R30
	ST   -Y,R6
	CALL _HCTL2032_Total_Setup
; 0000 050D 
; 0000 050E 
; 0000 050F     /* Enable low level and medium level interrupt.*/
; 0000 0510     PMIC.CTRL |= (PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);
	LDS  R30,162
	ORI  R30,LOW(0x5)
	STS  162,R30
; 0000 0511     /* Enable global interrupts.*/
; 0000 0512     sei();
	sei
; 0000 0513 
; 0000 0514 
; 0000 0515     /* Display the frequency of rotation on LEDs */
; 0000 0516 while (1)
_0x9C:
; 0000 0517 {
; 0000 0518 
; 0000 0519     // Place your code here
; 0000 051A     /*** SWITCH INTERFACE ****/
; 0000 051B     PORTK.DIRCLR = 0xFF;
	LDI  R30,LOW(255)
	STS  1826,R30
; 0000 051C 
; 0000 051D     /*** START THE CRANE OPERATION with ETC as default WHEN SW7 IS PRESSED ****/
; 0000 051E     if((~PORTK.IN) & PIN7_bm)
	LDS  R30,1832
	COM  R30
	ANDI R30,LOW(0x80)
	BREQ _0x9F
; 0000 051F     {
; 0000 0520         r_1 = home_x;//0.25;//0.3;
	CALL SUBOPT_0x5B
; 0000 0521         r_2 = home_theta;//PI*0.5;
	CALL SUBOPT_0x5C
; 0000 0522         count_ref = 0;
; 0000 0523         first_transmission = 1; // this is first transmission
	SBI  0x1,0
; 0000 0524         // RESET INTEGRATOR STATE and All Controller States
; 0000 0525         x_a1 = 0.0;
	CALL SUBOPT_0x55
; 0000 0526         x_a1_old = 0.0;
; 0000 0527         x_a1_tk = 0.0;
; 0000 0528         x_a2 = 0.0;
; 0000 0529         x_a2_old = 0.0;
; 0000 052A         x_a2_tk = 0.0;
; 0000 052B         x_1_old = 0;
	CALL SUBOPT_0x5A
; 0000 052C         x_2_old = 0;
; 0000 052D         x_3_old = 0;
; 0000 052E         x_4_old = 0;
; 0000 052F         x_5_old = 0;
; 0000 0530         x_6_old = 0;
; 0000 0531         x_7_old = 0;
; 0000 0532         x_8_old = 0;
; 0000 0533 
; 0000 0534         u_1 = 0;
; 0000 0535         u_2 = 0;
; 0000 0536         u_1_old = 0;
	CALL SUBOPT_0x56
; 0000 0537         u_2_old = 0;
; 0000 0538         v_1 = 0;
	LDI  R30,LOW(0)
	CALL SUBOPT_0x54
; 0000 0539         v_2 = 0;
; 0000 053A 
; 0000 053B         TCD0.CNT = 0x0000; // Reset the counter
	CALL SUBOPT_0x0
; 0000 053C         TCD0.PER= 0x1388; //Clock/(4*5000) = 100 Hz => h = 10 ms  , for 32Mhz (clock/(64*5000) = 100 Hz
; 0000 053D         h = 0.01;
	CALL SUBOPT_0x5D
; 0000 053E         estimation_param_init(); // update estimation parameters
; 0000 053F 
; 0000 0540         PORTF.OUTCLR = PIN6_bm; // It is a signal being used for measuring times for different tasks, keep it at zero initially,,
	LDI  R30,LOW(64)
	STS  1702,R30
; 0000 0541         HCTL2032_Reset_Counter(&PORTE);
	CALL SUBOPT_0x5E
; 0000 0542         HCTL2032_Reset_Counter(&PORTC);
; 0000 0543 
; 0000 0544         // Now START CALCULATING INTEGRATOR STATE and Controller
; 0000 0545         Periodic_ON = 0; // Turn On ETC
	CBI  0x0,7
; 0000 0546         start_flag = 1;
	SBI  0x0,1
; 0000 0547 
; 0000 0548 
; 0000 0549     }
; 0000 054A 
; 0000 054B     /**** Manual Controller ****/
; 0000 054C     /** Move Trolley Forward **/
; 0000 054D     // SWx_pressed is being used to conditionally send the control data on serial link. It is important otherwise
; 0000 054E     // if we pressed the button continuously then we would be sending same u continuously which is not required!
; 0000 054F     if((~PORTK.IN) & PIN0_bm)
_0x9F:
	LDS  R30,1832
	COM  R30
	ANDI R30,LOW(0x1)
	BRNE PC+3
	JMP _0xA6
; 0000 0550     {
; 0000 0551         start_flag = 0;
	CBI  0x0,1
; 0000 0552         u_1 = 0.5;
	__GETD1N 0x3F000000
	CALL SUBOPT_0x5F
; 0000 0553         u_2 = 0;
; 0000 0554         if(SW0_pressed)
	SBIS 0x0,2
	RJMP _0xA9
; 0000 0555         {
; 0000 0556             // SEND DATA IN RAW FROM
; 0000 0557             //Send Control Signal For Trolley, u_1
; 0000 0558             p = (unsigned char *) & u_1;
	__POINTWRM 16,17,_u_1
; 0000 0559             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xAB:
	CPI  R19,4
	BRSH _0xAC
; 0000 055A             {
; 0000 055B             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 055C             }
	SUBI R19,-1
	RJMP _0xAB
_0xAC:
; 0000 055D 
; 0000 055E             // Send Control Signal For Arm, u_2
; 0000 055F             p = (unsigned char *) & u_2;
	__POINTWRM 16,17,_u_2
; 0000 0560             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xAE:
	CPI  R19,4
	BRSH _0xAF
; 0000 0561             {
; 0000 0562             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0563             }
	SUBI R19,-1
	RJMP _0xAE
_0xAF:
; 0000 0564 
; 0000 0565             // SEND REFERENCE DATA IN RAW FROM
; 0000 0566             //Send Reference Signal For Trolley, r_1
; 0000 0567             p = (unsigned char *) & r_1;
	__POINTWRM 16,17,_r_1
; 0000 0568             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xB1:
	CPI  R19,4
	BRSH _0xB2
; 0000 0569             {
; 0000 056A              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 056B             }
	SUBI R19,-1
	RJMP _0xB1
_0xB2:
; 0000 056C             // Send Reference Signal For Arm, r_2
; 0000 056D             p = (unsigned char *) & r_2;
	__POINTWRM 16,17,_r_2
; 0000 056E             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xB4:
	CPI  R19,4
	BRSH _0xB5
; 0000 056F             {
; 0000 0570              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0571             }
	SUBI R19,-1
	RJMP _0xB4
_0xB5:
; 0000 0572 
; 0000 0573             // SEND Trolley and Arm Position DATA as well to maintain the same data length
; 0000 0574             //Send Trolley Data, x_1 and x_2
; 0000 0575             p = (unsigned char *) & x_1;
	__POINTWRM 16,17,_x_1
; 0000 0576             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xB7:
	CPI  R19,4
	BRSH _0xB8
; 0000 0577             {
; 0000 0578              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0579             }
	SUBI R19,-1
	RJMP _0xB7
_0xB8:
; 0000 057A             // Send Trolley Speed, x_2
; 0000 057B             p = (unsigned char *) & x_2;
	__POINTWRM 16,17,_x_2
; 0000 057C             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xBA:
	CPI  R19,4
	BRSH _0xBB
; 0000 057D             {
; 0000 057E              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 057F             }
	SUBI R19,-1
	RJMP _0xBA
_0xBB:
; 0000 0580 
; 0000 0581             //Send Arm Data, x_5 and x_6
; 0000 0582             p = (unsigned char *) & x_5;
	__POINTWRM 16,17,_x_5
; 0000 0583             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xBD:
	CPI  R19,4
	BRSH _0xBE
; 0000 0584             {
; 0000 0585              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0586             }
	SUBI R19,-1
	RJMP _0xBD
_0xBE:
; 0000 0587             // Send Arm Speed, x_6
; 0000 0588             p = (unsigned char *) & x_6;
	__POINTWRM 16,17,_x_6
; 0000 0589             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xC0:
	CPI  R19,4
	BRSH _0xC1
; 0000 058A             {
; 0000 058B              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 058C             }
	SUBI R19,-1
	RJMP _0xC0
_0xC1:
; 0000 058D         }
; 0000 058E         // if SW is being continuously pressed then don't send control repeateadly
; 0000 058F         SW0_pressed = 0;
_0xA9:
	CBI  0x0,2
; 0000 0590     }
; 0000 0591 
; 0000 0592     else
	RJMP _0xC4
_0xA6:
; 0000 0593     {
; 0000 0594     	SW0_pressed = 1;
	SBI  0x0,2
; 0000 0595     }
_0xC4:
; 0000 0596 
; 0000 0597     /** Move Trolley Backward **/
; 0000 0598     if((~PORTK.IN) & PIN1_bm)
	LDS  R30,1832
	COM  R30
	ANDI R30,LOW(0x2)
	BRNE PC+3
	JMP _0xC7
; 0000 0599     {
; 0000 059A         start_flag = 0;
	CBI  0x0,1
; 0000 059B         u_1 = -0.5;
	__GETD1N 0xBF000000
	CALL SUBOPT_0x5F
; 0000 059C         u_2 = 0;
; 0000 059D         if(SW1_pressed)
	SBIS 0x0,3
	RJMP _0xCA
; 0000 059E         {
; 0000 059F             // SEND DATA IN RAW FROM
; 0000 05A0             //Send Control Signal For Trolley, u_1
; 0000 05A1             p = (unsigned char *) & u_1;
	__POINTWRM 16,17,_u_1
; 0000 05A2             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xCC:
	CPI  R19,4
	BRSH _0xCD
; 0000 05A3             {
; 0000 05A4             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05A5             }
	SUBI R19,-1
	RJMP _0xCC
_0xCD:
; 0000 05A6 
; 0000 05A7             // Send Control Signal For Arm, u_2
; 0000 05A8             p = (unsigned char *) & u_2;
	__POINTWRM 16,17,_u_2
; 0000 05A9             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xCF:
	CPI  R19,4
	BRSH _0xD0
; 0000 05AA             {
; 0000 05AB             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05AC             }
	SUBI R19,-1
	RJMP _0xCF
_0xD0:
; 0000 05AD 
; 0000 05AE             // SEND REFERENCE DATA IN RAW FROM
; 0000 05AF             //Send Reference Signal For Trolley, r_1
; 0000 05B0             p = (unsigned char *) & r_1;
	__POINTWRM 16,17,_r_1
; 0000 05B1             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xD2:
	CPI  R19,4
	BRSH _0xD3
; 0000 05B2             {
; 0000 05B3              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05B4             }
	SUBI R19,-1
	RJMP _0xD2
_0xD3:
; 0000 05B5             // Send Reference Signal For Arm, r_2
; 0000 05B6             p = (unsigned char *) & r_2;
	__POINTWRM 16,17,_r_2
; 0000 05B7             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xD5:
	CPI  R19,4
	BRSH _0xD6
; 0000 05B8             {
; 0000 05B9              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05BA             }
	SUBI R19,-1
	RJMP _0xD5
_0xD6:
; 0000 05BB 
; 0000 05BC             // SEND Trolley and Arm Position DATA as well to maintain the same data length
; 0000 05BD             //Send Trolley Data, x_1 and x_2
; 0000 05BE             p = (unsigned char *) & x_1;
	__POINTWRM 16,17,_x_1
; 0000 05BF             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xD8:
	CPI  R19,4
	BRSH _0xD9
; 0000 05C0             {
; 0000 05C1              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05C2             }
	SUBI R19,-1
	RJMP _0xD8
_0xD9:
; 0000 05C3             // Send Trolley Speed, x_2
; 0000 05C4             p = (unsigned char *) & x_2;
	__POINTWRM 16,17,_x_2
; 0000 05C5             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xDB:
	CPI  R19,4
	BRSH _0xDC
; 0000 05C6             {
; 0000 05C7              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05C8             }
	SUBI R19,-1
	RJMP _0xDB
_0xDC:
; 0000 05C9 
; 0000 05CA             //Send Arm Data, x_5 and x_6
; 0000 05CB             p = (unsigned char *) & x_5;
	__POINTWRM 16,17,_x_5
; 0000 05CC             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xDE:
	CPI  R19,4
	BRSH _0xDF
; 0000 05CD             {
; 0000 05CE              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05CF             }
	SUBI R19,-1
	RJMP _0xDE
_0xDF:
; 0000 05D0             // Send Arm Speed, x_6
; 0000 05D1             p = (unsigned char *) & x_6;
	__POINTWRM 16,17,_x_6
; 0000 05D2             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xE1:
	CPI  R19,4
	BRSH _0xE2
; 0000 05D3             {
; 0000 05D4              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05D5             }
	SUBI R19,-1
	RJMP _0xE1
_0xE2:
; 0000 05D6         }
; 0000 05D7         // if SW is being continuously pressed then don't send control repeateadly
; 0000 05D8         SW1_pressed = 0;
_0xCA:
	CBI  0x0,3
; 0000 05D9     }
; 0000 05DA     else
	RJMP _0xE5
_0xC7:
; 0000 05DB     {
; 0000 05DC     	SW1_pressed = 1;
	SBI  0x0,3
; 0000 05DD     }
_0xE5:
; 0000 05DE 
; 0000 05DF     /** Move Arm Forward **/
; 0000 05E0     if((~PORTK.IN) & PIN3_bm)
	LDS  R30,1832
	COM  R30
	ANDI R30,LOW(0x8)
	BRNE PC+3
	JMP _0xE8
; 0000 05E1     {
; 0000 05E2         start_flag = 0;
	CALL SUBOPT_0x60
; 0000 05E3         u_1 = 0;
; 0000 05E4         u_2 = 0.5;
	__GETD1N 0x3F000000
	CALL SUBOPT_0x61
; 0000 05E5         if(SW3_pressed)
	SBIS 0x0,5
	RJMP _0xEB
; 0000 05E6         {
; 0000 05E7             // SEND DATA IN RAW FROM
; 0000 05E8             //Send Control Signal For Trolley, u_1
; 0000 05E9             p = (unsigned char *) & u_1;
	__POINTWRM 16,17,_u_1
; 0000 05EA             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xED:
	CPI  R19,4
	BRSH _0xEE
; 0000 05EB             {
; 0000 05EC             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05ED             }
	SUBI R19,-1
	RJMP _0xED
_0xEE:
; 0000 05EE 
; 0000 05EF             // Send Control Signal For Arm, u_2
; 0000 05F0             p = (unsigned char *) & u_2;
	__POINTWRM 16,17,_u_2
; 0000 05F1             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xF0:
	CPI  R19,4
	BRSH _0xF1
; 0000 05F2             {
; 0000 05F3             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05F4             }
	SUBI R19,-1
	RJMP _0xF0
_0xF1:
; 0000 05F5 
; 0000 05F6             // SEND REFERENCE DATA IN RAW FROM
; 0000 05F7             //Send Reference Signal For Trolley, r_1
; 0000 05F8             p = (unsigned char *) & r_1;
	__POINTWRM 16,17,_r_1
; 0000 05F9             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xF3:
	CPI  R19,4
	BRSH _0xF4
; 0000 05FA             {
; 0000 05FB              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 05FC             }
	SUBI R19,-1
	RJMP _0xF3
_0xF4:
; 0000 05FD             // Send Reference Signal For Arm, r_2
; 0000 05FE             p = (unsigned char *) & r_2;
	__POINTWRM 16,17,_r_2
; 0000 05FF             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xF6:
	CPI  R19,4
	BRSH _0xF7
; 0000 0600             {
; 0000 0601              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0602             }
	SUBI R19,-1
	RJMP _0xF6
_0xF7:
; 0000 0603 
; 0000 0604             // SEND Trolley and Arm Position DATA as well to maintain the same data length
; 0000 0605             //Send Trolley Data, x_1 and x_2
; 0000 0606             p = (unsigned char *) & x_1;
	__POINTWRM 16,17,_x_1
; 0000 0607             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xF9:
	CPI  R19,4
	BRSH _0xFA
; 0000 0608             {
; 0000 0609              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 060A             }
	SUBI R19,-1
	RJMP _0xF9
_0xFA:
; 0000 060B             // Send Trolley Speed, x_2
; 0000 060C             p = (unsigned char *) & x_2;
	__POINTWRM 16,17,_x_2
; 0000 060D             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xFC:
	CPI  R19,4
	BRSH _0xFD
; 0000 060E             {
; 0000 060F              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0610             }
	SUBI R19,-1
	RJMP _0xFC
_0xFD:
; 0000 0611 
; 0000 0612             //Send Arm Data, x_5 and x_6
; 0000 0613             p = (unsigned char *) & x_5;
	__POINTWRM 16,17,_x_5
; 0000 0614             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0xFF:
	CPI  R19,4
	BRSH _0x100
; 0000 0615             {
; 0000 0616              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0617             }
	SUBI R19,-1
	RJMP _0xFF
_0x100:
; 0000 0618             // Send Arm Speed, x_6
; 0000 0619             p = (unsigned char *) & x_6;
	__POINTWRM 16,17,_x_6
; 0000 061A             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x102:
	CPI  R19,4
	BRSH _0x103
; 0000 061B             {
; 0000 061C              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 061D             }
	SUBI R19,-1
	RJMP _0x102
_0x103:
; 0000 061E         }
; 0000 061F         // if SW is being continuously pressed then don't send control repeateadly
; 0000 0620         SW3_pressed = 0;
_0xEB:
	CBI  0x0,5
; 0000 0621     }
; 0000 0622 
; 0000 0623     else
	RJMP _0x106
_0xE8:
; 0000 0624     {
; 0000 0625     	SW3_pressed = 1;
	SBI  0x0,5
; 0000 0626     }
_0x106:
; 0000 0627 
; 0000 0628     /** Move Arm Backward **/
; 0000 0629     if((~PORTK.IN) & PIN4_bm)
	LDS  R30,1832
	COM  R30
	ANDI R30,LOW(0x10)
	BRNE PC+3
	JMP _0x109
; 0000 062A     {
; 0000 062B         start_flag = 0;
	CALL SUBOPT_0x60
; 0000 062C         u_1 = 0;
; 0000 062D         u_2 = -0.5;
	__GETD1N 0xBF000000
	CALL SUBOPT_0x61
; 0000 062E         if(SW4_pressed)
	SBIS 0x0,6
	RJMP _0x10C
; 0000 062F         {
; 0000 0630             // SEND DATA IN RAW FROM
; 0000 0631             //Send Control Signal For Trolley, u_1
; 0000 0632             p = (unsigned char *) & u_1;
	__POINTWRM 16,17,_u_1
; 0000 0633             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x10E:
	CPI  R19,4
	BRSH _0x10F
; 0000 0634             {
; 0000 0635             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0636             }
	SUBI R19,-1
	RJMP _0x10E
_0x10F:
; 0000 0637 
; 0000 0638             // Send Control Signal For Arm, u_2
; 0000 0639             p = (unsigned char *) & u_2;
	__POINTWRM 16,17,_u_2
; 0000 063A             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x111:
	CPI  R19,4
	BRSH _0x112
; 0000 063B             {
; 0000 063C             putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 063D             }
	SUBI R19,-1
	RJMP _0x111
_0x112:
; 0000 063E 
; 0000 063F             // SEND REFERENCE DATA IN RAW FROM
; 0000 0640             //Send Reference Signal For Trolley, r_1
; 0000 0641             p = (unsigned char *) & r_1;
	__POINTWRM 16,17,_r_1
; 0000 0642             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x114:
	CPI  R19,4
	BRSH _0x115
; 0000 0643             {
; 0000 0644              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0645             }
	SUBI R19,-1
	RJMP _0x114
_0x115:
; 0000 0646             // Send Reference Signal For Arm, r_2
; 0000 0647             p = (unsigned char *) & r_2;
	__POINTWRM 16,17,_r_2
; 0000 0648             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x117:
	CPI  R19,4
	BRSH _0x118
; 0000 0649             {
; 0000 064A              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 064B             }
	SUBI R19,-1
	RJMP _0x117
_0x118:
; 0000 064C 
; 0000 064D             // SEND Trolley and Arm Position DATA as well to maintain the same data length
; 0000 064E             //Send Trolley Data, x_1 and x_2
; 0000 064F             p = (unsigned char *) & x_1;
	__POINTWRM 16,17,_x_1
; 0000 0650             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x11A:
	CPI  R19,4
	BRSH _0x11B
; 0000 0651             {
; 0000 0652              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0653             }
	SUBI R19,-1
	RJMP _0x11A
_0x11B:
; 0000 0654             // Send Trolley Speed, x_2
; 0000 0655             p = (unsigned char *) & x_2;
	__POINTWRM 16,17,_x_2
; 0000 0656             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x11D:
	CPI  R19,4
	BRSH _0x11E
; 0000 0657             {
; 0000 0658              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0659             }
	SUBI R19,-1
	RJMP _0x11D
_0x11E:
; 0000 065A 
; 0000 065B             //Send Arm Data, x_5 and x_6
; 0000 065C             p = (unsigned char *) & x_5;
	__POINTWRM 16,17,_x_5
; 0000 065D             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x120:
	CPI  R19,4
	BRSH _0x121
; 0000 065E             {
; 0000 065F              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0660             }
	SUBI R19,-1
	RJMP _0x120
_0x121:
; 0000 0661             // Send Arm Speed, x_6
; 0000 0662             p = (unsigned char *) & x_6;
	__POINTWRM 16,17,_x_6
; 0000 0663             for (i=0; i<4; i++)
	LDI  R19,LOW(0)
_0x123:
	CPI  R19,4
	BRSH _0x124
; 0000 0664             {
; 0000 0665              putchar_usartf0( *p++ );
	CALL SUBOPT_0x4A
; 0000 0666             }
	SUBI R19,-1
	RJMP _0x123
_0x124:
; 0000 0667         }
; 0000 0668         // if SW is being continuously pressed then don't send control repeateadly
; 0000 0669         SW4_pressed = 0;
_0x10C:
	CBI  0x0,6
; 0000 066A     }
; 0000 066B     else
	RJMP _0x127
_0x109:
; 0000 066C     {
; 0000 066D     	SW4_pressed = 1;
	SBI  0x0,6
; 0000 066E 
; 0000 066F     }
_0x127:
; 0000 0670 
; 0000 0671     /********* Select Between ETC and TTC ***********/
; 0000 0672 
; 0000 0673     /** Turn on ETC **/
; 0000 0674     if((~PORTK.IN) & PIN5_bm)
	LDS  R30,1832
	COM  R30
	ANDI R30,LOW(0x20)
	BREQ _0x12A
; 0000 0675     {
; 0000 0676         Periodic_ON = 0; // If SW5 is pressed Turn On ETC
	CBI  0x0,7
; 0000 0677         start_flag = 1;
	SBI  0x0,1
; 0000 0678         PORTF.OUTCLR = PIN6_bm;
	LDI  R30,LOW(64)
	STS  1702,R30
; 0000 0679         TCD0.CNT = 0x0000; // Reset the counter
	CALL SUBOPT_0x0
; 0000 067A         TCD0.PER= 0x1388; //Clock/(4*5000) = 100 Hz => h = 10 ms  , for 32Mhz (clock/(64*5000) = 100 Hz
; 0000 067B         h = 0.01; // 10 ms sampling time
	CALL SUBOPT_0x5D
; 0000 067C         /*Calculate Estimation Parameters for ETC*/
; 0000 067D         estimation_param_init();
; 0000 067E 
; 0000 067F 
; 0000 0680     }
; 0000 0681     /** Turn on Periodic TTC **/
; 0000 0682     else if((~PORTK.IN) & PIN6_bm)
	RJMP _0x12F
_0x12A:
	LDS  R30,1832
	COM  R30
	ANDI R30,LOW(0x40)
	BREQ _0x130
; 0000 0683     {
; 0000 0684 
; 0000 0685         PORTF.OUTCLR = PIN6_bm;
	LDI  R30,LOW(64)
	STS  1702,R30
; 0000 0686         u_1 = 0.0;
	LDI  R30,LOW(0)
	STS  _u_1,R30
	STS  _u_1+1,R30
	STS  _u_1+2,R30
	STS  _u_1+3,R30
; 0000 0687         u_2 = 0.0;
	STS  _u_2,R30
	STS  _u_2+1,R30
	STS  _u_2+2,R30
	STS  _u_2+3,R30
; 0000 0688         r_1 = home_x;//0.25;//0.3;
	CALL SUBOPT_0x5B
; 0000 0689         r_2 = home_theta;//PI*0.5;
	CALL SUBOPT_0x5C
; 0000 068A         count_ref = 0;
; 0000 068B         TCD0.CNT = 0x0000; // Reset the counter
	LDI  R30,LOW(0)
	LDI  R31,HIGH(0)
	STS  2336,R30
	STS  2336+1,R31
; 0000 068C         //TCD0.PER= 0x30D4; // 25ms period
; 0000 068D         //TCD0.PER = 0x3A98; // 30 ms period
; 0000 068E         TCD0.PER = 0x4E20; // 40 ms period
	LDI  R30,LOW(20000)
	LDI  R31,HIGH(20000)
	STS  2342,R30
	STS  2342+1,R31
; 0000 068F         //h = 0.025; // 25 ms sampling time
; 0000 0690         //h = 0.030; // 30 ms sampling time
; 0000 0691         h = 0.040; // 30 ms sampling time
	__GETD1N 0x3D23D70A
	STS  _h,R30
	STS  _h+1,R31
	STS  _h+2,R22
	STS  _h+3,R23
; 0000 0692         /*Calculate Estimation Parameters for TTC*/
; 0000 0693         estimation_param_init();
	RCALL _estimation_param_init
; 0000 0694 
; 0000 0695 
; 0000 0696         // Clear Encoder Data Latches Before Starting Move
; 0000 0697         HCTL2032_Reset_Counter(&PORTE);
	CALL SUBOPT_0x5E
; 0000 0698         HCTL2032_Reset_Counter(&PORTC);
; 0000 0699 
; 0000 069A         // Now Start the move
; 0000 069B         Periodic_ON = 1; // If SW6 is pressed, Turn On Periodic Controller
	SBI  0x0,7
; 0000 069C         /* Reset All Controller States and Stored Data before starting first move */
; 0000 069D         reset_controller();
	RCALL _reset_controller
; 0000 069E         start_flag = 1;
	SBI  0x0,1
; 0000 069F 
; 0000 06A0     }
; 0000 06A1 
; 0000 06A2 }
_0x130:
_0x12F:
	RJMP _0x9C
; 0000 06A3 
; 0000 06A4 }
_0x135:
	RJMP _0x135

	.CSEG
_fabs:
    ld   r30,y+
    ld   r31,y+
    ld   r22,y+
    ld   r23,y+
    cbr  r23,0x80
    ret

	.DSEG

	.CSEG

	.CSEG
_HCTL2032_Total_Setup:
	LDD  R26,Y+5
	LDD  R27,Y+5+1
	ADIW R26,2
	LDI  R30,LOW(255)
	ST   X,R30
	LDD  R26,Y+3
	LDD  R27,Y+3+1
	ADIW R26,1
	ST   X,R30
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	ADIW R26,2
	LDI  R30,LOW(254)
	ST   X,R30
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	ADIW R26,1
	LDI  R30,LOW(1)
	CALL SUBOPT_0x62
	LDI  R30,LOW(1)
	ST   X,R30
	LDD  R26,Y+3
	LDD  R27,Y+3+1
	ADIW R26,6
	LDI  R30,LOW(192)
	ST   X,R30
    nop
	LDD  R26,Y+3
	LDD  R27,Y+3+1
	ADIW R26,5
	LDI  R30,LOW(192)
	ST   X,R30
	LDD  R26,Y+3
	LDD  R27,Y+3+1
	ADIW R26,5
	LDI  R30,LOW(16)
	ST   X,R30
	LDD  R26,Y+3
	LDD  R27,Y+3+1
	ADIW R26,6
	LDI  R30,LOW(2)
	ST   X,R30
	LDD  R30,Y+3
	LDD  R31,Y+3+1
	ST   -Y,R31
	ST   -Y,R30
	LDD  R30,Y+2
	ST   -Y,R30
	RCALL _HCTL2032_Set_Count_Mode
	CPI  R30,0
	BRNE _0x2020003
	LDI  R30,LOW(0)
	RJMP _0x2100002
_0x2020003:
	LDI  R30,LOW(1)
_0x2100002:
	ADIW R28,7
	RET
_HCTL2032_Set_Count_Mode:
	LD   R30,Y
	LDI  R31,0
	CPI  R30,LOW(0x1)
	LDI  R26,HIGH(0x1)
	CPC  R31,R26
	BRNE _0x2020007
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	ADIW R26,5
	LDI  R30,LOW(8)
	ST   X,R30
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	ADIW R26,6
	LDI  R30,LOW(4)
	ST   X,R30
	RJMP _0x2020006
_0x2020007:
	CPI  R30,LOW(0x2)
	LDI  R26,HIGH(0x2)
	CPC  R31,R26
	BRNE _0x2020008
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	ADIW R26,6
	LDI  R30,LOW(8)
	CALL SUBOPT_0x62
	LDI  R30,LOW(4)
	ST   X,R30
	RJMP _0x2020006
_0x2020008:
	CPI  R30,LOW(0x3)
	LDI  R26,HIGH(0x3)
	CPC  R31,R26
	BRNE _0x202000A
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	ADIW R26,5
	LDI  R30,LOW(8)
	CALL SUBOPT_0x62
	LDI  R30,LOW(4)
	ST   X,R30
	RJMP _0x2020006
_0x202000A:
	LDI  R30,LOW(0)
	RJMP _0x2100001
_0x2020006:
	LDI  R30,LOW(1)
_0x2100001:
	ADIW R28,3
	RET
_HCTL2032_Read_Count_Data:
	SBIW R28,10
	LDI  R24,10
	LDI  R26,LOW(0)
	LDI  R27,HIGH(0)
	LDI  R30,LOW(_0x202000B*2)
	LDI  R31,HIGH(_0x202000B*2)
	CALL __INITLOCB
	CALL __SAVELOCR6
	LDI  R17,0
	LDI  R16,0
	LDI  R19,0
	LDI  R18,0
	LDI  R21,0
	LDI  R20,0
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
nop
	LDD  R30,Y+16
	CPI  R30,0
	BREQ _0x202000C
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,5
	RJMP _0x2020010
_0x202000C:
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,6
_0x2020010:
	LDI  R30,LOW(2)
	ST   X,R30
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,2
	LDI  R30,LOW(255)
	CALL SUBOPT_0x63
	CALL SUBOPT_0x64
	CALL SUBOPT_0x63
	LDI  R30,LOW(16)
	ST   X,R30
nop
nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R17,X
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R17,X
nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,5
	CALL SUBOPT_0x64
	ST   X,R30
nop
nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R16,X
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R16,X
nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,6
	LDI  R30,LOW(32)
	CALL SUBOPT_0x63
	LDI  R30,LOW(1)
	ST   X,R30
nop
nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R19,X
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R19,X
nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,5
	LDI  R30,LOW(32)
	CALL SUBOPT_0x63
	LDI  R30,LOW(1)
	ST   X,R30
nop
nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
//nop
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R18,X
	LDD  R26,Y+19
	LDD  R27,Y+19+1
	ADIW R26,8
	LD   R18,X
nop
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,5
	LDI  R30,LOW(16)
	ST   X,R30
	MOV  R30,R17
	CALL SUBOPT_0x65
	MOVW R26,R30
	MOVW R24,R22
	LDI  R30,LOW(24)
	CALL __LSLD12
	CALL SUBOPT_0x66
	MOV  R30,R16
	CALL SUBOPT_0x65
	CALL __LSLD16
	CALL SUBOPT_0x67
	MOV  R30,R19
	CALL SUBOPT_0x65
	MOVW R26,R30
	MOVW R24,R22
	LDI  R30,LOW(8)
	CALL __LSLD12
	CALL SUBOPT_0x67
	MOV  R30,R18
	CALL SUBOPT_0x65
	CALL SUBOPT_0x67
	__GETD1S 12
	CALL __LOADLOCR6
	ADIW R28,21
	RET
_HCTL2032_Reset_Counter:
	LD   R26,Y
	LDD  R27,Y+1
	ADIW R26,6
	LDI  R30,LOW(192)
	ST   X,R30
    nop
	LD   R26,Y
	LDD  R27,Y+1
	ADIW R26,5
	LDI  R30,LOW(192)
	ST   X,R30
	LD   R26,Y
	LDD  R27,Y+1
	ADIW R26,5
	LDI  R30,LOW(16)
	ST   X,R30
	LD   R26,Y
	LDD  R27,Y+1
	ADIW R26,6
	LDI  R30,LOW(2)
	ST   X,R30
	ADIW R28,2
	RET

	.CSEG

	.CSEG
_handler_TCE0_CCA_vect:
	CALL SUBOPT_0x68
	ORI  R30,1
	__PUTB1RNS 10,4
	LDS  R30,_i_S1030001000
	LDS  R31,_i_S1030001000+1
	ADIW R30,1
	STS  _i_S1030001000,R30
	STS  _i_S1030001000+1,R31
	MOVW R30,R10
	LDD  R26,Z+4
	LDI  R30,LOW(251)
	AND  R30,R26
	__PUTB1RNS 10,4
	MOV  R30,R13
	LDS  R26,_i_S1030001000
	LDS  R27,_i_S1030001000+1
	LDI  R31,0
	CP   R26,R30
	CPC  R27,R31
	BRLO _0x2060005
	MOVW R26,R10
	ADIW R26,4
	LD   R30,X
	ORI  R30,4
	ST   X,R30
	LDI  R30,LOW(0)
	STS  _i_S1030001000,R30
	STS  _i_S1030001000+1,R30
_0x2060005:
	RJMP _0x2060007
_handler_TCE0_CCB_vect:
	CALL SUBOPT_0x68
	ORI  R30,LOW(0x3)
	__PUTB1RNS 10,4
	RJMP _0x2060007
_handler_TCE0_CCC_vect:
	CALL SUBOPT_0x68
	ORI  R30,2
	__PUTB1RNS 10,4
	RJMP _0x2060007
_handler_TCE0_CCD_vect:
	CALL SUBOPT_0x68
	__PUTB1RNS 10,4
	RJMP _0x2060007
_handler_TCC0_ERR_vect:
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
	LDS  R30,_j_S1030005000
	SUBI R30,-LOW(1)
	STS  _j_S1030005000,R30
	LDS  R26,_j_S1030005000
	CPI  R26,LOW(0x3)
	BRLO _0x2060006
	MOVW R30,R10
	LDD  R26,Z+4
	LDI  R30,LOW(64)
	EOR  R30,R26
	__PUTB1RNS 10,4
	LDI  R30,LOW(0)
	STS  _j_S1030005000,R30
_0x2060006:
_0x2060007:
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	RETI

	.CSEG

	.CSEG

	.CSEG

	.CSEG

	.DSEG
_Pos_Count_HCTL_1:
	.BYTE 0x4
_Pos_Count_HCTL_2:
	.BYTE 0x4
_Pos_Count_HCTL_3:
	.BYTE 0x4
_Pos_Count_HCTL_4:
	.BYTE 0x4
_Temp_Pos_Count_HCTL_1:
	.BYTE 0x4
_Temp_Pos_Count_HCTL_2:
	.BYTE 0x4
_Temp_Pos_Count_HCTL_3:
	.BYTE 0x4
_Temp_Pos_Count_HCTL_4:
	.BYTE 0x4
_h:
	.BYTE 0x4
_ad:
	.BYTE 0x4
_bd:
	.BYTE 0x4
_ad1:
	.BYTE 0x4
_bd1:
	.BYTE 0x4
_bi1:
	.BYTE 0x4
_bi2:
	.BYTE 0x4
_a01:
	.BYTE 0x4
_a02:
	.BYTE 0x4
_x_1:
	.BYTE 0x4
_x_2:
	.BYTE 0x4
_x_3:
	.BYTE 0x4
_x_4:
	.BYTE 0x4
_x_a1:
	.BYTE 0x4
_x_5:
	.BYTE 0x4
_x_6:
	.BYTE 0x4
_x_7:
	.BYTE 0x4
_x_8:
	.BYTE 0x4
_x_a2:
	.BYTE 0x4
_x_1_old:
	.BYTE 0x4
_x_2_old:
	.BYTE 0x4
_x_3_old:
	.BYTE 0x4
_x_4_old:
	.BYTE 0x4
_x_5_old:
	.BYTE 0x4
_x_6_old:
	.BYTE 0x4
_x_7_old:
	.BYTE 0x4
_x_8_old:
	.BYTE 0x4
_x_a1_old:
	.BYTE 0x4
_x_a2_old:
	.BYTE 0x4
_Phi:
	.BYTE 0x4
_x_1_tk:
	.BYTE 0x4
_x_2_tk:
	.BYTE 0x4
_x_3_tk:
	.BYTE 0x4
_x_4_tk:
	.BYTE 0x4
_x_a1_tk:
	.BYTE 0x4
_x_5_tk:
	.BYTE 0x4
_x_6_tk:
	.BYTE 0x4
_x_7_tk:
	.BYTE 0x4
_x_8_tk:
	.BYTE 0x4
_x_a2_tk:
	.BYTE 0x4
_norm_e:
	.BYTE 0x4
_norm_epsilon:
	.BYTE 0x4
_w_3:
	.BYTE 0x4
_w_4:
	.BYTE 0x4
_w_8:
	.BYTE 0x4
_w_9:
	.BYTE 0x4
_r_1:
	.BYTE 0x4
_r_2:
	.BYTE 0x4
_home_x:
	.BYTE 0x4
_home_theta:
	.BYTE 0x4
_u_1:
	.BYTE 0x4
_u_2:
	.BYTE 0x4
_u_1_old:
	.BYTE 0x4
_u_2_old:
	.BYTE 0x4
_u_11:
	.BYTE 0x4
_v_1:
	.BYTE 0x4
_v_2:
	.BYTE 0x4
_iterations:
	.BYTE 0x4
_iterations_old:
	.BYTE 0x4
_count_ref:
	.BYTE 0x4
__seed_G100:
	.BYTE 0x4
_i_S1030001000:
	.BYTE 0x2
_j_S1030005000:
	.BYTE 0x1

	.CSEG
;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:17 WORDS
SUBOPT_0x0:
	LDI  R30,LOW(0)
	LDI  R31,HIGH(0)
	STS  2336,R30
	STS  2336+1,R31
	LDI  R30,LOW(5000)
	LDI  R31,HIGH(5000)
	STS  2342,R30
	STS  2342+1,R31
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:10 WORDS
SUBOPT_0x1:
	ST   -Y,R0
	ST   -Y,R1
	ST   -Y,R15
	ST   -Y,R22
	ST   -Y,R23
	ST   -Y,R24
	ST   -Y,R25
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
	CALL __SAVELOCR4
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x2:
	LDS  R30,_iterations
	LDS  R31,_iterations+1
	LDS  R22,_iterations+2
	LDS  R23,_iterations+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x3:
	LDI  R30,LOW(1632)
	LDI  R31,HIGH(1632)
	ST   -Y,R31
	ST   -Y,R30
	LDI  R30,LOW(1664)
	LDI  R31,HIGH(1664)
	ST   -Y,R31
	ST   -Y,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x4:
	LDI  R30,LOW(1760)
	LDI  R31,HIGH(1760)
	ST   -Y,R31
	ST   -Y,R30
	LDI  R30,LOW(1600)
	LDI  R31,HIGH(1600)
	ST   -Y,R31
	ST   -Y,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:33 WORDS
SUBOPT_0x5:
	CALL __CDF1
	__GETD2N 0x40000000
	CALL __MULF12
	__GETD2N 0x40490FDB
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x6:
	MOVW R26,R30
	MOVW R24,R22
	__GETD1N 0x45800000
	CALL __DIVF21
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x7:
	STS  _x_1,R30
	STS  _x_1+1,R31
	STS  _x_1+2,R22
	STS  _x_1+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x8:
	STS  _x_5,R30
	STS  _x_5+1,R31
	STS  _x_5+2,R22
	STS  _x_5+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x9:
	STS  _x_3,R30
	STS  _x_3+1,R31
	STS  _x_3+2,R22
	STS  _x_3+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0xA:
	STS  _x_7,R30
	STS  _x_7+1,R31
	STS  _x_7+2,R22
	STS  _x_7+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 8 TIMES, CODE SIZE REDUCTION:39 WORDS
SUBOPT_0xB:
	LDS  R30,_x_1
	LDS  R31,_x_1+1
	LDS  R22,_x_1+2
	LDS  R23,_x_1+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0xC:
	CALL __PUTPARD1
	CALL _fabs
	MOVW R26,R30
	MOVW R24,R22
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 5 TIMES, CODE SIZE REDUCTION:21 WORDS
SUBOPT_0xD:
	LDS  R30,_x_1_old
	LDS  R31,_x_1_old+1
	LDS  R22,_x_1_old+2
	LDS  R23,_x_1_old+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 10 TIMES, CODE SIZE REDUCTION:51 WORDS
SUBOPT_0xE:
	LDS  R30,_x_3
	LDS  R31,_x_3+1
	LDS  R22,_x_3+2
	LDS  R23,_x_3+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0xF:
	__GETD1N 0x3F666666
	CALL __CMPF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x10:
	LDS  R30,_x_3_old
	LDS  R31,_x_3_old+1
	LDS  R22,_x_3_old+2
	LDS  R23,_x_3_old+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 9 TIMES, CODE SIZE REDUCTION:45 WORDS
SUBOPT_0x11:
	LDS  R30,_x_5
	LDS  R31,_x_5+1
	LDS  R22,_x_5+2
	LDS  R23,_x_5+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 9 TIMES, CODE SIZE REDUCTION:45 WORDS
SUBOPT_0x12:
	LDS  R30,_x_7
	LDS  R31,_x_7+1
	LDS  R22,_x_7+2
	LDS  R23,_x_7+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x13:
	LDS  R30,_x_7_old
	LDS  R31,_x_7_old+1
	LDS  R22,_x_7_old+2
	LDS  R23,_x_7_old+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 7 TIMES, CODE SIZE REDUCTION:21 WORDS
SUBOPT_0x14:
	CALL __MULF12
	__GETD2N 0x3F800000
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x15:
	CALL __ADDF12
	__GETD2N 0x3F800000
	CALL __DIVF21
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 10 TIMES, CODE SIZE REDUCTION:51 WORDS
SUBOPT_0x16:
	LDS  R30,_h
	LDS  R31,_h+1
	LDS  R22,_h+2
	LDS  R23,_h+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x17:
	CALL __SWAPD12
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x18:
	LDS  R30,_u_1_old
	LDS  R31,_u_1_old+1
	LDS  R22,_u_1_old+2
	LDS  R23,_u_1_old+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x19:
	CALL __ADDF12
	LDS  R26,_h
	LDS  R27,_h+1
	LDS  R24,_h+2
	LDS  R25,_h+3
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x1A:
	LDS  R26,_Phi
	LDS  R27,_Phi+1
	LDS  R24,_Phi+2
	LDS  R25,_Phi+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:5 WORDS
SUBOPT_0x1B:
	LDS  R26,_h
	LDS  R27,_h+1
	LDS  R24,_h+2
	LDS  R25,_h+3
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x1C:
	CALL __MULF12
	LDS  R26,_x_7_old
	LDS  R27,_x_7_old+1
	LDS  R24,_x_7_old+2
	LDS  R25,_x_7_old+3
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x1D:
	LDS  R30,_u_2_old
	LDS  R31,_u_2_old+1
	LDS  R22,_u_2_old+2
	LDS  R23,_u_2_old+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:15 WORDS
SUBOPT_0x1E:
	LDS  R26,_x_2
	LDS  R27,_x_2+1
	LDS  R24,_x_2+2
	LDS  R25,_x_2+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x1F:
	LDS  R26,_x_6
	LDS  R27,_x_6+1
	LDS  R24,_x_6+2
	LDS  R25,_x_6+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 14 TIMES, CODE SIZE REDUCTION:23 WORDS
SUBOPT_0x20:
	__GETD1N 0x3F800000
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x21:
	STS  _w_3,R30
	STS  _w_3+1,R31
	STS  _w_3+2,R22
	STS  _w_3+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x22:
	CALL __PUTPARD1
	JMP  _fabs

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:5 WORDS
SUBOPT_0x23:
	RCALL SUBOPT_0x20
	STS  _w_8,R30
	STS  _w_8+1,R31
	STS  _w_8+2,R22
	STS  _w_8+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x24:
	CALL __DIVF21
	CALL __CFD1U
	LDS  R26,_count_ref
	LDS  R27,_count_ref+1
	LDS  R24,_count_ref+2
	LDS  R25,_count_ref+3
	CALL __CPD12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:15 WORDS
SUBOPT_0x25:
	STS  _r_1,R30
	STS  _r_1+1,R31
	STS  _r_1+2,R22
	STS  _r_1+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:4 WORDS
SUBOPT_0x26:
	STS  _r_2,R30
	STS  _r_2+1,R31
	STS  _r_2+2,R22
	STS  _r_2+3,R23
	SBI  0x1,1
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:13 WORDS
SUBOPT_0x27:
	RCALL SUBOPT_0x25
	LDS  R30,_home_theta
	LDS  R31,_home_theta+1
	LDS  R22,_home_theta+2
	LDS  R23,_home_theta+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x28:
	RCALL SUBOPT_0xB
	CALL __SUBF12
	MOVW R26,R30
	MOVW R24,R22
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 8 TIMES, CODE SIZE REDUCTION:39 WORDS
SUBOPT_0x29:
	LDS  R30,_x_2
	LDS  R31,_x_2+1
	LDS  R22,_x_2+2
	LDS  R23,_x_2+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 8 TIMES, CODE SIZE REDUCTION:39 WORDS
SUBOPT_0x2A:
	LDS  R30,_x_4
	LDS  R31,_x_4+1
	LDS  R22,_x_4+2
	LDS  R23,_x_4+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:19 WORDS
SUBOPT_0x2B:
	LDS  R30,_r_1
	LDS  R31,_r_1+1
	LDS  R22,_r_1+2
	LDS  R23,_r_1+3
	__GETD2N 0xC0B672B0
	CALL __MULF12
	LDS  R26,_x_a1
	LDS  R27,_x_a1+1
	LDS  R24,_x_a1+2
	LDS  R25,_x_a1+3
	CALL __ADDF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x2C:
	LDS  R26,_r_2
	LDS  R27,_r_2+1
	LDS  R24,_r_2+2
	LDS  R25,_r_2+3
	RCALL SUBOPT_0x11
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 7 TIMES, CODE SIZE REDUCTION:33 WORDS
SUBOPT_0x2D:
	LDS  R30,_x_6
	LDS  R31,_x_6+1
	LDS  R22,_x_6+2
	LDS  R23,_x_6+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 7 TIMES, CODE SIZE REDUCTION:33 WORDS
SUBOPT_0x2E:
	LDS  R30,_x_8
	LDS  R31,_x_8+1
	LDS  R22,_x_8+2
	LDS  R23,_x_8+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:19 WORDS
SUBOPT_0x2F:
	LDS  R30,_r_2
	LDS  R31,_r_2+1
	LDS  R22,_r_2+2
	LDS  R23,_r_2+3
	__GETD2N 0xC03028F6
	CALL __MULF12
	LDS  R26,_x_a2
	LDS  R27,_x_a2+1
	LDS  R24,_x_a2+2
	LDS  R25,_x_a2+3
	CALL __ADDF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x30:
	CALL __ADDF12
	CALL __PUTPARD1
	JMP  _sqrt

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x31:
	LDS  R26,_x_2_tk
	LDS  R27,_x_2_tk+1
	LDS  R24,_x_2_tk+2
	LDS  R25,_x_2_tk+3
	RCALL SUBOPT_0x29
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x32:
	LDS  R26,_x_3_tk
	LDS  R27,_x_3_tk+1
	LDS  R24,_x_3_tk+2
	LDS  R25,_x_3_tk+3
	RCALL SUBOPT_0xE
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x33:
	LDS  R26,_x_4_tk
	LDS  R27,_x_4_tk+1
	LDS  R24,_x_4_tk+2
	LDS  R25,_x_4_tk+3
	RCALL SUBOPT_0x2A
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:13 WORDS
SUBOPT_0x34:
	LDS  R26,_x_a1_tk
	LDS  R27,_x_a1_tk+1
	LDS  R24,_x_a1_tk+2
	LDS  R25,_x_a1_tk+3
	LDS  R30,_x_a1
	LDS  R31,_x_a1+1
	LDS  R22,_x_a1+2
	LDS  R23,_x_a1+3
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x35:
	LDS  R26,_x_5_tk
	LDS  R27,_x_5_tk+1
	LDS  R24,_x_5_tk+2
	LDS  R25,_x_5_tk+3
	RCALL SUBOPT_0x11
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x36:
	LDS  R26,_x_6_tk
	LDS  R27,_x_6_tk+1
	LDS  R24,_x_6_tk+2
	LDS  R25,_x_6_tk+3
	RCALL SUBOPT_0x2D
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x37:
	LDS  R26,_x_7_tk
	LDS  R27,_x_7_tk+1
	LDS  R24,_x_7_tk+2
	LDS  R25,_x_7_tk+3
	RCALL SUBOPT_0x12
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x38:
	LDS  R26,_x_8_tk
	LDS  R27,_x_8_tk+1
	LDS  R24,_x_8_tk+2
	LDS  R25,_x_8_tk+3
	RCALL SUBOPT_0x2E
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:13 WORDS
SUBOPT_0x39:
	LDS  R26,_x_a2_tk
	LDS  R27,_x_a2_tk+1
	LDS  R24,_x_a2_tk+2
	LDS  R25,_x_a2_tk+3
	LDS  R30,_x_a2
	LDS  R31,_x_a2+1
	LDS  R22,_x_a2+2
	LDS  R23,_x_a2+3
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x3A:
	RCALL SUBOPT_0xB
	__GETD2N 0xC0B672B0
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x3B:
	RCALL SUBOPT_0x29
	__GETD2N 0x3E4CCCCD
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x3C:
	RCALL SUBOPT_0xE
	__GETD2N 0xC0200000
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x3D:
	RCALL SUBOPT_0x2A
	__GETD2N 0xBE19652C
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:15 WORDS
SUBOPT_0x3E:
	CALL __ADDF12
	LDS  R26,_x_a1
	LDS  R27,_x_a1+1
	LDS  R24,_x_a1+2
	LDS  R25,_x_a1+3
	CALL __ADDF12
	STS  _v_1,R30
	STS  _v_1+1,R31
	STS  _v_1+2,R22
	STS  _v_1+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x3F:
	RCALL SUBOPT_0x11
	__GETD2N 0xC03028F6
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x40:
	RCALL SUBOPT_0x2D
	__GETD2N 0x3DE90FF9
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x41:
	RCALL SUBOPT_0x12
	__GETD2N 0x40200000
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x42:
	RCALL SUBOPT_0x2E
	__GETD2N 0x3E2FB7E9
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:27 WORDS
SUBOPT_0x43:
	CALL __ADDF12
	LDS  R26,_x_a2
	LDS  R27,_x_a2+1
	LDS  R24,_x_a2+2
	LDS  R25,_x_a2+3
	CALL __ADDF12
	STS  _v_2,R30
	STS  _v_2+1,R31
	STS  _v_2+2,R22
	STS  _v_2+3,R23
	LDS  R26,_v_1
	LDS  R27,_v_1+1
	LDS  R24,_v_1+2
	LDS  R25,_v_1+3
	RCALL SUBOPT_0x20
	CALL __CMPF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x44:
	LDS  R26,_v_1
	LDS  R27,_v_1+1
	LDS  R24,_v_1+2
	LDS  R25,_v_1+3
	__GETD1N 0xBF800000
	CALL __CMPF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 6 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x45:
	__GETD1N 0xBF800000
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x46:
	LDS  R30,_v_1
	LDS  R31,_v_1+1
	LDS  R22,_v_1+2
	LDS  R23,_v_1+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 5 TIMES, CODE SIZE REDUCTION:21 WORDS
SUBOPT_0x47:
	LDS  R26,_v_2
	LDS  R27,_v_2+1
	LDS  R24,_v_2+2
	LDS  R25,_v_2+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x48:
	RCALL SUBOPT_0x47
	RCALL SUBOPT_0x45
	CALL __CMPF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x49:
	LDS  R30,_v_2
	LDS  R31,_v_2+1
	LDS  R22,_v_2+2
	LDS  R23,_v_2+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 56 TIMES, CODE SIZE REDUCTION:272 WORDS
SUBOPT_0x4A:
	MOVW R26,R16
	__ADDWRN 16,17,1
	LD   R30,X
	ST   -Y,R30
	JMP  _putchar_usartf0

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x4B:
	LDS  R30,_x_a1
	LDS  R31,_x_a1+1
	LDS  R22,_x_a1+2
	LDS  R23,_x_a1+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x4C:
	LDS  R30,_x_a2
	LDS  R31,_x_a2+1
	LDS  R22,_x_a2+2
	LDS  R23,_x_a2+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:13 WORDS
SUBOPT_0x4D:
	LDS  R26,_x_1
	LDS  R27,_x_1+1
	LDS  R24,_x_1+2
	LDS  R25,_x_1+3
	LDS  R30,_r_1
	LDS  R31,_r_1+1
	LDS  R22,_r_1+2
	LDS  R23,_r_1+3
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x4E:
	__GETD2N 0x3BA3D70A
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x4F:
	LDS  R30,_u_1
	LDS  R31,_u_1+1
	LDS  R22,_u_1+2
	LDS  R23,_u_1+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x50:
	CALL __SUBF12
	__GETD2N 0x3E4CCCCD
	CALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:13 WORDS
SUBOPT_0x51:
	LDS  R26,_x_5
	LDS  R27,_x_5+1
	LDS  R24,_x_5+2
	LDS  R25,_x_5+3
	LDS  R30,_r_2
	LDS  R31,_r_2+1
	LDS  R22,_r_2+2
	LDS  R23,_r_2+3
	CALL __SUBF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x52:
	LDS  R30,_u_2
	LDS  R31,_u_2+1
	LDS  R22,_u_2+2
	LDS  R23,_u_2+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:31 WORDS
SUBOPT_0x53:
	LDI  R30,LOW(0)
	STS  _u_1,R30
	STS  _u_1+1,R30
	STS  _u_1+2,R30
	STS  _u_1+3,R30
	STS  _u_2,R30
	STS  _u_2+1,R30
	STS  _u_2+2,R30
	STS  _u_2+3,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:27 WORDS
SUBOPT_0x54:
	STS  _v_1,R30
	STS  _v_1+1,R30
	STS  _v_1+2,R30
	STS  _v_1+3,R30
	LDI  R30,LOW(0)
	STS  _v_2,R30
	STS  _v_2+1,R30
	STS  _v_2+2,R30
	STS  _v_2+3,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:101 WORDS
SUBOPT_0x55:
	LDI  R30,LOW(0)
	STS  _x_a1,R30
	STS  _x_a1+1,R30
	STS  _x_a1+2,R30
	STS  _x_a1+3,R30
	STS  _x_a1_old,R30
	STS  _x_a1_old+1,R30
	STS  _x_a1_old+2,R30
	STS  _x_a1_old+3,R30
	STS  _x_a1_tk,R30
	STS  _x_a1_tk+1,R30
	STS  _x_a1_tk+2,R30
	STS  _x_a1_tk+3,R30
	STS  _x_a2,R30
	STS  _x_a2+1,R30
	STS  _x_a2+2,R30
	STS  _x_a2+3,R30
	STS  _x_a2_old,R30
	STS  _x_a2_old+1,R30
	STS  _x_a2_old+2,R30
	STS  _x_a2_old+3,R30
	STS  _x_a2_tk,R30
	STS  _x_a2_tk+1,R30
	STS  _x_a2_tk+2,R30
	STS  _x_a2_tk+3,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:27 WORDS
SUBOPT_0x56:
	STS  _u_1_old,R30
	STS  _u_1_old+1,R30
	STS  _u_1_old+2,R30
	STS  _u_1_old+3,R30
	LDI  R30,LOW(0)
	STS  _u_2_old,R30
	STS  _u_2_old+1,R30
	STS  _u_2_old+2,R30
	STS  _u_2_old+3,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x57:
	LDS  R26,_h
	LDS  R27,_h+1
	LDS  R24,_h+2
	LDS  R25,_h+3
	__GETD1N 0x3D4CCCCD
	CALL __DIVF21
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x58:
	RCALL SUBOPT_0x16
	__GETD2N 0x41200000
	RJMP SUBOPT_0x14

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x59:
	RCALL SUBOPT_0x16
	__GETD2N 0x40A00000
	RJMP SUBOPT_0x14

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:69 WORDS
SUBOPT_0x5A:
	LDI  R30,LOW(0)
	STS  _x_1_old,R30
	STS  _x_1_old+1,R30
	STS  _x_1_old+2,R30
	STS  _x_1_old+3,R30
	STS  _x_2_old,R30
	STS  _x_2_old+1,R30
	STS  _x_2_old+2,R30
	STS  _x_2_old+3,R30
	STS  _x_3_old,R30
	STS  _x_3_old+1,R30
	STS  _x_3_old+2,R30
	STS  _x_3_old+3,R30
	STS  _x_4_old,R30
	STS  _x_4_old+1,R30
	STS  _x_4_old+2,R30
	STS  _x_4_old+3,R30
	STS  _x_5_old,R30
	STS  _x_5_old+1,R30
	STS  _x_5_old+2,R30
	STS  _x_5_old+3,R30
	STS  _x_6_old,R30
	STS  _x_6_old+1,R30
	STS  _x_6_old+2,R30
	STS  _x_6_old+3,R30
	STS  _x_7_old,R30
	STS  _x_7_old+1,R30
	STS  _x_7_old+2,R30
	STS  _x_7_old+3,R30
	STS  _x_8_old,R30
	STS  _x_8_old+1,R30
	STS  _x_8_old+2,R30
	STS  _x_8_old+3,R30
	RJMP SUBOPT_0x53

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:5 WORDS
SUBOPT_0x5B:
	LDS  R30,_home_x
	LDS  R31,_home_x+1
	LDS  R22,_home_x+2
	LDS  R23,_home_x+3
	RJMP SUBOPT_0x27

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:12 WORDS
SUBOPT_0x5C:
	STS  _r_2,R30
	STS  _r_2+1,R31
	STS  _r_2+2,R22
	STS  _r_2+3,R23
	LDI  R30,LOW(0)
	STS  _count_ref,R30
	STS  _count_ref+1,R30
	STS  _count_ref+2,R30
	STS  _count_ref+3,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x5D:
	__GETD1N 0x3C23D70A
	STS  _h,R30
	STS  _h+1,R31
	STS  _h+2,R22
	STS  _h+3,R23
	JMP  _estimation_param_init

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x5E:
	LDI  R30,LOW(1664)
	LDI  R31,HIGH(1664)
	ST   -Y,R31
	ST   -Y,R30
	CALL _HCTL2032_Reset_Counter
	LDI  R30,LOW(1600)
	LDI  R31,HIGH(1600)
	ST   -Y,R31
	ST   -Y,R30
	JMP  _HCTL2032_Reset_Counter

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:12 WORDS
SUBOPT_0x5F:
	STS  _u_1,R30
	STS  _u_1+1,R31
	STS  _u_1+2,R22
	STS  _u_1+3,R23
	LDI  R30,LOW(0)
	STS  _u_2,R30
	STS  _u_2+1,R30
	STS  _u_2+2,R30
	STS  _u_2+3,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:5 WORDS
SUBOPT_0x60:
	CBI  0x0,1
	LDI  R30,LOW(0)
	STS  _u_1,R30
	STS  _u_1+1,R30
	STS  _u_1+2,R30
	STS  _u_1+3,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x61:
	STS  _u_2,R30
	STS  _u_2+1,R31
	STS  _u_2+2,R22
	STS  _u_2+3,R23
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x62:
	ST   X,R30
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	ADIW R26,5
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x63:
	ST   X,R30
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,6
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x64:
	LDI  R30,LOW(32)
	ST   X,R30
	LDD  R26,Y+17
	LDD  R27,Y+17+1
	ADIW R26,5
	LDI  R30,LOW(1)
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:6 WORDS
SUBOPT_0x65:
	LDI  R31,0
	CALL __CWD1
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x66:
	__PUTD1S 12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:9 WORDS
SUBOPT_0x67:
	__GETD2S 12
	CALL __ORD12
	RJMP SUBOPT_0x66

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:21 WORDS
SUBOPT_0x68:
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
	MOVW R30,R10
	LDD  R26,Z+4
	LDI  R30,LOW(252)
	AND  R30,R26
	RET


	.CSEG
__ROUND_REPACK:
	TST  R21
	BRPL __REPACK
	CPI  R21,0x80
	BRNE __ROUND_REPACK0
	SBRS R30,0
	RJMP __REPACK
__ROUND_REPACK0:
	ADIW R30,1
	ADC  R22,R25
	ADC  R23,R25
	BRVS __REPACK1

__REPACK:
	LDI  R21,0x80
	EOR  R21,R23
	BRNE __REPACK0
	PUSH R21
	RJMP __ZERORES
__REPACK0:
	CPI  R21,0xFF
	BREQ __REPACK1
	LSL  R22
	LSL  R0
	ROR  R21
	ROR  R22
	MOV  R23,R21
	RET
__REPACK1:
	PUSH R21
	TST  R0
	BRMI __REPACK2
	RJMP __MAXRES
__REPACK2:
	RJMP __MINRES

__UNPACK:
	LDI  R21,0x80
	MOV  R1,R25
	AND  R1,R21
	LSL  R24
	ROL  R25
	EOR  R25,R21
	LSL  R21
	ROR  R24

__UNPACK1:
	LDI  R21,0x80
	MOV  R0,R23
	AND  R0,R21
	LSL  R22
	ROL  R23
	EOR  R23,R21
	LSL  R21
	ROR  R22
	RET

__CFD1U:
	SET
	RJMP __CFD1U0
__CFD1:
	CLT
__CFD1U0:
	PUSH R21
	RCALL __UNPACK1
	CPI  R23,0x80
	BRLO __CFD10
	CPI  R23,0xFF
	BRCC __CFD10
	RJMP __ZERORES
__CFD10:
	LDI  R21,22
	SUB  R21,R23
	BRPL __CFD11
	NEG  R21
	CPI  R21,8
	BRTC __CFD19
	CPI  R21,9
__CFD19:
	BRLO __CFD17
	SER  R30
	SER  R31
	SER  R22
	LDI  R23,0x7F
	BLD  R23,7
	RJMP __CFD15
__CFD17:
	CLR  R23
	TST  R21
	BREQ __CFD15
__CFD18:
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R23
	DEC  R21
	BRNE __CFD18
	RJMP __CFD15
__CFD11:
	CLR  R23
__CFD12:
	CPI  R21,8
	BRLO __CFD13
	MOV  R30,R31
	MOV  R31,R22
	MOV  R22,R23
	SUBI R21,8
	RJMP __CFD12
__CFD13:
	TST  R21
	BREQ __CFD15
__CFD14:
	LSR  R23
	ROR  R22
	ROR  R31
	ROR  R30
	DEC  R21
	BRNE __CFD14
__CFD15:
	TST  R0
	BRPL __CFD16
	RCALL __ANEGD1
__CFD16:
	POP  R21
	RET

__CDF1U:
	SET
	RJMP __CDF1U0
__CDF1:
	CLT
__CDF1U0:
	SBIW R30,0
	SBCI R22,0
	SBCI R23,0
	BREQ __CDF10
	CLR  R0
	BRTS __CDF11
	TST  R23
	BRPL __CDF11
	COM  R0
	RCALL __ANEGD1
__CDF11:
	MOV  R1,R23
	LDI  R23,30
	TST  R1
__CDF12:
	BRMI __CDF13
	DEC  R23
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R1
	RJMP __CDF12
__CDF13:
	MOV  R30,R31
	MOV  R31,R22
	MOV  R22,R1
	PUSH R21
	RCALL __REPACK
	POP  R21
__CDF10:
	RET

__SWAPACC:
	PUSH R20
	MOVW R20,R30
	MOVW R30,R26
	MOVW R26,R20
	MOVW R20,R22
	MOVW R22,R24
	MOVW R24,R20
	MOV  R20,R0
	MOV  R0,R1
	MOV  R1,R20
	POP  R20
	RET

__UADD12:
	ADD  R30,R26
	ADC  R31,R27
	ADC  R22,R24
	RET

__NEGMAN1:
	COM  R30
	COM  R31
	COM  R22
	SUBI R30,-1
	SBCI R31,-1
	SBCI R22,-1
	RET

__SUBF12:
	PUSH R21
	RCALL __UNPACK
	CPI  R25,0x80
	BREQ __ADDF129
	LDI  R21,0x80
	EOR  R1,R21

	RJMP __ADDF120

__ADDF12:
	PUSH R21
	RCALL __UNPACK
	CPI  R25,0x80
	BREQ __ADDF129

__ADDF120:
	CPI  R23,0x80
	BREQ __ADDF128
__ADDF121:
	MOV  R21,R23
	SUB  R21,R25
	BRVS __ADDF1211
	BRPL __ADDF122
	RCALL __SWAPACC
	RJMP __ADDF121
__ADDF122:
	CPI  R21,24
	BRLO __ADDF123
	CLR  R26
	CLR  R27
	CLR  R24
__ADDF123:
	CPI  R21,8
	BRLO __ADDF124
	MOV  R26,R27
	MOV  R27,R24
	CLR  R24
	SUBI R21,8
	RJMP __ADDF123
__ADDF124:
	TST  R21
	BREQ __ADDF126
__ADDF125:
	LSR  R24
	ROR  R27
	ROR  R26
	DEC  R21
	BRNE __ADDF125
__ADDF126:
	MOV  R21,R0
	EOR  R21,R1
	BRMI __ADDF127
	RCALL __UADD12
	BRCC __ADDF129
	ROR  R22
	ROR  R31
	ROR  R30
	INC  R23
	BRVC __ADDF129
	RJMP __MAXRES
__ADDF128:
	RCALL __SWAPACC
__ADDF129:
	RCALL __REPACK
	POP  R21
	RET
__ADDF1211:
	BRCC __ADDF128
	RJMP __ADDF129
__ADDF127:
	SUB  R30,R26
	SBC  R31,R27
	SBC  R22,R24
	BREQ __ZERORES
	BRCC __ADDF1210
	COM  R0
	RCALL __NEGMAN1
__ADDF1210:
	TST  R22
	BRMI __ADDF129
	LSL  R30
	ROL  R31
	ROL  R22
	DEC  R23
	BRVC __ADDF1210

__ZERORES:
	CLR  R30
	CLR  R31
	CLR  R22
	CLR  R23
	POP  R21
	RET

__MINRES:
	SER  R30
	SER  R31
	LDI  R22,0x7F
	SER  R23
	POP  R21
	RET

__MAXRES:
	SER  R30
	SER  R31
	LDI  R22,0x7F
	LDI  R23,0x7F
	POP  R21
	RET

__MULF12:
	PUSH R21
	RCALL __UNPACK
	CPI  R23,0x80
	BREQ __ZERORES
	CPI  R25,0x80
	BREQ __ZERORES
	EOR  R0,R1
	SEC
	ADC  R23,R25
	BRVC __MULF124
	BRLT __ZERORES
__MULF125:
	TST  R0
	BRMI __MINRES
	RJMP __MAXRES
__MULF124:
	PUSH R0
	PUSH R17
	PUSH R18
	PUSH R19
	PUSH R20
	CLR  R17
	CLR  R18
	CLR  R25
	MUL  R22,R24
	MOVW R20,R0
	MUL  R24,R31
	MOV  R19,R0
	ADD  R20,R1
	ADC  R21,R25
	MUL  R22,R27
	ADD  R19,R0
	ADC  R20,R1
	ADC  R21,R25
	MUL  R24,R30
	RCALL __MULF126
	MUL  R27,R31
	RCALL __MULF126
	MUL  R22,R26
	RCALL __MULF126
	MUL  R27,R30
	RCALL __MULF127
	MUL  R26,R31
	RCALL __MULF127
	MUL  R26,R30
	ADD  R17,R1
	ADC  R18,R25
	ADC  R19,R25
	ADC  R20,R25
	ADC  R21,R25
	MOV  R30,R19
	MOV  R31,R20
	MOV  R22,R21
	MOV  R21,R18
	POP  R20
	POP  R19
	POP  R18
	POP  R17
	POP  R0
	TST  R22
	BRMI __MULF122
	LSL  R21
	ROL  R30
	ROL  R31
	ROL  R22
	RJMP __MULF123
__MULF122:
	INC  R23
	BRVS __MULF125
__MULF123:
	RCALL __ROUND_REPACK
	POP  R21
	RET

__MULF127:
	ADD  R17,R0
	ADC  R18,R1
	ADC  R19,R25
	RJMP __MULF128
__MULF126:
	ADD  R18,R0
	ADC  R19,R1
__MULF128:
	ADC  R20,R25
	ADC  R21,R25
	RET

__DIVF21:
	PUSH R21
	RCALL __UNPACK
	CPI  R23,0x80
	BRNE __DIVF210
	TST  R1
__DIVF211:
	BRPL __DIVF219
	RJMP __MINRES
__DIVF219:
	RJMP __MAXRES
__DIVF210:
	CPI  R25,0x80
	BRNE __DIVF218
__DIVF217:
	RJMP __ZERORES
__DIVF218:
	EOR  R0,R1
	SEC
	SBC  R25,R23
	BRVC __DIVF216
	BRLT __DIVF217
	TST  R0
	RJMP __DIVF211
__DIVF216:
	MOV  R23,R25
	PUSH R17
	PUSH R18
	PUSH R19
	PUSH R20
	CLR  R1
	CLR  R17
	CLR  R18
	CLR  R19
	CLR  R20
	CLR  R21
	LDI  R25,32
__DIVF212:
	CP   R26,R30
	CPC  R27,R31
	CPC  R24,R22
	CPC  R20,R17
	BRLO __DIVF213
	SUB  R26,R30
	SBC  R27,R31
	SBC  R24,R22
	SBC  R20,R17
	SEC
	RJMP __DIVF214
__DIVF213:
	CLC
__DIVF214:
	ROL  R21
	ROL  R18
	ROL  R19
	ROL  R1
	ROL  R26
	ROL  R27
	ROL  R24
	ROL  R20
	DEC  R25
	BRNE __DIVF212
	MOVW R30,R18
	MOV  R22,R1
	POP  R20
	POP  R19
	POP  R18
	POP  R17
	TST  R22
	BRMI __DIVF215
	LSL  R21
	ROL  R30
	ROL  R31
	ROL  R22
	DEC  R23
	BRVS __DIVF217
__DIVF215:
	RCALL __ROUND_REPACK
	POP  R21
	RET

__CMPF12:
	TST  R25
	BRMI __CMPF120
	TST  R23
	BRMI __CMPF121
	CP   R25,R23
	BRLO __CMPF122
	BRNE __CMPF121
	CP   R26,R30
	CPC  R27,R31
	CPC  R24,R22
	BRLO __CMPF122
	BREQ __CMPF123
__CMPF121:
	CLZ
	CLC
	RET
__CMPF122:
	CLZ
	SEC
	RET
__CMPF123:
	SEZ
	CLC
	RET
__CMPF120:
	TST  R23
	BRPL __CMPF122
	CP   R25,R23
	BRLO __CMPF121
	BRNE __CMPF122
	CP   R30,R26
	CPC  R31,R27
	CPC  R22,R24
	BRLO __CMPF122
	BREQ __CMPF123
	RJMP __CMPF121

_sqrt:
	sbiw r28,4
	push r21
	ldd  r25,y+7
	tst  r25
	brne __sqrt0
	adiw r28,8
	rjmp __zerores
__sqrt0:
	brpl __sqrt1
	adiw r28,8
	rjmp __maxres
__sqrt1:
	push r20
	ldi  r20,66
	ldd  r24,y+6
	ldd  r27,y+5
	ldd  r26,y+4
__sqrt2:
	st   y,r24
	std  y+1,r25
	std  y+2,r26
	std  y+3,r27
	movw r30,r26
	movw r22,r24
	ldd  r26,y+4
	ldd  r27,y+5
	ldd  r24,y+6
	ldd  r25,y+7
	rcall __divf21
	ld   r24,y
	ldd  r25,y+1
	ldd  r26,y+2
	ldd  r27,y+3
	rcall __addf12
	rcall __unpack1
	dec  r23
	rcall __repack
	ld   r24,y
	ldd  r25,y+1
	ldd  r26,y+2
	ldd  r27,y+3
	eor  r26,r30
	andi r26,0xf8
	brne __sqrt4
	cp   r27,r31
	cpc  r24,r22
	cpc  r25,r23
	breq __sqrt3
__sqrt4:
	dec  r20
	breq __sqrt3
	movw r26,r30
	movw r24,r22
	rjmp __sqrt2
__sqrt3:
	pop  r20
	pop  r21
	adiw r28,8
	ret

__SUBD12:
	SUB  R30,R26
	SBC  R31,R27
	SBC  R22,R24
	SBC  R23,R25
	RET

__ORD12:
	OR   R30,R26
	OR   R31,R27
	OR   R22,R24
	OR   R23,R25
	RET

__ANEGD1:
	COM  R31
	COM  R22
	COM  R23
	NEG  R30
	SBCI R31,-1
	SBCI R22,-1
	SBCI R23,-1
	RET

__LSLD12:
	TST  R30
	MOV  R0,R30
	MOVW R30,R26
	MOVW R22,R24
	BREQ __LSLD12R
__LSLD12L:
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R23
	DEC  R0
	BRNE __LSLD12L
__LSLD12R:
	RET

__LSLD16:
	MOV  R22,R30
	MOV  R23,R31
	LDI  R30,0
	LDI  R31,0
	RET

__CWD1:
	MOV  R22,R31
	ADD  R22,R22
	SBC  R22,R22
	MOV  R23,R22
	RET

__PUTPARD1:
	ST   -Y,R23
	ST   -Y,R22
	ST   -Y,R31
	ST   -Y,R30
	RET

__SWAPD12:
	MOV  R1,R24
	MOV  R24,R22
	MOV  R22,R1
	MOV  R1,R25
	MOV  R25,R23
	MOV  R23,R1

__SWAPW12:
	MOV  R1,R27
	MOV  R27,R31
	MOV  R31,R1

__SWAPB12:
	MOV  R1,R26
	MOV  R26,R30
	MOV  R30,R1
	RET

__CPD12:
	CP   R30,R26
	CPC  R31,R27
	CPC  R22,R24
	CPC  R23,R25
	RET

__SAVELOCR6:
	ST   -Y,R21
__SAVELOCR5:
	ST   -Y,R20
__SAVELOCR4:
	ST   -Y,R19
__SAVELOCR3:
	ST   -Y,R18
__SAVELOCR2:
	ST   -Y,R17
	ST   -Y,R16
	RET

__LOADLOCR6:
	LDD  R21,Y+5
__LOADLOCR5:
	LDD  R20,Y+4
__LOADLOCR4:
	LDD  R19,Y+3
__LOADLOCR3:
	LDD  R18,Y+2
__LOADLOCR2:
	LDD  R17,Y+1
	LD   R16,Y
	RET

__INITLOCB:
__INITLOCW:
	ADD  R26,R28
	ADC  R27,R29
__INITLOC0:
	LPM  R0,Z+
	ST   X+,R0
	DEC  R24
	BRNE __INITLOC0
	RET

;END OF CODE MARKER
__END_OF_CODE:
