/*
 * Board specific setup info
 *
 * (C) Copyright 2003, ARM Ltd.
 * Philippe Robin, <philippe.robin@arm.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <version.h>

.globl lowlevel_init
lowlevel_init:

init_dram:
    mov r4, lr		/* save lr */

    /*Set Scratch register Bit 7 before initialize*/
    /* *(scu1+ 0) = 0xa8 */
    ldr r0, =0x1e6e2000
    ldr r1, =0x1688a8a8
    str r1, [r0]

    /*Patch for AST1100/2100 A2 */
#ifdef CONFIG_AST1100A2_PATCH
    ldr r0, =0x1e6e207c
    ldr r1, [r0]
    ldr r2, =0x000000FF
    and	r1, r1, r2
    cmp	r1, #0x02
    blt _exit_A2_Patch
    
    ldr r0, =0x1e6e2030
    ldr r1, =0x20101A03
    str r1, [r0]
_exit_A2_Patch:
#endif
    
    ldr r0, =0x1e6e2040
    ldr r1, [r0]
    orr r1, r1, #0x80
    str r1, [r0]

    /*Check Stratch Register Bit 6*/
    ldr r0, =0x1e6e2040
    ldr r1, [r0]
    bic r1, r1, #0xFFFFFFBF
    mov r2, r1, lsr #6
    cmp r2, #0x01
    beq platform_exit

    /* DRAM Setting */
#ifdef 	CONFIG_AST1100_FPGA    
    ldr r0, =0x1e6e0000
    ldr r1, =0xfc600309
    str r1, [r0]

    ldr r0, =0x1e6e0004
    ldr r1, =0x00000484
    str r1, [r0]
      
    ldr r0, =0x1e6e0008
    ldr r1, =0x0011030f		/* VGA */
    str r1, [r0]

    ldr r0, =0x1e6e0010
    ldr r1, =0x01001726
    str r1, [r0]

    ldr r0, =0x1e6e0018
    ldr r1, =0x05220103
    str r1, [r0]

    ldr r0, =0x1e6e0020
    ldr r1, =0x00aa0000
    str r1, [r0]

    ldr r0, =0x1e6e0014
    ldr r1, =0x01001726
    str r1, [r0]

    ldr r0, =0x1e6e001c
    ldr r1, =0x05220103
    str r1, [r0]

    ldr r0, =0x1e6e0024
    ldr r1, =0x00aa0000
    str r1, [r0]

    ldr r0, =0x1e6e0038
    ldr r1, =0xffffff02
    str r1, [r0]

    ldr r0, =0x1e6e003c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0040
    ldr r1, =0x66666666
    str r1, [r0]

    ldr r0, =0x1e6e0044
    ldr r1, =0x66666666
    str r1, [r0]

    ldr r0, =0x1e6e0048
    ldr r1, =0x66666660
    str r1, [r0]

    ldr r0, =0x1e6e004c
    ldr r1, =0x66666666
    str r1, [r0]

    ldr r0, =0x1e6e0050
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0054
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0058
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e005c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0060
    ldr r1, =0x0f000000
    str r1, [r0]
                 
    ldr r0, =0x1e6e0064
    ldr r1, =0x00000000
    str r1, [r0]
 
    ldr r0, =0x1e6e0068
    ldr r1, =0x00000000
    str r1, [r0]
 
    ldr r0, =0x1e6e0070
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0074
    ldr r1, =0x00000000
    str r1, [r0]
  
    ldr r0, =0x1e6e0078
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e007c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0034
    ldr r1, =0x00000001
    str r1, [r0]
   
    /* Delay about 400us */
    ldr r2, =0x00000400 
delay0:	
    nop
    nop
    subs r2, r2, #1          /* until source end addreee [r2]    */
    bne delay0	 
	
    ldr r0, =0x1e6e002c
    ldr r1, =0x00000532
    str r1, [r0]
 
    ldr r0, =0x1e6e0030
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000005
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000007
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000003
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r0, =0x1e6e000c
    ldr r1, =0x0000b408
    str r1, [r0]

    ldr r0, =0x1e6e002c
    ldr r1, =0x00000432
    str r1, [r0]
  
    ldr r0, =0x1e6e0028
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r0, =0x1e6e0030
    ldr r1, =0x00000384
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000003
    str r1, [r0]

    ldr r0, =0x1e6e0030
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000003
    str r1, [r0]

    ldr r0, =0x1e6e000c
    ldr r1, =0x0000a008
    str r1, [r0]

    ldr r0, =0x1e6e0034
    ldr r1, =0x00000003
    str r1, [r0]
#else		//Real Chip

#ifdef CONFIG_DDR512_200
/* DDR 512Mx16 200MHz */
    ldr r0, =0x1e6e2020
    ldr r1, =0x000041f0		//200MHz
    str r1, [r0]

    ldr r0, =0x1e6e0000
    ldr r1, =0xfc600309
    str r1, [r0]

    ldr r0, =0x1e6e006C
    ldr r1, =0x00909090
    str r1, [r0]

    ldr r0, =0x1e6e0064
    ldr r1, =0x00050000
    str r1, [r0]

    ldr r0, =0x1e6e2070		//Set VGA size trapping to SDC
    ldr r1, [r0]
    ldr r2, =0x0000000C
    and	r1, r1, r2
    mov	r2, r1, lsl #2
    
    ldr r0, =0x1e6e0004
    ldr r1, =0x00000585
    orr r1, r1, r2
    str r1, [r0]
      
    ldr r0, =0x1e6e0008
    ldr r1, =0x0011030F		/* VGA */
    str r1, [r0]

    ldr r0, =0x1e6e0010
    ldr r1, =0x22202633
    str r1, [r0]

    ldr r0, =0x1e6e0018
    ldr r1, =0x1208020e
    str r1, [r0]

    ldr r0, =0x1e6e0020
    ldr r1, =0x00a82222
    str r1, [r0]

    ldr r0, =0x1e6e0014
    ldr r1, =0x01001523
    str r1, [r0]

    ldr r0, =0x1e6e001c
    ldr r1, =0x1024010d
    str r1, [r0]

    ldr r0, =0x1e6e0024
    ldr r1, =0x00cb2522
    str r1, [r0]

    ldr r0, =0x1e6e0038
    ldr r1, =0xffffff82
    str r1, [r0]

    ldr r0, =0x1e6e003c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0040
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0044
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0048
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e004c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0050
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0054
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0058
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e005c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0060
    ldr r1, =0x036AF022
    str r1, [r0]
                 
    ldr r0, =0x1e6e0064
    ldr r1, =0x002d2605
    str r1, [r0]
 
    ldr r0, =0x1e6e0068
    ldr r1, =0x02020202
    str r1, [r0]
 
    ldr r0, =0x1e6e0070
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0074
    ldr r1, =0x00000000
    str r1, [r0]
  
    ldr r0, =0x1e6e0078
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e007c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0034
    ldr r1, =0x00000001
    str r1, [r0]
   
    /* Delay about 400us */
    ldr r2, =0x00000400 
delay0:	
    nop
    nop
    subs r2, r2, #1          /* until source end addreee [r2]    */
    bne delay0	 
	
    ldr r0, =0x1e6e002c
    ldr r1, =0x00000132
    str r1, [r0]
 
    ldr r0, =0x1e6e0030
    ldr r1, =0x00000002
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000003
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r0, =0x1e6e000c
    ldr r1, =0x00005A08
    str r1, [r0]

    ldr r0, =0x1e6e002c
    ldr r1, =0x00000032
    str r1, [r0]
  
    ldr r0, =0x1e6e0028
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r0, =0x1e6e000c
    ldr r1, =0x00005A21
    str r1, [r0]

    ldr r0, =0x1e6e0034
    ldr r1, =0x0001c043
    str r1, [r0]

    ldr r0, =0x1e6e0120			/* for AST2000 compatible */
    ldr r1, =0x00004c41			/* 200MHz */
    str r1, [r0]

#else
/* DDRII 512Mx16 200MHz */
    ldr r0, =0x1e6e2020
    ldr r1, =0x000041f0		//200MHz
    str r1, [r0]

    ldr r0, =0x1e6e0000
    ldr r1, =0xfc600309
    str r1, [r0]

    ldr r0, =0x1e6e006C
    ldr r1, =0x00909090
    str r1, [r0]

    ldr r0, =0x1e6e0064
    ldr r1, =0x00050000
    str r1, [r0]

    ldr r0, =0x1e6e2070		//Set VGA size trapping to SDC
    ldr r1, [r0]
    ldr r2, =0x0000000C
    and	r1, r1, r2
    mov	r2, r1, lsl #2

#ifdef CONFIG_DDRII1G_200
    ldr r0, =0x1e6e0004
    ldr r1, =0x00000d89
    orr r1, r1, r2
    str r1, [r0]
#else    
    ldr r0, =0x1e6e0004
    ldr r1, =0x00000585
    orr r1, r1, r2
    str r1, [r0]
#endif
      
    ldr r0, =0x1e6e0008
    ldr r1, =0x0011030F		/* VGA */
    str r1, [r0]

    ldr r0, =0x1e6e0010
    ldr r1, =0x22201724
    str r1, [r0]

    ldr r0, =0x1e6e0018
    ldr r1, =0x1e29011a
    str r1, [r0]

    ldr r0, =0x1e6e0020
    ldr r1, =0x00c82222
    str r1, [r0]

    ldr r0, =0x1e6e0014
    ldr r1, =0x01001523
    str r1, [r0]

    ldr r0, =0x1e6e001c
    ldr r1, =0x1024010d
    str r1, [r0]

    ldr r0, =0x1e6e0024
    ldr r1, =0x00cb2522
    str r1, [r0]

    ldr r0, =0x1e6e0038
    ldr r1, =0xffffff82
    str r1, [r0]

    ldr r0, =0x1e6e003c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0040
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0044
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0048
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e004c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0050
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0054
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0058
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e005c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0060
    ldr r1, =0x032AA02a
    str r1, [r0]
                 
    ldr r0, =0x1e6e0064
    ldr r1, =0x002d3000
    str r1, [r0]
 
    ldr r0, =0x1e6e0068
    ldr r1, =0x00000000
    str r1, [r0]
 
    ldr r0, =0x1e6e0070
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0074
    ldr r1, =0x00000000
    str r1, [r0]
  
    ldr r0, =0x1e6e0078
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e007c
    ldr r1, =0x00000000
    str r1, [r0]

    ldr r0, =0x1e6e0034
    ldr r1, =0x00000001
    str r1, [r0]
   
    /* Delay about 400us */
    ldr r2, =0x00000400 
delay0:	
    nop
    nop
    subs r2, r2, #1          /* until source end addreee [r2]    */
    bne delay0	 
	
    ldr r0, =0x1e6e002c
    ldr r1, =0x00000732
    str r1, [r0]
 
    ldr r0, =0x1e6e0030
    ldr r1, =0x00000040
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000005
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000007
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000003
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r0, =0x1e6e000c
    ldr r1, =0x00005A08
    str r1, [r0]

    ldr r0, =0x1e6e002c
    ldr r1, =0x00000632
    str r1, [r0]
  
    ldr r0, =0x1e6e0028
    ldr r1, =0x00000001
    str r1, [r0]

    ldr r0, =0x1e6e0030
    ldr r1, =0x000003c0
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000003
    str r1, [r0]

    ldr r0, =0x1e6e0030
    ldr r1, =0x00000040
    str r1, [r0]

    ldr r0, =0x1e6e0028
    ldr r1, =0x00000003
    str r1, [r0]

    ldr r0, =0x1e6e000c
    ldr r1, =0x00005A21
    str r1, [r0]

    ldr r0, =0x1e6e0034
    ldr r1, =0x00007c03
    str r1, [r0]

    ldr r0, =0x1e6e0120			/* for AST2000 compatible */
    ldr r1, =0x00004c41			/* 200MHz */
    str r1, [r0]
#endif

#endif

    /*Set Scratch register Bit 6 after ddr initial finished */
    ldr r0, =0x1e6e2040
    ldr r1, [r0]
    orr r1, r1, #0x40
    str r1, [r0]

platform_exit:
    mov lr, r4		/* restore lr */
 
    /* back to arch calling code */    
    mov pc, lr

