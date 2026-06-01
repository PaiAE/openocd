/* SPDX-License-Identifier: GPL-2.0-or-later */

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

	/* Params:
	 * r0 - flash base (in), status (out)
	 * r1 - count
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address, workarea data
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - tmp
	 */

#define PT32_FLASH_CR1_OFFSET  	0x00 /* offset of CR1 register from flash reg base */
#define PT32_FLASH_CR2_OFFSET  	0x04 /* offset of CR2 register from flash reg base */
#define PT32_FLASH_KR1_OFFSET  	0x0C /* offset of KR1 register from flash reg base */
#define PT32_FLASH_KR2_OFFSET  	0x10 /* offset of KR2 register from flash reg base */
#define PT32_FLASH_SR1_OFFSET  	0x18 /* offset of SR1 register from flash reg base */
#define PT32_FLASH_AR_OFFSET  	0x20 /* offset of AR register from flash reg base */
#define PT32_FLASH_DR1_OFFSET  	0x24 /* offset of DR1 register from flash reg base */

	.thumb_func
	.global _start
_start:
	ldr		r7, =0x00000012
	str		r7, [r0, #PT32_FLASH_CR1_OFFSET]	/*AINC = 1, nws = 2*/
	ldr		r7, =0x3B6A0000
	str		r7, [r0, #PT32_FLASH_KR1_OFFSET] 	/*unlock KR1*/
	ldr		r7, =0x1C000000
	cmp		r4, r7
	bhs		change_key							/*if r4 >= r7, goto change_key*/
	ldr		r7, =0x0000ADEB
	b		key_load
change_key:
	ldr		r7, =0x000063D2
key_load:	
	str		r7, [r0, #PT32_FLASH_KR1_OFFSET] 	/*unlock AR*/
	str		r4, [r0, #PT32_FLASH_AR_OFFSET]		/* write target_address to AR register */
	ldr		r7, =0xB75C0000
	str		r7, [r0, #PT32_FLASH_KR2_OFFSET] 	/*unlock KR2*/
	ldr		r7, =0x0000D3A5
	str		r7, [r0, #PT32_FLASH_KR2_OFFSET] 	/*unlock CR2*/
	mov     r4, r2
	adds	r4, #8								/*set r4 = fifo addr, fifo addr = target address + wp(4) + rp(4)*/
	
wait_fifo:
	ldr 	r6, [r2, #0]	/* read wp */
	cmp 	r6, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r5, [r2, #4]	/* read rp */
	cmp 	r5, r6			/* wait until rp != wp */
	beq 	wait_fifo
	
	ldr		r6, [r5]		/* tmp = *rp */
	adds	r5, #4			/* rp += 4 */

progam:
	str		r6, [r0, #PT32_FLASH_DR1_OFFSET]	/* write data to DR1 register */	
	ldr		r6, =0x00000001
	str		r6, [r0, #PT32_FLASH_CR2_OFFSET]	/* start progam */
	
busy:
	ldr 	r6, [r0, #PT32_FLASH_SR1_OFFSET]	/* wait until DONE flag is set */
	movs	r7, #0x70							/* check the error bits */
	tst 	r6, r7
	bne 	error
	movs	r7, #1
	tst 	r6, r7
	beq 	busy
	str		r7, [r0, #PT32_FLASH_SR1_OFFSET]	/* clear the DONE flag */

test_wrap:	
	cmp 	r5, r3								/* wrap rp at end of buffer */
	bcc		no_wrap
	
	mov		r5, r4
	
no_wrap:
	str 	r5, [r2, #4]						/* store rp */
	subs	r1, r1, #1							/* decrement word count */
	cmp     r1, #0
	beq     exit								/* loop if not done */
	b		wait_fifo
	
error:
	str		r7, [r0, #PT32_FLASH_SR1_OFFSET]	/* clear the error bits */
	movs	r0, #0
	str 	r0, [r2, #4]						/* set rp = 0 on error */
	
exit:
	mov		r0, r6								/* return status in r0 */
	bkpt	#0
