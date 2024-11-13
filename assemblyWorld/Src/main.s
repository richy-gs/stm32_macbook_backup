	.section	.data
A: 	.word		0

	.section	.text
	.syntax 	unified
	.global 	main

main:
	ldr  r0, =0x40021000//  RCC base address
	ldr  r1, [r0, #0x18]//  RCC_APB2ENR register actual value
	orr  r1, r1, #4//       set bit 2 to IO port A clock enable
	str  r1, [r0, #0x18]//  RCC_APB2ENR modified

	ldr  r0, =0x40010800//  GPIO Port A base address
	ldr  r1, =0x00200000//  value to reset pin5 of port A (LD2 is connected to PA5)
	str  r1, [r0, #0x10]//  GPIOx_BSRR register modified

	ldr  r1, [r0]//         GPIOx_CRL register actual value
	ldr  r2, =0xFF1FFFFF//	mask value
	and  r1, r1, r2//       clear bits 23 and 22 to configure pin as output, 21 for speed
	str  r1, [r0]//         GPIOx_CRL modified

	ldr  r1, [r0]//         GPIOx_CRL register actual value
	ldr  r2, =0x100000//    mask value
	orr  r1, r1, r2//       set bit 20 to select max speed 10MHz
	str  r1, [r0]//         GPIOx_CRL modified

LoopForever:
	mov  r1, #0x20//        value to set pin5 of port A (Turn-ON LD2)
	str  r1, [r0, #0x10]//	GPIOx_BSRR register modified
	ldr  r1, =0x00200000//  value to reset pin5 of port A (Turn-OFF LD2)
	str  r1, [r0, #0x10]//	GPIOx_BSRR register modified
	b    LoopForever
