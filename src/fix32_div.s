
        .syntax     unified
        .thumb

        .global     fix32_div
        .type       fix32_div, "function"

        .global     fix32_inv_table
        .type       fix32_inv_table,    "object"
      
.align

fix32_inv_table:
        .4byte      0xf9fbfdfe, 0xf1f3f5f7, 0xeaeceef0, 0xe3e5e6e8
        .4byte      0xdcdddfe1, 0xd5d7d8da, 0xced0d2d3, 0xc8c9cbcd
        .4byte      0xc2c3c5c6, 0xbcbdbfc0, 0xb6b7b9ba, 0xb0b1b3b4
        .4byte      0xaaacadae, 0xa5a6a7a9, 0x9fa1a2a3, 0x9a9c9d9e
        .4byte      0x95969899, 0x90919394, 0x8b8d8e8f, 0x8788898a
        .4byte      0x82838486, 0x7e7f8081, 0x797a7b7c, 0x75767778
        .4byte      0x71727374, 0x6d6e6f70, 0x696a6b6c, 0x65666768
        .4byte      0x61626364, 0x5d5e5f60, 0x595a5b5c, 0x56575858
        .4byte      0x52535455, 0x4f505151, 0x4b4c4d4e, 0x48494a4b
        .4byte      0x45464647, 0x42424344, 0x3f3f4041, 0x3b3c3d3e
        .4byte      0x38393a3b, 0x35363738, 0x33333435, 0x30303132
        .4byte      0x2d2e2e2f, 0x2a2b2c2c, 0x2828292a, 0x25262627
        .4byte      0x22232424, 0x20202122, 0x1d1e1e1f, 0x1b1b1c1d
        .4byte      0x18191a1a, 0x16171718, 0x14141515, 0x11121213
        .4byte      0x0f101011, 0x0d0d0e0f, 0x0b0b0c0c, 0x09090a0a
        .4byte      0x06070708, 0x04050506, 0x02030304, 0x00010102      
        
@ Returns the quotient of two fixed-point numbers. The first two arguments
@ specify a dividend and non-zero divisor. The last argument specifies the
@ format of the first two arguments. For more information, see the function
@ description in the header file.
@
@ Execution time: 46-56 cycles
@ Absolute error: 2.0 LSB


	.thumb
	.thumb_func
fix32_div:
        push        {lr}

@ Splits the divisor to the magnitude and sign parts. The magnitude part goes
@ to the Newton-Raphson method, as it can only take positive values. The sign
@ part is combined with the dividend.

        tst         r1, r1
        itt	        mi
        negmi       r0, r0
        negmi       r1, r1

@ Normalizes the divisor to the range from one-half to one at the Q32 fixed
@ point representation. Finds the normalization shift which will be used to
@ scale the final result to the required fixed-point representation.

        clz         lr, r1
        lsl         r1, r1, lr
        rsb         lr, lr, #62
        sub         lr, lr, r2

@ Looks up on the nine most significant bits of the divisor to determine the
@ initial nine-bit Q8 estimate to its reciprocal. Because the leading bit of
@ the divisor is always set, it is not used to access the table. The leading
@ bit of a reciprocal is always set too, so the lookup table stores only its
@ eight least significand bits, the ninth bit is restored by software.

        ldr         ip, =fix32_inv_table
        lsr         r2, r1, #23
        sub         r2, r2, #256
        ldrb        ip, [ip, r2]
        add         ip, ip, #256

@ Performs the first Newton-Raphson iteration, producing the Q16 estimate
@ to the multiplicative inverse of the divisor.

        mul         r2, ip, ip
        umull       r3, r2, r1, r2
        rsb         r2, r2, ip, lsl #9

@ Performs the second Newton-Raphson iteration, producing the Q30 estimate
@ to the multiplicative inverse of the divisor.

        umull       ip, r3, r2, r2
        movs        ip, ip, lsr #2
        adc         ip, ip, r3, lsl #30
        umull       r3, ip, r1, ip
        add         ip, ip, r3, lsr #31
        rsb         ip, ip, r2, lsl #15

@ Multiplies the absolute value of a dividend by the multiplicative inverse
@ of a divisor. On the next step the resulting product will be denormalized
@ to get the actual quotient.

        smull       r0, r1, r0, ip

@ Performs the denormalization by arithmetically shifting the product from
@ the previous step to the right. Because the number of bits to be shifted
@ can be greater than 32, the operation is performed in two steps. The code
@ below partially shifts the quotient to reduce the number of places to be
@ shifted to no more than 32.

        subs        lr, lr, #32
        itte	gt
        movgt       r0, r1
        asrgt       r1, r1, #31
        addle       lr, lr, #32

@ Now, when the number of bits to be shifted is less than or equal to 32,
@ the code below finally denormalizes the quotient and rounds the result.

        lsrs        r0, r0, lr
        rsb         lr, lr, #32
        lsl         r1, r1, lr
        adc         r0, r0, r1
        pop         {pc}

        @ The reciprocal lookup table. The table consist of 256 eight-bit entries.
@ Each entry represents the eight least significant bits of a nine-bit Q8
@ reciprocal of a number in the range from one-half to one. The ninth bit
@ is ignored since it is always set and can be restored by software. The
@ entries are packed into 32-bit little-endian words.


        .end
        

