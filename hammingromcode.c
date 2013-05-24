#include "hammingromcode.h"
/**
 * \file hammingromcode.c
 *
 * \author daminetreg (damien.buhl@lecbna.org)
 *
 * \brief The Hamming ROM Code 1-bit ECC for the TI-OMAP 3430/3530 hardware is a
 * an ECC code capable of detecting up to 2-bit errors in 512 byte chunks and
 * correct 1-bit errors.
 *
 * The hamming code is a well known algorithm, however the implementation used in
 * the TI-OMAP 3430/3530 ROM Code bootloader / hw is a little bit different of the
 * general hamming algorithm.
 *
 * The hardware expects a 3-bytes ECC containing even and odd parity informations
 * for columns and rows. This implementation considers that there are BYTE_SIZE_BITS
 * columns in a row and that there is CHUNK_SIZE_BYTES rows.
 *
 */

#include <stddef.h>
#include <stdint.h>
#include <tgmath.h>

#define CHUNK_SIZE_BYTES 512

/**
 * Defines how much bits there is in a Byte. Fix here to compute correctly on funny 
 * platforms where CHAR_BIT could be different.
 */
#define BYTE_SIZE_BITS 8

/**
 * Mask to compute even parity for all columns
 */
const uint8_t EVEN_WHOLE = 0xFF;

/**
 * Mask to compute even parity for the half of the columns
 */
const uint8_t EVEN_HALF = 0x0F;

/**
 * Mask to compute even parity for columns : 00 11 00 11
 */
const uint8_t EVEN_FOURTH = 0x33;

/**
 * Mask to compute even parity for columns : 01 01 01 01
 */
const uint8_t EVEN_EIGHTH = 0x55;

/**
 * Mask to compute odd parity for all columns
 */
const uint8_t ODD_WHOLE = 0x00;

/**
 * Mask to compute odd parity for the half of the columns.
 */
const uint8_t ODD_HALF = 0xF0;

/**
 * Mask to compute odd parity for columns : 11 00 11 00
 */
const uint8_t ODD_FOURTH = 0xCC;

/**
 * Mask to compute odd parity for columns : 10 10 10 10
 */
const uint8_t ODD_EIGHTH = 0xAA;

/**
 * Enum used to define which kind of parity is computed.
 */
enum ParityType {
	Even = 0,
	Odd = 1
};

/**
 * \brief Computes a 3 bytes Hamming code 1-bit ECC compliant to the TI-OMAP 3430/3530
 * ROM code.
 *
 *
 * \param buf 512 bytes to compute ECC on it.
 * \param code Resulting 3 bytes ECC.
 *	 This lays out the ECC as it follows:
 *	
 *		- code[0] : contains 8 bits of the even row parity (the 9th bits of even 
 *		row parity is stored in code[2] because the ti hardware expects it so!)
 *	
 *		- code[1] : contains 8 bits of the odd row parity (the 9th bit of odd
 *		row parity is stored in code[2] because the ti hardware expects it so!)
 *	
 *		- code[2] : the first 3 least significant bits contains column even parity,
*		the 4th least significant bit contains the 9th even row partiy bit.
 *		Then comes the 4 most significant bits layed out the same way : the most
 *		significant bit contains the 9th bit of odd row parity, the 2dn to 4th 
 *		most significant bits contains the odd column parity of the computed ECC.
 *
 * 		0  1  2  3  4  5  6  7  8
 *		+--+--+--+--+--+--+--+--+
 *		|     even row parity   |
 *		+--+--+--+--+--+--+--+--+
 *		|     odd row parity    |
 *		+--+--+--+--+--+--+--+--+
 *		|9 | odd    |9 | even   |  Here 9th od and 9th ev are respectively
 *		|th| column |th| column |  the 9th bit of odd and even row parity.
 *		|od| parity |ev| parity |
 *		+--+--+--+--+--+--+--+--+
 */
void hamming_romcode_compute_ecc(const uint8_t *buf, uint8_t *code);

/**
 * \brief Compute the parity of the provided byte by xoring each bits together.
 *
 * \param value byte to check bit parity for.
 *
 * \param columnsMask For which columns should the parity be computed. (1 column == 1 bit, 
 * 										value always contains max 8 columns, but through the columnsMask 
 * 										parameter computation can be done only on the bit wanted).
 *
 * \return one byte with the most significant bit set or not following if the bit 
 * 					parity of the provided value byte is odd (return 1) or 
 * 					even (return 0). if we could have 1-bit type in the C language we would
 * 					return a single bit here.
 */
uint8_t compute_bitwise_parity(uint8_t value, uint8_t columnsMask);

/**
 * \brief Xor together the final row parity bits from the provided rowParities array which 
 * contains the parity for each row.  This compute the parity bit which should be stored 
 * at the parity position provided.
 *
 * \param rowParities even parity of the rows checked by this code.
 * \param parityType Whether to compute even or odd row parity.
 * \param parityBitPosition 
 * 					Which parity bit position has to be computed, this means which range is checked
 * 					by this parity bit. 
 *
 * 					Only power of 2 bit positions are parity bits, but this won't be checked by the
 * 					function, if you provide something else than a power of 2 position, expects to
 * 					get unexpected results because you would be violating a preconditions.
 *
 * 					Each parity bit covers all bits where the bitwise AND of the parity position 
 * 					and the bit position is non-zero :
 *
 * 					Examples (from wikipedia - http://en.wikipedia.org/wiki/Hamming_code) :
 *						- Parity bit 1 covers all bit positions which have the least significant bit set:
 *							bit 1 (the parity bit itself), 3, 5, 7, 9, etc.
 *						- Parity bit 2 covers all bit positions which have the second least significant 
 *							bit set: bit 2 (the parity bit itself), 3, 6, 7, 10, 11, etc.
 *						- Parity bit 4 covers all bit positions which have the third least significant 
 *							bit set: bits 4–7, 12–15, 20–23, etc.
 *						- Parity bit 8 covers all bit positions which have the fourth least significant 
 *							bit set: bits 8–15, 24–31, 40–47, etc.
 * \return 
 * 					the parity bit for the parityBitPosition chosen, that is to say a parity bit 
 * 					computed from the rowParities representing the parity of the covered bits range
 * 					that this parityBitPosition covers. 
 * 
 */
uint8_t compute_row_parity_bits(const uint8_t rowParities[CHUNK_SIZE_BYTES], int parityType, size_t parityBitPosition);

void hamming_romcode_compute_ecc(const uint8_t *buf, uint8_t *code) {
	//TODO: Assert buf is 512 byte long
	//TODO: Assert code is at least 3 bytes long.

	uint16_t evenParityBuffer=0;
	uint16_t oddParityBuffer=0;

	//Compute column even and odd parity
	uint8_t columnsParity = 0;
	size_t i;
	for (i = 0; i < CHUNK_SIZE_BYTES; ++i) {
		columnsParity ^= buf[i];
	}
	
	evenParityBuffer |= compute_bitwise_parity(columnsParity, EVEN_HALF) << 2 |
												compute_bitwise_parity(columnsParity, EVEN_FOURTH) << 1 |
												compute_bitwise_parity(columnsParity, EVEN_EIGHTH) << 0;

	oddParityBuffer |= compute_bitwise_parity(columnsParity, ODD_HALF) << 2 |
											compute_bitwise_parity(columnsParity, ODD_FOURTH) << 1 |
											compute_bitwise_parity(columnsParity, ODD_EIGHTH) << 0;

	//Compute rows even and odd parity
	uint8_t rowsParity[CHUNK_SIZE_BYTES] = { 0 };

	size_t j;
	for (j = 0; j < CHUNK_SIZE_BYTES; ++j) {
		rowsParity[j] = compute_bitwise_parity(buf[j], EVEN_WHOLE);
	}

	// Now that even parity is computed foreach rows, we should compute even + odd parity for them
	// and place them where they should (i.e. power of 2 positions as defined by the general hamming code algorithm) 
	
	size_t iterations = (log(CHUNK_SIZE_BYTES) / log(2));
	size_t k;
	for (k = 0; k < iterations; ++k) {
		uint8_t rowEvenParityBits = compute_row_parity_bits(rowsParity, Even, pow(2, k));
		evenParityBuffer |= (uint16_t) (rowEvenParityBits << (3+k));

		uint8_t rowOddParityBits = compute_row_parity_bits(rowsParity, Odd, pow(2, k));
		oddParityBuffer |= (uint16_t) (rowOddParityBits << (3+k));
	}

	code[0] = 0x00;
	code[1] = 0x00;
	code[2] = 0x00;

	code[0] = (uint8_t) (evenParityBuffer 		& 	0x00FF);
	code[1] = (uint8_t) (oddParityBuffer 			& 	0x00FF);
	code[2] = (uint8_t) ( ( (oddParityBuffer & 0x0F00) >> 4) | ((evenParityBuffer 	& 	0x0F00) >> 8) );
}

uint8_t compute_bitwise_parity(uint8_t value, uint8_t columnsMask) {
	uint8_t parity = 0;
	
	size_t i;
	for (i = 0; i < BYTE_SIZE_BITS; ++i) {

		if ((columnsMask & 0x1) != 0) { // Whether this is a column to check for.
			parity ^= (value & 0x1); // Clears the others bit that the one checked, to avoid xoring columns that shouldn't be xored.
		}

		// Remove checked column, continue with next one.
		columnsMask >>= 1;
		value >>= 1;
	}

	return (parity & 0x1); // Return only the least significant bit to tell whether it's even or odd. 
}

uint8_t compute_row_parity_bits(const uint8_t rowParities[CHUNK_SIZE_BYTES], int parityType, size_t parityBitPosition) {
	uint8_t parity = 0;

	size_t i;
	for (i = ( (parityType == Even) ? 0 : parityBitPosition); 
			i < CHUNK_SIZE_BYTES; 
			i += (2*parityBitPosition)) {
		
		size_t j;
		for (j = 0; j < parityBitPosition; ++j) {
			//printf("Computing row parity : %d/%d \n", (i+j), CHUNK_SIZE_BYTES);
			parity ^= rowParities[i + j];
		}

	}

	return (parity & 0x1); // Return only the least significant bit to tell whether it's even or odd. 
}

void hammingromcode_calculate_ecc(const unsigned char *buf, unsigned char *code) {
	hamming_romcode_compute_ecc(buf, code);
}
