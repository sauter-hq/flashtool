#ifndef HAMMINGROMCODE_H_
#define HAMMINGROMCODE_H_

/**
 * \brief Computes a 3 bytes Hamming code 1-bit ECC compliant to the TI-OMAP 3430/3530
 * ROM code.
 *
 * \param buf 512 bytes to compute ECC on it.
 * \param code Resulting 3 bytes ECC.
 *
 * \sa hamming_romcode_compute_ecc in implementation file. Documentation provides detailed
 * information on ECC layout.
 */
void hammingromcode_calculate_ecc(const unsigned char *buf, unsigned char *code);

#endif /* HAMMINGROMCODE_H_ */
