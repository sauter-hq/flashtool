/*
 * bchtool.h
 *
 *  Created on: Apr 4, 2013
 *      Author: robert
 */

#ifndef BCHTOOL_H_
#define BCHTOOL_H_


int bch_calculate_ecc(const unsigned char *buf, unsigned char *code);
void free_ecc_memory();

#endif /* BCHTOOL_H_ */

