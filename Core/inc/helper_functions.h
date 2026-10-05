/*
 * helper_functions.h
 *
 * Created: 09/01/2026 12:32:07
 *  Author: slin9
 */ 


#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

int8_t num_of_free_msbs_after_multiplication(uint16_t, uint16_t, uint16_t*);
int8_t num_of_free_msbs_after_16bit_mult(uint16_t op_1, uint16_t op_2);
uint8_t num_to_char(uint16_t num, char* string);
uint8_t num_to_char_abs(uint16_t num, char* string);
#endif /* HELPER_FUNCTIONS_H_ */