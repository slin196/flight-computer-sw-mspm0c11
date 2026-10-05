/*
 * helper_functions.c
 *
 * Created: 09/01/2026 09:52:52
 *  Author: slin9
 */ 

#include <stdint.h>

/*uint8_t num_of_free_msbs_after_multiplication(uint16_t op_1, uint16_t op_2) {
	uint8_t msb_position;
	for (msb_position = 0; op_1 >> msb_position; ++msb_position); //find the position of msb bit

	uint8_t bits_left = 16-msb_position; //TODO: immer pr�fen das bits_left nicht negativ wird
	for (msb_position = 0; op_2 >> msb_position; ++msb_position);
	if ((msb_position + 1) >= bits_left) return 0;
	bits_left -= (msb_position + 1);
	return bits_left;
}*/

/*returns the number of 0 - msb's before first 1 - bit in the result of 16 bit multiplication.
  In case of overflow it returns the number of overflowed bits as a negative number
*/
int8_t num_of_free_msbs_after_16bit_mult(uint16_t op_1, uint16_t op_2) {
	uint8_t msb_position;
	uint32_t result = (uint32_t)op_1 * (uint32_t)op_2;
	for (msb_position = 0; result >> msb_position; ++msb_position); //find the position of msb bit
	return 16-(int8_t)msb_position;
}

char num_to_char_helper(uint8_t num) {
	switch (num) {
		case 0: return '0';
		case 1: return '1';
		case 2: return '2';
		case 3: return '3';
		case 4: return '4';
		case 5: return '5';
		case 6: return '6';
		case 7: return '7';
		case 8: return '8';
		case 9: return '9';
	}
	return 'x';
}

//this function assumes string is 5 byte long
uint8_t num_to_char(int16_t num, char* string) {
	//if (string[5] != '\0') return 0; //error TODO: das hier liefert einen ab und zu 0 zur�ck, wieso das so ist gilt es vll. noch irgendwann
	//herauszufinden
	volatile uint8_t mod, five = 5;
	for (uint8_t i = 0; i<five; ++i) string[i] = '_';
	if (num<0) string[0] = '-';
	int16_t div_by_ten;
	while(five&&num) { //2**16 is five digits long in decimal notation
		div_by_ten = num/10;
		if (num<0) mod = (div_by_ten*10) - num;
		else mod = num - (div_by_ten*10);
		num /= 10;
		string[--five] = num_to_char_helper(mod);
	}
	return 1;
}

uint8_t num_to_char_abs(uint16_t num, char* string) {
	//if (string[5] != '\0') return 0; //error TODO: das hier liefert einen ab und zu 0 zur�ck, wieso das so ist gilt es vll. noch irgendwann
	//herauszufinden
	volatile uint8_t mod, five = 5;
	for (uint8_t i = 0; i<five; ++i) string[i] = '_';
	int16_t div_by_ten;
	while(five&&num) { //2**16 is five digits long in decimal notation
		div_by_ten = num/10;
		mod = num - (div_by_ten*10);
		num /= 10;
		string[--five] = num_to_char_helper(mod);
	}
	return 1;
}




