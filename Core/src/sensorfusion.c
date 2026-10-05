/*
 * sensorfusion.c
 *
 * Created: 29/12/2025 20:09:10
 *  Author: slin9
 */ 
#include <stdint.h>
#include "../inc/sys_settings.h"
#ifdef FIXED_POINT
#include "../inc/fix16.h"
#else 
#include <math.h>
#endif
#include "../inc/helper_functions.h"
#include "../inc/sensorfusion.h"

//atan2 //TODO: eine approximationsfunktion finden die nur mit ganzzahlarithmetik auskommt

#ifdef FIXED_POINT

#define BIT_MASK_LSBS(integer, num_of_lsbs) (integer & num_of_lsbs)

#define ALPHA_NUM_OF_SUB_COMMA_DIGITS (3)
#define SUB_COMMA_PRECISION (ALPHA_NUM_OF_SUB_COMMA_DIGITS)
#define ALPHA_SUB_COMMA_MASK ((1<<ALPHA_NUM_OF_SUB_COMMA_DIGITS) - 1)
#define ALPHA (14)
#define BETA ((1<<ALPHA_NUM_OF_SUB_COMMA_DIGITS)-(ALPHA&ALPHA_SUB_COMMA_MASK))

void complementary_filter(attitude* state, sensordata gyro, sensordata acc, uint8_t dt) {//noch festlegen in welcher einheit dt die zeit angibt
	//TODO: achtung der fix_16_t ist als int32_t getypedef und viele konstanten die in den funktionen vorkommen, sind mehr als 16 bit breit
	//dies gilt es vll. noch zu korrigieren im file fix_16.h, 32 bit genauigkeit ist schon allein wegen der eingangswerte wahrscheinlich unn�tig
	attitude angl_acc, angl_gyro;//TODO: die funktion fix16_atan2 ruft die funktion fix16_div auf, im file fix16.c gibt es von der zwei funktionen
	//je nachdem welches makro aktiv ist. eine der zwei ist besonders f�r atmel avr ausgelegt. stelle sicher das das entsprechnde makro aktiviert ist
	//untersuche nochmal welche funktion um wie viel schneller ist
	angl_acc.roll = fix16_atan2(acc.a_y, acc.a_z) >> 16; //TODO: dies funktion noch austesten, was f�r ergebnisse liefert sie? beachte das sie werte
	//zur�ck gibt die einen �berlauf indizieren evtl. noch entsprechend abfragen
	//TODO: evtl. weiterhin die accelometer ergebnisse von g in m/s2 umrechen bevor du sie in die atan2 funktion passt.
	//TODO: die fix16 lib arbeitet mit int32_t datentypen, vll das ganze noch so umschreiben das sie mit 8 oder 16 bit arbeitet,
	//wenn diese datentypen eine f�r dieses ausreichende genauigkeit bieten
	angl_acc.pitch = fix16_atan2(-acc.a_x, fix16_sqrt(acc.a_y*acc.a_y + acc.a_z*acc.a_z)) >> 16;//TODO: wie viele bits werden hier als 
	//nachkommastellen vorgesehen?
	angl_acc.yaw = 0; //TODO: 0 weil kein magnetometer vorhanden ist. das probl ist das dann yaw nur durch die gyroskop daten
	//bestimmt wird und das gyroskop einen leichten drift hat
	//uint16_t gyro_term = gyro;
	//state->x = alpha;
	angl_gyro.pitch = (state->pitch + dt * gyro.a_x); //TODO:schau das pitch und a_x diesselbe einheit haben, d.h. bogenma� oder gradma� beiderseits sind
	//TODO: achtung die einheit die f�r pitch, roll, yaw entsteht ist grad/sekunde nicht grad!! das hast du w�hrend du den code geschrieben hast
	//falsch im kopf gehabt (siehe definition von sensordata f�r zitat). vll. musst das noch ber�cksichtigen und den code entsprechend anpassen
	angl_gyro.roll = (state->roll + dt * gyro.a_y); //TODO: �berl�ufe gilt es auch auszuschlie�en
	angl_gyro.yaw = (state->yaw + dt * gyro.a_z); //TODO: der state muss noch initialisert werden bevor du funktion aufrufst sonst sind die werte
	//undefiniert. die drohne muss dann wahrscheinlich im startzustand diesen state dann auc wirklich haben,, oder pendelt sich das ein?
	
	state->pitch = (BETA*angl_gyro.pitch + ALPHA*angl_acc.pitch) >> SUB_COMMA_PRECISION; //TODO: die ganzen berechnungen sollten das gradma� verwenden
	//damit du sicherstellen kannst, das keine �berl�ufe erfolgen und trotzdem eine hohe genauigkeit herrscht
	
	state->roll = (BETA*angl_gyro.roll + ALPHA*angl_acc.roll) >> SUB_COMMA_PRECISION;
	
	state->yaw = (BETA*angl_gyro.yaw + ALPHA*angl_acc.yaw) >> SUB_COMMA_PRECISION;
}

#else

#define ALPHA  (0.8) //TODO: dann noch richtigen werte wählen
#define BETA (1.0 - ALPHA)
#define dt (MAIN_LOOP_RUNTIME_IN_S) //TODO: noch richtigen wert hier zuweisen


//this functions shall take the gyroscope values as degrees/second, that is in 
void complementary_filter(attitude* state, sensordata gyro, sensordata acc) {//noch festlegen in welcher einheit dt die zeit angibt
	//TODO: achtung der fix_16_t ist als int32_t getypedef und viele konstanten die in den funktionen vorkommen, sind mehr als 16 bit breit
	//dies gilt es vll. noch zu korrigieren im file fix_16.h, 32 bit genauigkeit ist schon allein wegen der eingangswerte wahrscheinlich unn�tig
	attitude angl_acc, angl_gyro;//TODO: die funktion fix16_atan2 ruft die funktion fix16_div auf, im file fix16.c gibt es von der zwei funktionen
	//je nachdem welches makro aktiv ist. eine der zwei ist besonders f�r atmel avr ausgelegt. stelle sicher das das entsprechnde makro aktiviert ist
	//untersuche nochmal welche funktion um wie viel schneller ist
	angl_acc.roll = atan2(acc.a_y, acc.a_z);
	angl_acc.roll *= (180.0f/M_PI); // convert radians into degrees

	angl_acc.pitch = atan2(-acc.a_x, sqrt(acc.a_y*acc.a_y + acc.a_z*acc.a_z));//TODO: wie viele bits werden hier als 
	//nachkommastellen vorgesehen?
	angl_acc.pitch *= (180.0f/M_PI); // convert radians into degrees
	angl_acc.yaw = 0.0f; //TODO: 0 weil kein magnetometer vorhanden ist. das probl ist das dann yaw nur durch die gyroskop daten
	//bestimmt wird und das gyroskop einen leichten drift hat
	//uint16_t gyro_term = gyro;
	//state->x = alpha;
	angl_gyro.pitch = (state->pitch + dt * gyro.a_x); 
	angl_gyro.roll = (state->roll + dt * gyro.a_y); //TODO: �berl�ufe gilt es auch auszuschlie�en
	angl_gyro.yaw = (state->yaw + dt * gyro.a_z); //TODO: der state muss noch initialisert werden bevor du funktion aufrufst sonst sind die werte
	//undefiniert. die drohne muss dann wahrscheinlich im startzustand diesen state dann auc wirklich haben,, oder pendelt sich das ein?
	
	state->pitch = (BETA*angl_gyro.pitch + ALPHA*angl_acc.pitch); //TODO: die ganzen berechnungen sollten das gradma� verwenden
	//damit du sicherstellen kannst, das keine �berl�ufe erfolgen und trotzdem eine hohe genauigkeit herrscht
	
	state->roll = (BETA*angl_gyro.roll + ALPHA*angl_acc.roll);
	
	state->yaw = (BETA*angl_gyro.yaw + ALPHA*angl_acc.yaw);
}

#endif

/*
	int8_t bits_left, alpha, beta, right_shifts;
	alpha = ALPHA;
	beta = BETA;
	right_shifts = ALPHA_NUM_OF_SUB_COMMA_DIGITS
	//TODO: num_of_free_msbs_after_multiplication nimmt zwei unsigend integer als args entgegen. in der praxis treten allerdings auch
	//negative werte f�r pitch roll und yaw auf, das noch entsprechend bei der �bergabe ber�cksichtigen
	uint16_t factor = max(ALPHA)
	bits_left = num_of_free_msbs_after_16bit_mult(ALPHA, angl_acc.pitch); //TODO: error handling, verk�rze alpha in diesem fall
	if (bits_left < 0) {
		alpha >>= abs(bits_left); //TODO: alternativ k�nnte auch anhand ALPHA sehr lang machen, so 20-25 bits dann w�rde die maximale
		//pr�zision herrschen und ma
		beta >>= abs(bits_left);
		right_shifts += bits_left;
		//uint8_t shifts_right = min(bits_left, SUB_COMMA_PRECISION);
		
		state->pitch = (beta*angl_gyro.pitch + alpha*angl_acc.pitch) >> right_shifts;*/