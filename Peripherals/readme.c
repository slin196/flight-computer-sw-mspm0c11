

/*
Wieso nicht peripherals und implements ordner zusammenschmeißen, wo do in den files hier lediglich die funktion aus den implements 
zuweisen? Peripherals ordner ist praktisch noch eine zusätzliche abstraktionsebene über den implements. hier werden die optionen die eine
peripherie aufweist festgelegt. beispiel: beispiel pwm kann phase correct, invertiert undso weiter sein, die implements stellen 
alle möglichen optionen bereit und an die interfaces bindet dann eine der optionen:
beispiel
pwm_drv_if if = {
#ifdef PMW_PHASE_CORRECT
    .set_duty_cycle = pwm_phase_correct
#elif PWM_INVERTED 
    .set_duty_cylcle = etc...
}

die darüberliegenden motortreiber interessieren sich nicht ob ein pwm phase correct oder eine lesezugriff auf ein IMU-Register blockierend oder nicht erfolgt
erfolgt. <---die idee ist durchaus diskutabel, sowas wie ob ein pwm invertierend, center oder edge aligned ist, wird normalerweise in 
der initialisierung vorgenommen nicht in den funktionen die ständig aufgerufen werden um den duty cycle zu setzen
*/