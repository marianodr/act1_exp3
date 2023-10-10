// *******************************************************************//
// Rodriguez y Silke 30/09/2023
// Programa contador de paquetes de yerba
// Utiliza Interrupciones y Timer
// *******************************************************************//
// Definiciones
// ---------------------------------------------------------------------
#define F_CPU    16000000UL
#define BOUNCE_DELAY 8 //ms
#define DISPLAY_DELAY 5 //ms
#define ALARM_TIME 10000 //ms
#define ALARM_PERIOD 60000 //ms
#define MINTHR      5
#define MAXTHR      400

// Entradas:
#define P1      PD1           // Selecciona el umbral
#define P2      PD0           // Controla los modos de operacion: mcf(configuracion) y mct (contador)
#define P3      PD2           // Simula el sensor contador
// Salidas:
#define NUM0       PA0
#define NUM1       PA1
#define NUM2       PA2
#define NUM3       PA3
#define UNIT       PA4
#define TEN        PA5
#define HUND       PA6
#define ALARM      PA7
#define LEDMCF     PC6 //PB0
#define LEDMCT     PC7 //PB1

// Macros de usuario
// -------------------------------------------------------------------
#define	sbi(p,b)	p |= _BV(b)					//	sbi(p,b) setea el bit b de p.
#define	cbi(p,b)	p &= ~(_BV(b))				//	cbi(p,b) borra el bit b de p.
#define	tbi(p,b)	p ^= _BV(b)					//	tbi(p,b) togglea el bit b de p.
#define is_high(p,b)	(p & _BV(b)) == _BV(b)	//	is_high(p,b) p/testear si el bit b de p es 1.
#define is_low(p,b)		(p & _BV(b)) == 0		//	is_low(p,b) p/testear si el bit b de p es 0.

// Inclusion de archivos de cabecera
// -------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

// Variables globales
// -------------------------------------------------------------------
volatile int FlagP1 = 1, FlagP2 = 1, FlagP3 = 1, FlagALARM = 0, StateAlarm = 0;
int number = 0, count = 0, thresh = MINTHR, FlagDisplay = 1;
unsigned int ms_timer=0, actual = 0;
unsigned char unit, ten, hundred;

// Declararacion de funciones
// -------------------------------------------------------------------
void initPorts();
void boot();
void initExternalInterrupts();
void initTimer0();
void mcf();
void mct();
void alarm();
void display();
void decimalToBCD(unsigned char*, unsigned char*, unsigned char*);
void outBCD(int n);


// Interrupciones externas
// -------------------------------------------------------------------
ISR(INT0_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P2)){           // Comprueba si P2 sigue en BAJO
		tbi(FlagP2,0);                 // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT1_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P1)){           // Comprueba si P1 sigue en BAJO
		FlagP1 = 0;                 // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT2_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P3)){           // Comprueba si P3 sigue en BAJO
		FlagP3 = 0;                 // Establecer la bandera para indicar la interrupcion
	}
}

ISR(TIMER0_COMPA_vect) {
    // Cada 1 milisegundo ocurre una interrupcion e incrementa el timer
	ms_timer++;
}

// Programa
// -------------------------------------------------------------------
int main(void){

	initPorts();
	boot();
	initExternalInterrupts();
	initTimer0();
	sei();

	// Inicio
	while(1){
		if(FlagP2){
			mcf();
		}
		else{
			mct();
		}
		display();
		alarm();
	}
}

// Funciones
// ----------------------------------------------------------------------------------------
void initPorts(){
	// Se configura e inicializa los puertos
	DDRA = 0xFF;     // Puerto A todo como salida
	PORTA = 0x70;    // Inicializa el puerto A

	DDRC = 0XC0;	   // PC6 y PC7 como salida
	PORTC = 0X00;      // Inicializa PC6 y PC7

	// Se configura a PD0, PD1 y PD2 como puertos de entrada con resistencia pull-up internas:
	PORTD = (1 << P1) | (1 << P2) | (1 << P3);
}

void boot(){
	// Secuencia de Arranque
	int Seq[]={3,2,1};
	sbi(PORTA, NUM3); // Numero 8 en BCD
	sbi(PORTA,ALARM); //Enciende la alarma

	for(int i=0; i<3; i++){

		number=Seq[i];
		FlagDisplay = 1;

		for(int i=0; i<60; i++){
			display();
			_delay_ms(10);
		}
	}
	cbi(PORTA,ALARM);
	PORTA = 0x70; // Deshabilita los puertos que controlan los transistores
}

void initExternalInterrupts(){
	// Habilitacion de interrupciones externas:
	EICRA |= (1 << ISC01) | (1 << ISC11) | (1 << ISC21);;	// Configura INT0, INT1 e INT2 sensible a flanco desc.
	EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2);	    // Habilita INT0, INT1 e INT2
	EIFR = 0x00;
	//sei();							                    // Habilita las interrup. globalmente.
}

void initTimer0(){
    // Configura el modo CTC (Clear Timer on Compare Match)
    TCCR0A |= (1 << WGM02);

    // Configura el preescalador para que el temporizador cuente cada 64 microsegundos
	TCCR0B |= (1 << CS01) | (1 << CS00);

	// Calcula el valor de OCR0A para 1 milisegundo
	OCR0A = 249;

    // Habilita la interrupción de comparación para OCR0A
    TIMSK0 |= (1 << OCIE0A);
}

// Modo Configuracion
void mcf(){
	// Incremento del umbral
	if(!FlagP1){
		if(thresh!=MAXTHR){
			thresh++;
		}
		else {
			thresh=MINTHR;
		}
		FlagP1=1;
	}

	// Decremento del umbral (esta opcion no se solicita en la consigna)
	if(!FlagP3){
		if(thresh!=MINTHR){
			thresh--;
		}
		else {
			thresh=MAXTHR;
		}
		FlagP3=1;
	}

	// Actualizacion del numero a mostrar en el display
	if(number!=thresh){
		number = thresh;
		FlagDisplay=1;
	}

	// Enciende el led que indica el modo configuracion
	sbi(PORTC,LEDMCF);
	cbi(PORTC,LEDMCT);
}

// Modo Contador
void mct(){
	// Deteccion de paquete
	if(!FlagP3){
		if(count!=999){
			count++;
		}
		else {
			count=0;
		}
		FlagP3=1;
	}

	// Se enciende la alarma si la cantidad de packs alcanzó el umbral y si la alarma no se activo antes.
	if(count>=thresh && !StateAlarm){
		FlagALARM = 1;
	}if(count<thresh){
		StateAlarm=0;
	}

	// Reinicio de contador a traves de P1
	if(!FlagP1){
		count=0;
		FlagP1=1;
		StateAlarm = 0;
	}

	// Actualizacion del numero a mostrar en el display
	if(number!=count){
		number=count;
		FlagDisplay=1;
	}

	// Enciende el led que indica el modo CONTADOR
	sbi(PORTC,LEDMCT);
	cbi(PORTC,LEDMCF);
}

void display(){

	if(FlagDisplay){                          //si hubo un cambio en number, calcula sus digitos
		decimalToBCD(&unit, &ten, &hundred);
		FlagDisplay = 0;
	}

	// Muestra la unidad
	outBCD(unit);
	cbi(PORTA,UNIT);
	_delay_ms(DISPLAY_DELAY);
	sbi(PORTA,UNIT);

	// Muestra la decena
	if(ten != 0 || hundred != 0){
		outBCD(ten);
		cbi(PORTA,TEN);
		_delay_ms(DISPLAY_DELAY);
		sbi(PORTA,TEN);
	}

	// Muestra la centena
	if(hundred != 0){
		outBCD(hundred);
		cbi(PORTA,HUND);
		_delay_ms(DISPLAY_DELAY);
		sbi(PORTA,HUND);
	}
}

void decimalToBCD(unsigned char *unit, unsigned char *ten, unsigned char *hundred){
	*unit = (number % 10) & 0x0F;
	*ten = ((number/10) % 10) & 0x0F;
	*hundred = (number / 100) & 0x0F;
}

void outBCD(int n){
	PORTA &= 0xF0;                  // Limpia los 4 bits menos significativos correspondientes al codigo BCD

	if(n>=1 && n<=9){               // Verifica el rango de n
		PORTA |= (n & 0x0F);        // Asigna el numero n a los 4 bits inferiores de PORTA
	}
}

void alarm(){

	if(FlagALARM){
		//Variables de control
		StateAlarm = 1;
		ms_timer = 0;
		FlagALARM = 0;

	}

	if(StateAlarm){
		if(ms_timer < ALARM_TIME){
			sbi(PORTA,ALARM);
		}else if (ms_timer < ALARM_PERIOD){
			cbi(PORTA, ALARM);
		}else{
			TCNT0 = 0;
			ms_timer = 0;
		}
	}else{
		cbi(PORTA, ALARM);
	}

}
