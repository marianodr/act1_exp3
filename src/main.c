// *******************************************************************//
// Rodriguez y Silke 30/09/2023
// Programa contador de paquetes de yerba
// Utiliza Interrupciones y Timer
// *******************************************************************//
// Definiciones
// ---------------------------------------------------------------------
#define F_CPU    16000000UL
#define BOUNCE_DELAY 8 //ms
#define DISPLAY_DELAY 10 //ms
#define MINTHR      50
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
#define LEDMCF     PB0
#define LEDMCT     PB1

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
volatile int FlagP1 = 1, FlagP2 = 1, FlagP3 = 1;
int number = 0, count = 0, thresh = MINTHR, FlagDisplay = 1;
unsigned char unit, ten, hundred;

// Declararacion de funciones
// -------------------------------------------------------------------
void initPorts();
void boot();
void initExternalInterrupts();
void mcf();
void mct();
void alarm10();
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

// Programa
// -------------------------------------------------------------------
int main(void){

	initPorts();
	boot();
	initExternalInterrupts();

	// Inicio
	while(1){
		if(FlagP2){
			mcf();           // Analizar la posibilidad de que mcf y mct retornen y asignen a number
		}
		else{
			mct();
		}
		display();
	}
}

// Funciones
// ----------------------------------------------------------------------------------------
void initPorts(){
	// Se configura e inicializa los puertos
	DDRA = 0xFF;     // Puerto A todo como salida
	PORTA = 0x70;    // Inicializa el puerto A

	DDRB = 0x03;     // PB0 y PB1 como salida
	PORTB = 0x00;    // Inicializa PB0 y PB1

	// Se configura a PD0, PD1 y PD2 como puertos de entrada con resistencia pull-up internas:
	PORTD = (1 << P1) | (1 << P2) | (1 << P3);
}

void boot(){
	// Secuencia de Arranque
	sbi(PORTA, NUM3); // Numero 8 en BCD

	for(int i=0; i<5; i++){
		// Enciende y apaga display unidad
		cbi(PORTA, UNIT);       
		_delay_ms(DISPLAY_DELAY);
		sbi(PORTA, UNIT);

		// Enciende y apaga display decena
		cbi(PORTA, TEN);       
		_delay_ms(DISPLAY_DELAY);
		sbi(PORTA, TEN);

		// Enciende y apaga display centena
		cbi(PORTA, HUND);       
		_delay_ms(DISPLAY_DELAY);
		sbi(PORTA, HUND);

		_delay_ms(400);
	}

	PORTA = 0x70; // Deshabilita los puertos que controlan los transistores
}

void initExternalInterrupts(){
	// Habilitacion de interrupciones externas:
	EICRA |= (1 << ISC01) | (1 << ISC11) | (1 << ISC21);;	// Configura INT0, INT1 e INT2 sensible a flanco desc.
	EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2);	    // Habilita INT0, INT1 e INT2
	EIFR = 0x00;
	sei();							                        // Habilita las interrup. globalmente.
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

	if(number!=thresh){
		number = thresh;
		FlagDisplay=1;
	}
	
	// Enciende el led que indica el modo configuracion
	sbi(PORTB,LEDMCF);
	cbi(PORTB,LEDMCT);
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
	
	// Se enciende la alarma si la cantidad de packs alcanzó el umbral
	if(count==thresh)
		alarm10();
	
	if(!FlagP1){
		count=0;
		FlagP1=1;
	}

	if(number!=count){
		number=count;
		FlagDisplay=1;
	}

	// Enciende el led que indica el modo CONTADOR
	sbi(PORTB,LEDMCT);
	cbi(PORTB,LEDMCF);

}

void alarm10(){
	/* 
	alarm10:
	* Debe activar la salida ALARM durante 10 segundos.
	* ¿Como debe actuar el sistema durante la alarma? (si cambia de modo).
	* Utilizar Temporizador. */
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
	outBCD(ten);
	cbi(PORTA,TEN);
	_delay_ms(DISPLAY_DELAY);
	sbi(PORTA,TEN);

	// Muestra la centena
	outBCD(hundred);
	cbi(PORTA,HUND);
	_delay_ms(DISPLAY_DELAY);
	sbi(PORTA,HUND);

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