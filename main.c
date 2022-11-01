/* Universidad del Valle de Guatemala
 IE2023 Programación de Microcontroladores
 Autor: Luis Pablo Carranza
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Laboratorio 4
 Hardware PIC16F887
 Creado: 10/10/22
 Última Modificación: 10/10/22*/

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSC 
//oscillator without clock out)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and 
//can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR 
//pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
//protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
//protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/
//External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-
//Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin 
//has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit 
//(Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
//(Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "oscilador.h"
#include <stdlib.h>
#include <string.h>
#define _XTAL_FREQ 8000000

//******************************************************************************
// Definición de funciones y variables
//******************************************************************************
void setup(void);
void initUART(void);        
void cadena(char *puntero);
void setup_ADC(void);

unsigned int valADC;
char buffer[sizeof(unsigned int)*8+1]; // Variable de buffer para conversión
                                       // de unsigned int a char
//******************************************************************************
// Función principal
//******************************************************************************
void main(void) {
    setup();            // Realiza la configuración de puertos
    setupINTOSC(7);     // Oscilador a 8 MHz
    initUART();         // Configuración para el módulo UART
    setup_ADC();        // Configuración para el módulo ADC
    
    while(1){
        // Imprime el siguiente encabezado en la terminal
        cadena("\r---------------------------MENU----------------------------");
        cadena("\r ELIGA UNA OPCION ESCRIBIENDO EL NUMERO");
        cadena("\r 1 - Leer potenciometro");
        cadena("\r 2 - Enviar ASCII");
        // Mientras no exista interrupción de la comunicación serial no hace nda
        while(PIR1bits.RCIF == 0); // Cuando hay interrupción continua
        // Revisa el valor que recibe desde la terminal
        if (RCREG == 0b00110001){  // Si es 1 en ASCII ejecuta
            cadena("\r LECTURA DEL POTENCIOMETRO");   
            ADCON0bits.CHS = 0b0000;    // Inicia el ADC
            ADCON0bits.GO = 1;
            while (ADCON0bits.GO == 1); // Revisa si terminó la conversión ADC
            ADIF = 0;                   // Apaga la bandera del ADC
            valADC = ((ADRESH << 2) + (ADRESL >> 6));
            // Junta los 10 bits en una variable
            cadena("\r VALOR ACTUAL: ");
            cadena(utoa(buffer,valADC,10)); // Convierte el valor de la variable
                                            // De binario a tipo cadena y lo 
                                            // Muestra en la terminal
            cadena("\r");
        }
        if (RCREG == 0b00110010){   // Si es 2 en ASCII ejecuta
             cadena("\r ENVIAR ASCII"); 
             cadena("\r ESCRIBA EL CARACTER QUE DESEA EN ASCII"); 
             while(PIR1bits.RCIF == 0); // Mientras escribe nada no hace nada
             PORTB = RCREG;             // Muestra el ASCII en el puerto B
             cadena("\r El ascii que esta mostrando es de: ");
             TXREG = PORTB;             // Muestra el ASCII en la terminal
             cadena("\r");
             PIR1bits.RCIF == 0;        // Apaga la interrupción de UART
             
        }
        PIR1bits.RCIF = 0;
        __delay_ms(500);
        
    }
    

}
//******************************************************************************
// Configuración de puertos
//******************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
    
    TRISB = 0;
    PORTB = 0;
    TRISD = 0;
    PORTD = 0; 
}
//******************************************************************************
// Configuración de módulo UART
//******************************************************************************
void initUART(void){
    // Configuración velocidad de baud rate
    SPBRG = 12;
    
    TXSTAbits.SYNC = 0;     // Modo asíncrono
    RCSTAbits.SPEN = 1;     // Habilitar módulo UART
    
    TXSTAbits.TXEN = 1;     // Habilitar la transmisión
    PIR1bits.TXIF = 0;
    
    RCSTAbits.CREN = 1;     // Habilitar la recepción
}
//******************************************************************************
// Función para enviar caracteres del PIC a la terminal
//******************************************************************************
void cadena(char *puntero){
    while(*puntero != '\0'){    // Mientras puntero no está en blanco ejecuta
        while (PIR1bits.TXIF == 0); // Si no hay nada en TXIF no hace nada
        TXREG = *puntero;           // Envía la siguiente cadena
        puntero++;                  // Incrementa el puntero
    }

}
//******************************************************************************
// Configuración de módulo ADC
//******************************************************************************
void setup_ADC(void){
    PORTAbits.RA0 = 0;      // Inicia el bit 0 de PORTA en 0
    TRISAbits.TRISA0 = 1;   // RA0 es entrada
    ANSELbits.ANS0 = 1;     // RA0 es analógico    
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;   // Fosc/8
    
    ADCON1bits.VCFG1 = 0;   // Ref VSS
    ADCON1bits.VCFG0 = 0;   // Ref VDD
    
    ADCON1bits.ADFM = 0;    // Justificado a la izquierda
    
    ADCON0bits.ADON = 1;    // Habilitar el convertidor ADC
    __delay_us(100);
}
    
    



