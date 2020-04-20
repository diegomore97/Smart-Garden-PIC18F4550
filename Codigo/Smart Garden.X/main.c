//ESTE SISTEMA USA RELOJ DE 24 HORAS
//DIEGO MORENO ARROYO

#define _XTAL_FREQ 4000000 //FRECUENCIA DEL CRISTAL A 4 MHZ
#include "config.h"
#include <stdint.h>
#include "I2C.h"
#include "RTC.h"
#include "UART.h"

#define TOTAL_SENSORES 3 //Sensores a Utilizar
#define MAX_SENSORES 8   //Maximo 8 sensores
#define REPETICIONES 6  //REPETICIONES PARA QUE TRANSCURRA 1 MINUTO
#define SENSIBILIDAD_HUMEDAD 60 //Este nos indica a partir de cuantos bits se
//considera seco el suelo

typedef struct {
    unsigned char humedadMedida; //0 - 255
    unsigned char pinSensor; //0 -7 PORT D

} SensorHumedad;

typedef struct {
    unsigned char hora; //0 - 23
    unsigned char regar; // Boolean
    unsigned char tiempoRegar; //Minutos que se regaran en esa hora
} Horario;

Horario horarios[24]; //24 HORAS DEL DIA
SensorHumedad sensores[MAX_SENSORES];
unsigned char hora = 0, minutos = 0, segundos = 0;
unsigned char datoRecibido = 0;
unsigned char overflowTimer = 0;
unsigned char byteUart = 0;

int VALOR_TIMER0 = 26473;
int contInterrupciones = 0;
unsigned char minutosRegar = 0;
unsigned char minutosTranscurridos = 0;
unsigned char regando = 0;

void inicializarObjetos(void);
void asignarHorarios(void); //MODULO ESP8266
int horaRegar(void);
int estaSeco(SensorHumedad s);
void constructorSensor(SensorHumedad s, unsigned char humedad, unsigned char pin);
void dameHoraActual(void); //MODULO RTC 3231
void setRtc(unsigned char direccion);
unsigned char leer_eeprom(uint16_t direccion);
void escribe_eeprom(uint16_t direccion, unsigned char dato);
void escribeHorariosMemoria(void);
void leeHorariosMemoria(void);
void setRtcDefault(void);
void fijaHoraRtc(void);
void setTiempoRegar(void);
void encenderBombas(void);
void restablecerDatosSensor(void);
void lecturaAnalogaSensores(void);
void lecturaWifi(void);
unsigned char dameHumedadSuelo(char canalLeer);
void configurarAdc(void);
void mostrarMenu(void);
void sistemaPrincipal(unsigned char opcion);
void sistemaRegado(void);

void __interrupt() desbordamiento(void) {

    if (INTCONbits.TMR0IF) {

        INTCONbits.TMR0IF = 0; //Regresando Bandera a 0 (Interrupcion por Timer 0)

        TMR0 = VALOR_TIMER0; //Overflow cada 10 Segundos

        overflowTimer = 1;

    }

    if (PIR1bits.RCIF) { // interrupción USART PIC por recepción

        byteUart = RCREG;
        datoRecibido = 1;

    }

}

void constructorSensor(SensorHumedad s, unsigned char humedad, unsigned char pin) {

    s.humedadMedida = humedad;
    s.pinSensor = pin;
}

void configurarAdc(void) {
    ADCON1 = 0b00000001; //VSS REFERENCIA|TODOS LOS CANALES ANALOGOS
    ADCON2 = 0b10100101; //TIEMPO DE ADQUISICION 8 TAD, JUSTIFICADO A LA DERECHA, FOSC/16
    TRISA = 1;
    TRISE = 1;

    PORTA = 0;
    PORTE = 0;
}

int estaSeco(SensorHumedad s) {
    return s.humedadMedida >= SENSIBILIDAD_HUMEDAD;
}

int horaRegar() {

    return horarios[hora].regar == 1;
}

void inicializarObjetos() {

    for (int i = 0; i < 24; i++) {
        horarios[i].hora = i;
        horarios[i].regar = 0;
        horarios[i].tiempoRegar = 15;
    }

}

void dameHoraActual() { //RTC DS3231

    segundos = convertirDato(leer_rtc(0x00));
    minutos = convertirDato(leer_rtc(0x01));
    hora = convertirDato(leer_rtc(0x02));
}

void setRtc(unsigned char direccion) {
    unsigned char dato;
    char datoCapturado = 0;

    for (int i = 0; i < 2; i++) {//en las direccienes que van desde 0 a 50
        char Rx = UART_read();
        if (Rx >= 48 && Rx <= 57) {
            if (!i) {
                dato = ((Rx - 48) & 0x0F) << 4;
            } else {
                dato |= (Rx - 48) & 0x0F;
                datoCapturado = 1;
            }
        } else {
            datoCapturado = 0;
            UART_printf("\r\n Solo se permiten numeros \r\n");
            UART_printf("\r\n Vuelva a grabar los datos \r\n");
            break;
        }
    }

    if (datoCapturado)
        escribe_rtc(direccion, dato);
}

void escribe_eeprom(uint16_t direccion, unsigned char dato) {
    i2c_inicia_com(); //inicia la comunicación serial i2C PIC
    i2c_envia_dato(0xAE); //envía la dirección del esclavo con el que el maestro se
    i2c_envia_dato(direccion >> 8); //envia parte alta de la direccion del registro
    //de la EEPROM donde se guardará el dato
    i2c_envia_dato(direccion); //envia parte baja de la direccion del registro
    //de la EEPROM donde se guardará el dato
    i2c_envia_dato(dato);
    i2c_detener();
    __delay_ms(10);
}

unsigned char leer_eeprom(uint16_t direccion) {

    unsigned char dato;

    i2c_inicia_com();
    i2c_envia_dato(0xAE);
    i2c_envia_dato(direccion >> 8);
    i2c_envia_dato(direccion);
    i2c_reinicia_com();
    i2c_envia_dato(0xAF);
    dato = i2c_recibe_dato();
    i2c_detener();
    __delay_ms(10);

    return dato;
}

void escribeHorariosMemoria() {

    int contMemoria = 0; //Variable que cuenta cuantos Bytes se ha escrito en la EEPROM
    for (int i = 0; i < 24; i++) {
        escribe_eeprom(contMemoria++, horarios[i].hora);
        escribe_eeprom(contMemoria++, horarios[i].regar);
        escribe_eeprom(contMemoria++, horarios[i].tiempoRegar);
    }
}

void leeHorariosMemoria() {

    int contMemoria = 0; //Variable que cuenta cuantos Bytes se han leido en la EEPROM

    for (int i = 0; i < 24; i++) {
        horarios[i].hora = leer_eeprom(contMemoria++);
        horarios[i].regar = leer_eeprom(contMemoria++);
        horarios[i].tiempoRegar = leer_eeprom(contMemoria++);
    }

}

void setRtcDefault(void) {
    unsigned char horaRtc;

    horaRtc = ((1) & 0x0F) << 4;

    horaRtc |= (2) & 0x0F;

    escribe_rtc(0x02, horaRtc); //HORA: 12

    escribe_rtc(0x01, 0); //MINUTOS: 0 MINUTOS
    escribe_rtc(0x00, 0); //SEGUNDOS: 0 SEGUNDOS
}

void encenderBombas() {

    char flagSeco = 0;

    for (int i = 0; i < TOTAL_SENSORES; i++) {

        switch (i) {

            case 0:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD0 = 1; //ENCERNDER BOMBA 1
                    flagSeco = 1;
                }
                break;

            case 1:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD1 = 1; //ENCERNDER BOMBA 2
                    flagSeco = 1;
                }
                break;

            case 2:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD2 = 1; //ENCERNDER BOMBA 3
                    flagSeco = 1;
                }
                break;

            case 3:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD3 = 1; //ENCERNDER BOMBA 4
                    flagSeco = 1;
                }
                break;

            case 4:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD4 = 1; //ENCERNDER BOMBA 5
                    flagSeco = 1;
                }
                break;

            case 5:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD5 = 1; //ENCERNDER BOMBA 6
                    flagSeco = 1;
                }
                break;

            case 6:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD6 = 1; //ENCERNDER BOMBA 7
                    flagSeco = 1;
                }
                break;

            case 7:
                if (estaSeco(sensores[i])) {
                    LATDbits.LATD7 = 1; //ENCERNDER BOMBA 8
                    flagSeco = 1;
                }
                break;

        }
    }

    if (flagSeco)
        regando = 1;

}

void fijaHoraRtc(void) {

    unsigned char dato = 0;

    UART_printf("\r\n FIJA HORA \r\n");

    //// Seccion Horas ///
    UART_printf("\r\n Envie las Horas en formato 24 Ej: 15 \r\n");
    setRtc(0x02);
    ///////Seccion minutos/////
    UART_printf("\r\n Envie los Minutos Ej: 25 \r\n");
    setRtc(0x01);

    UART_printf("\r\n HORA ESTABLECIDA CORRECTAMENTE \r\n");
    escribe_rtc(0x00, 0); //SEGUNDOS: 0 SEGUNDOS
}

void asignarHorarios() //ESP8266
{
    unsigned char hora;
    unsigned char Rx;
    char datoCapturado = 0;

    UART_printf("\r\n OPCIONES DE REGADO \r\n");
    UART_printf("\r\n Ingrese una hora en formato de 24 hrs ej: 15 \r\n");

    for (int i = 0; i < 2; i++) {//en las direccienes que van desde 0 a 50
        Rx = UART_read();
        if (Rx >= 48 && Rx <= 57) {
            if (!i) {
                Rx -= 48;
                hora = Rx;
                hora *= 10;
            } else {
                datoCapturado = 1;
                Rx -= 48;
                hora += Rx;
            }
        } else {
            datoCapturado = 0;
            UART_printf("\r\n Solo se permiten numeros \r\n");
            UART_printf("\r\n Vuelva a grabar los datos \r\n");
            break;
        }
    }

    if (datoCapturado) {

        UART_printf("\r\n Ingrese 1 para regar || ingrese 0 para no regar: \r\n");

        Rx = UART_read();
        Rx -= 48;

        if (Rx != 1 && Rx != 0) //Si no recibe cero o uno
            Rx = 0;

        UART_printf("\r\n Horario Modificado! \r\n");

        horarios[hora].regar = Rx;

        escribeHorariosMemoria(); //HACER ESTO INDEPENDIENTEMENTE DE SI MANDA DATO
        //POR UART

    }

}

void setTiempoRegar() {

    unsigned char hora;
    unsigned char tiempoRegar;
    unsigned char Rx;
    char datoCapturado = 0;

    UART_printf("\r\n TIEMPO DE REGADO \r\n");
    UART_printf("\r\n Ingrese una hora en formato de 24 hrs ej: 15 \r\n");

    for (int i = 0; i < 2; i++) {//en las direccienes que van desde 0 a 50
        Rx = UART_read();
        if (Rx >= 48 && Rx <= 57) {
            if (!i) {
                Rx -= 48;
                hora = Rx;
                hora *= 10;
            } else {
                datoCapturado = 1;
                Rx -= 48;
                hora += Rx;
            }
        } else {
            datoCapturado = 0;
            UART_printf("\r\n Solo se permiten numeros \r\n");
            UART_printf("\r\n Vuelva a grabar los datos \r\n");
            break;
        }
    }

    if (datoCapturado) {

        UART_printf("\r\n Ingrese los minutos que desee que se riegue en ese horario ej: 15 \r\n");

        for (int i = 0; i < 2; i++) {//en las direccienes que van desde 0 a 50
            Rx = UART_read();
            if (Rx >= 48 && Rx <= 57) {
                if (!i) {
                    Rx -= 48;
                    tiempoRegar = Rx;
                    tiempoRegar *= 10;
                } else {
                    datoCapturado = 1;
                    Rx -= 48;
                    tiempoRegar += Rx;
                }
            } else {
                datoCapturado = 0;
                UART_printf("\r\n Solo se permiten numeros \r\n");
                UART_printf("\r\n Vuelva a grabar los datos \r\n");
                break;
            }
        }

        if (datoCapturado) {

            UART_printf("\r\n Minutos de riego establecidos! \r\n");

            horarios[hora].tiempoRegar = tiempoRegar;
            minutosRegar = horarios[hora].tiempoRegar;

            escribeHorariosMemoria();

        }

    }

}

unsigned char dameHumedadSuelo(char canalLeer) {

    __delay_us(20);

    ADCON0bits.ADON = 1; //INICIAR ADC
    ADCON0bits.CHS = canalLeer;
    ADCON0bits.GO = 1;
    ADCON0bits.GO_DONE = 1; //Bandera en 1

    while (ADCON0bits.GO_DONE);

    ADCON0bits.ADON = 0; //APAGAR ADC

    return (ADRESH << 8) +ADRESL;

}

void restablecerDatosSensor() {
    for (int i = 0; i < TOTAL_SENSORES; i++)
        constructorSensor(sensores[i], 0, i);
}

void lecturaWifi() {

    char Rx;

    for (int i = 0; i < TOTAL_SENSORES; i++) {

        __delay_ms(5);

        Rx = UART_read();
        Rx -= 48;

        if (Rx == 1) //1 Si esta seco | 0 si esta humedo
            sensores[i].humedadMedida = 60;
        else
            sensores[i].humedadMedida = 0;
    }

}

void lecturaAnalogaSensores() {

    for (int i = 0; i < TOTAL_SENSORES; i++) {
        sensores[i].humedadMedida = dameHumedadSuelo(i);
        __delay_ms(5);
    }

}

void mostrarMenu(void) {

    UART_printf("\r\n Ingresa una opcion a Realizar: \r\n");
    UART_printf("\r\n 1. Fijar Hora Actual \r\n");
    UART_printf("\r\n 2. Asignar Horarios para regar \r\n");
    UART_printf("\r\n 3. Programar tiempo de riego en un horario \r\n");
    UART_printf("\r\n Opcion:  \r");
    UART_printf("\r\n");
}

void sistemaPrincipal(unsigned char opcion) {

    PIE1bits.RCIE = 0; //deshabilita interrupción por recepción USART PIC.

    switch (opcion) {

        case 1:
            fijaHoraRtc();
            break;

        case 2:
            asignarHorarios();
            break;

        case 3:
            setTiempoRegar();
            break;


        default:
            UART_printf("\r\n Solo se permiten numeros del 1 al 3 \r\n");
            break;
    }

    mostrarMenu();
    PIE1bits.RCIE = 1; //habilita interrupción por recepción USART PIC.

}

void sistemaRegado(void) {
    
    //UART_printf(".\n"); //Para verificar si se desborda el timer cada 10 seg

    if (regando) {
        contInterrupciones++;

        if (contInterrupciones == REPETICIONES) {
            contInterrupciones = 0;
            minutosTranscurridos++;
            //UART_printf("MINUTO TRANSCURRIDO!\n");

            if (minutosTranscurridos >= minutosRegar) {
                //UART_printf("\n\nRiego Finalizado!\n");
                LATD = 0; //POWER OFF MOTOR
                minutosTranscurridos = 0;
                regando = 0;
            }
        }

    } else {
        
        dameHoraActual();

        if (horaRegar() && !minutos) {
            
            //UART_printf("\n\nRiego Iniciado!\n");
            
            //lecturaWifi();
            lecturaAnalogaSensores();

            minutosRegar = horarios[hora].tiempoRegar;
            encenderBombas();
        }
    }

}

void main(void) {

    INTCONbits.GIE = 1; //GLOBALS INTERRUPTIONS ENABLED
    INTCONbits.PEIE = 1; // PERIPHERAL INTERRUPTIONS ENABLED
    INTCONbits.TMR0IE = 0; //INTERRUPTION BY TIMER 0 OFF
    INTCONbits.TMR0IF = 0; //INITIALIZING FLAG
    PIE1bits.RCIE = 1; //habilita interrupción por recepción USART PIC.

    T0CON = 0b00000111; //Timer 0 apagado, 16 bits , Temporizador, Prescaler 1:256

    INTCONbits.TMR0IE = 1; //INTERRUPTION BY TIMER 0 ON

    //Matriz bidimensional que represente en las filas las
    //24 horas del dia y en las columnas si se regara en ese horario

    restablecerDatosSensor();

    UART_init(9600); //9600 Baudios
    i2c_iniciar();
    configurarAdc();
    inicializarObjetos();

    leeHorariosMemoria();
    //setRtcDefault(); //Comentar despues de programar el chip por primera vez y volver a programar

    TRISD = 0; //PUERTO D COMO SALIDA
    LATD = 0;

    TMR0 = VALOR_TIMER0; //Overflow cada 10 Segundos
    
    INTCONbits.TMR0IF = 1; //Inicializando Bandera del TIMER0
    
    T0CONbits.TMR0ON = 1; //Iniciar Timer 0
    
    mostrarMenu();
        
    while (1) {

        if (datoRecibido) {

            datoRecibido = 0; //Bajando bandera

            byteUart -= 48; //Convirtiendo ASCII A ENTERO

            sistemaPrincipal(byteUart);

        }

        if (overflowTimer) {

            overflowTimer = 0; //Bajando bandera
            sistemaRegado();
        }

    }
    return;
}
