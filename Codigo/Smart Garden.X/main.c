//ESTE SISTEMA USA RELOJ DE 24 HORAS
//DIEGO MORENO ARROYO
//COMENTA LAS LINEAS QUE DICEN "COMENTAR" YA QUE ESAS SOLO SE UTILIZAN PARA 
//SIMULACION O DEBUGUEO

#define _XTAL_FREQ 4000000 //FRECUENCIA DEL CRISTAL A 4 MHZ
#include "config.h"
#include "I2C.h"
#include "RTC.h"
#include "UART.h"
#include "DHT11.h"
#include <stdio.h>
#include <stdint.h>

#define TEMPERATURA_MAX 34
#define TOTAL_SENSORES 3 //Sensores a Utilizar
#define MAX_SENSORES 8   //Maximo 8 sensores
#define REPETICIONES 6  //REPETICIONES PARA QUE TRANSCURRA 1 MINUTO
#define SENSIBILIDAD_HUMEDAD 94 //Este nos indica a partir de cuantos bits se
//considera seco el suelo esta expresado en un porcentaje del 0 al 100
#define MAX_TIEMPO_INACTIVIDAD 1 //Decenas de segundo de espera para que el 
//usuario setie datos en el sistema a traves del protocolo UART
#define TAMANO_CADENA 50   
//Tolerancia para lectura de sensores por wifi en minutos
#define TOLERANCIA_LECTURA 3 

//INSTRUCCIONES DE CONTROL
#define SETEO_DENEGADO 'F' //Variable que se mandara por UART a otro Micro
#define SOLICITUD_DATOS_SENSORES 'R' //Variable que se mandara por UART a otro Micro
#define ENVIANDO_DATOS_SENSORES 'O' //El otro micro ya nos confirmo que mandara los datos
#define INTERRUMPIR_COMANDOS 'I' //Se utiliza esta constante para la opcion de leer sensores
#define BITS_ADC 1023 //Indica el numero de bits de lectura de ADC en este caso 10 bits

typedef struct {
    unsigned char porcientoHumedad;
    unsigned char pinSensor; //0 -7 PORT D

} SensorHumedad;

typedef struct {
    unsigned char hora; //0 - 23
    unsigned char regar; // Boolean
    unsigned char tiempoRegar; //Minutos que se regaran en esa hora
} Horario;

unsigned char MODO_COMUNICACION; //0 NORMAL  | 1 WIFI
Horario horarios[24]; //24 HORAS DEL DIA
SensorHumedad sensores[MAX_SENSORES];
unsigned char hora = 0, minutos = 0, segundos = 0;
unsigned char datoRecibido = 0;
unsigned char overflowTimer = 0;

int VALOR_TIMER0 = 26473;
int contInterrupciones = 0;
unsigned char minutosRegar = 0;
unsigned char minutosTranscurridos = 0;
unsigned char regando = 0;
unsigned char peticionLecturaSensores = 0; //Flag para saber si se recibio la lectura
//de sensores via wifi

void inicializarObjetos(void);
void asignarHorarios(void); //MODULO ESP8266
int horaRegar(void);
int estaSeco(SensorHumedad s);
void constructorSensor(SensorHumedad s, unsigned char porcientoHumedad, unsigned char pin);
void dameHoraActual(void); //MODULO RTC 3231
unsigned char setRtc(unsigned char direccion);
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
short dameHumedadSuelo(char canalLeer);
void configurarAdc(void);
void mostrarMenu(void);
void sistemaPrincipal(unsigned char opcion);
void sistemaRegado(void);
void dameDatosSistema(void);
void dameTemperaturaHumedad(unsigned char* Humedad, unsigned char* Temperatura);
void mostrarDatosSensores(void);
void mostrarDatosSensoresWIFI(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
unsigned char getValue(short numCharacters); //ASCII TO NUMBER FROM UART
void configBluetoothHC_06(void);

void __interrupt() desbordamiento(void) {

    if (INTCONbits.TMR0IF) {

        if (esperandoDatos) {

            tiempoInactividadTrans++;

            if (tiempoInactividadTrans == MAX_TIEMPO_INACTIVIDAD)
                esperaDatoConcluida = 1; //Ya pasaron 10 segundos de espera


        }

        INTCONbits.TMR0IF = 0; //Regresando Bandera a 0 (Interrupcion por Timer 0)
        TMR0 = VALOR_TIMER0; //Overflow cada 10 Segundos
        overflowTimer = 1;

    }

    if (PIR1bits.RCIF) { // interrupción USART PIC por recepción

        byteUart = RCREG;
        datoRecibido = 1;

    }

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void constructorSensor(SensorHumedad s, unsigned char porcientoHumedad, unsigned char pin) {
    s.porcientoHumedad = porcientoHumedad;
    s.pinSensor = pin;
}

void configurarAdc(void) {
    ADCON1 = 0b00000000; //VSS REFERENCIA|TODOS LOS CANALES ANALOGOS
    ADCON2 = 0b10100101; //TIEMPO DE ADQUISICION 8 TAD, JUSTIFICADO A LA DERECHA, FOSC/16
}

int estaSeco(SensorHumedad s) {
    unsigned char temperatura;

    dameTemperaturaHumedad(NULL, &temperatura);

    //Temperatura Ambiente Sensada por el DHT11 y Humedad suelo por el FC -28
    return (s.porcientoHumedad < SENSIBILIDAD_HUMEDAD) && (temperatura < TEMPERATURA_MAX);
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

unsigned char setRtc(unsigned char direccion) {

    unsigned char dato = 0;
    unsigned char seteado = 0;

    dato = getValue(2);

    if (dato != SETEO_DENEGADO) {
        escribe_rtc(direccion, dato);
        seteado = 1;
    }

    return seteado;
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

    UART_printf("\r\n FIJA HORA \r\n"); //comentar

    //// Seccion Horas ///
    UART_printf("\r\n Envie las Horas en formato 24 Ej: 15 \r\n"); //comentar
    ///////Seccion minutos/////

    if (setRtc(0x02)) {
        UART_printf("\r\n Envie los Minutos Ej: 25 \r\n"); //comentar
        if (setRtc(0x01)) {
            UART_printf("\r\n HORA ESTABLECIDA CORRECTAMENTE \r\n"); //comentar
            escribe_rtc(0x00, 0); //SEGUNDOS: 0 SEGUNDOS

        }
    }

}

void asignarHorarios() //ESP8266
{
    unsigned char hora;
    unsigned char Rx;

    UART_printf("\r\n OPCIONES DE REGADO \r\n"); //comentar

    UART_printf("\r\n Ingrese una hora en formato de 24 hrs ej: 15 \r\n"); //comentar
    hora = getValue(2);

    if (hora != SETEO_DENEGADO) {

        UART_printf("\r\n Ingrese 1 para regar || ingrese 0 para no regar: \r\n"); //comentar

        Rx = getValue(1);


        if (Rx != SETEO_DENEGADO) {

            horarios[hora].regar = Rx;

            escribeHorariosMemoria(); //HACER ESTO INDEPENDIENTEMENTE DE SI MANDA DATO
            //POR UART

            UART_printf("\r\n Horario Modificado! \r\n"); //comentar

        }

    }

}

void setTiempoRegar() {

    unsigned char hora;
    unsigned char tiempoRegar;

    UART_printf("\r\n TIEMPO DE REGADO \r\n"); //comentar

    UART_printf("\r\n Ingrese una hora en formato de 24 hrs ej: 15 \r\n"); //comentar
    hora = getValue(2);

    if (hora != SETEO_DENEGADO) {

        UART_printf("\r\n Ingrese los minutos que desee que se riegue en ese horario ej: 15 \r\n"); //comentar
        tiempoRegar = getValue(2);

        if (tiempoRegar != SETEO_DENEGADO) {

            UART_printf("\r\n Minutos de riego establecidos! \r\n"); //comentar

            horarios[hora].tiempoRegar = tiempoRegar;
            minutosRegar = horarios[hora].tiempoRegar;

            escribeHorariosMemoria();

        }
    }

}

short dameHumedadSuelo(char canalLeer) {

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

    PIE1bits.RCIE = 0; //deshabilita interrupción por recepción USART PIC.

    unsigned char Rx = 0, humedadMedida;
    char buffer[TAMANO_CADENA];

    restablecerDatosSensor();

    UART_write(SOLICITUD_DATOS_SENSORES); //Indicar al otro Micro que ya estoy listo para recibir los datos

    UART_printf("\r\nSolicitando Muestreo de sensores\r\n\n"); //comentar

    Rx = UART_read(); //Esperar la confirmacion del otro micro

    if (Rx == ENVIANDO_DATOS_SENSORES) {

        peticionLecturaSensores = 1;

        for (int i = 0; i < TOTAL_SENSORES; i++) {

            sprintf(buffer, "\r\nIngrese el porcentaje de humedad del sensor %d\r\n", i); //comentar
            UART_printf(buffer); //comentar     

            humedadMedida = getValue(3);

            if (humedadMedida != SETEO_DENEGADO)
                sensores[i].porcientoHumedad = humedadMedida;
            else
                sensores[i].porcientoHumedad = 100; //Lectura de Tierra Seca

        }

        UART_printf("\r\nSensores Leidos con Exito!\r\n\n"); //comentar

    } else {
        peticionLecturaSensores = 0;
    }

    PIE1bits.RCIE = 1; //habilita interrupción por recepción USART PIC.

}

void lecturaAnalogaSensores() {

    for (int i = 0; i < TOTAL_SENSORES; i++) { //Sensores humedad suelo FC-28
        sensores[i].porcientoHumedad = map(dameHumedadSuelo(i), 0, BITS_ADC, 100, 0);
        __delay_ms(5);
    }

}

void mostrarMenu(void) {

    UART_printf("\r\n Ingresa una opcion a Realizar: \r\n");
    UART_printf("\r\n 1. Fijar Hora Actual \r\n");
    UART_printf("\r\n 2. Asignar Horarios para regar \r\n");
    UART_printf("\r\n 3. Programar tiempo de riego en un horario \r\n");
    UART_printf("\r\n 4. Dame datos del sistema \r\n");
    UART_printf("\r\n 5. Mostrar valores sensores \r\n");
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

        case 4:
            dameDatosSistema();
            break;

        case 5:
            if (MODO_COMUNICACION)
                mostrarDatosSensoresWIFI();
            else
                mostrarDatosSensores();
            break;


        default:
            UART_printf("\r\n Solo se permiten numeros del 1 al 3 \r\n"); //comentar
            break;
    }

    mostrarMenu(); //comentar
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

        //5 minutos de tolerancia para lectura de sensores
        if ((!MODO_COMUNICACION && (horaRegar() && !minutos)) || (MODO_COMUNICACION
                && (horaRegar() && minutos < TOLERANCIA_LECTURA))) {

            //UART_printf("\n\nRiego Iniciado!\n");

            if (MODO_COMUNICACION) {
                lecturaWifi();
                if (peticionLecturaSensores) {
                    minutosRegar = horarios[hora].tiempoRegar;
                    encenderBombas();
                }
                mostrarMenu();
            } else {
                lecturaAnalogaSensores(); //SENSORES CONECTADOS AL PIC
                minutosRegar = horarios[hora].tiempoRegar;
                encenderBombas();
            }
        }
    }

}

void dameDatosSistema(void) {

    char buffer[TAMANO_CADENA];

    UART_write(INTERRUMPIR_COMANDOS); //Esta Indica que inicio la transmicion de cadenas

    UART_printf("\r\n\nHora | Regar(1 si 0 no) | Minutos de riego \r\n\n");

    for (int i = 0; i < 24; i++) {

        sprintf(buffer, "%d | %d | %d \r\n", horarios[i].hora, horarios[i].regar,
                horarios[i].tiempoRegar);

        UART_printf(buffer);

    }

    UART_write(INTERRUMPIR_COMANDOS); //Esta Indica que acabo la transmicion de cadenas

}

void dameTemperaturaHumedad(unsigned char* Humedad, unsigned char* Temperatura) {

    PIE1bits.RCIE = 0; //deshabilita interrupción por recepción USART PIC.
    T0CONbits.TMR0ON = 0; //PARAR Timer 0

    unsigned char humedad, humedadDecimal;
    unsigned char temperatura, temperaturaDecimal;
    unsigned checkSum = 0;

    DHT11_Start(); /* send start pulse to DHT11 module */

    if (check_response()) {

        /* read 40-bit data from DHT11 module */
        humedad = DHT11_ReadData(); /* read Relative Humidity's integral value */
        humedadDecimal = DHT11_ReadData(); /* read Relative Humidity's decimal value */
        temperatura = DHT11_ReadData(); /* read Temperature's integral value */
        temperaturaDecimal = DHT11_ReadData(); /* read Relative Temperature's decimal value */
        checkSum = DHT11_ReadData(); /* read 8-bit checksum value */

        if (checkSum != (humedad + humedadDecimal + temperatura + temperaturaDecimal))
            UART_printf("Error de lectura DHT11\r\n"); //comentar
        else {
            *Humedad = humedad;
            *Temperatura = temperatura;
        }

    } else
        UART_printf("DHT11 NO RESPONDIO\r\n"); //Comentar


    PIE1bits.RCIE = 1; //habilita interrupción por recepción USART PIC.
    T0CONbits.TMR0ON = 1; //Iniciar Timer 0
}

void mostrarDatosSensores(void) {

    char buffer[TAMANO_CADENA];
    unsigned char temperatura, humedad;

    dameTemperaturaHumedad(&humedad, &temperatura);

    UART_write(INTERRUMPIR_COMANDOS); //Esta Indica que se transmitiran cadenas por
    //UART que no tengan que ver con instrucciones

    sprintf(buffer, "\r\n\nLa Humedad Ambiente es: %d\r\n", humedad);
    UART_printf(buffer);
    sprintf(buffer, "\r\n\nLa Temperatura es: %d C\r\n", temperatura);
    UART_printf(buffer);

    lecturaAnalogaSensores();


    for (int i = 0; i < TOTAL_SENSORES; i++) {

        sprintf(buffer, "\r\n\nPorcentaje Humedad del sensor %d: %d %c\r\n"
                , i, sensores[i].porcientoHumedad, 37);
        UART_printf(buffer);
    }

    UART_write(INTERRUMPIR_COMANDOS); //Esta Indica que acabo la transmicion de cadenas

}

void mostrarDatosSensoresWIFI(void) {


    char buffer[TAMANO_CADENA];
    unsigned char temperatura, humedad;

    lecturaWifi();

    UART_write(INTERRUMPIR_COMANDOS); //Esta Indica que se transmitiran cadenas por
    //UART que no tengan que ver con instrucciones

    if (peticionLecturaSensores) {

        for (int i = 0; i < TOTAL_SENSORES; i++) {

            sprintf(buffer, "\r\n\nPorcentaje Humedad del sensor %d: %d %c\r\n"
                    , i, sensores[i].porcientoHumedad, 37);
            UART_printf(buffer);

        }

    }

    dameTemperaturaHumedad(&humedad, &temperatura);

    sprintf(buffer, "\r\n\nLa Humedad Ambiente es: %d\r\n", humedad);
    UART_printf(buffer);
    sprintf(buffer, "\r\n\nLa Temperatura es: %d C\r\n", temperatura);
    UART_printf(buffer);

    UART_write(INTERRUMPIR_COMANDOS); //Esta Indica que acabo la transmicion de cadenas

}

void configBluetoothHC_06(void) {
    __delay_ms(1000);
    UART_printf("AT+NAMESMARTHOME");
    __delay_ms(1000);
    UART_printf("AT+BAUD4"); //9600 Baudios
    __delay_ms(1000);
    UART_printf("AT+PIN2501");
    __delay_ms(1000);
}

unsigned char getValue(short numCharacters) { //ASCII TO NUMBER FROM UART

    unsigned char Rx = 0;
    unsigned char datoIncorrecto = 0;
    unsigned char dato = 0;

    switch (numCharacters) {

        case 1:
            Rx = UART_read();

            if (Rx >= 48 && Rx <= 57) {
                Rx -= 48;
                dato = Rx;
            } else {
                UART_write(SETEO_DENEGADO); //Notificar al otro micro que no se recibira el dato
                datoIncorrecto = 1;
            }

            break;

        case 2:
            for (int i = 0; i < 2; i++) {//en las direccienes que van desde 0 a 50

                Rx = UART_read();

                if (Rx >= 48 && Rx <= 57) {

                    Rx -= 48;

                    switch (i) {
                        case 0:
                            dato = Rx;
                            dato *= 10;
                            break;

                        case 1:
                            dato += Rx;
                            break;

                        default:
                            break;
                    }

                } else {
                    //UART_printf("\r\n DATO NO RECIBIDO \r\n"); //comentar
                    datoIncorrecto = 1;
                    UART_write(SETEO_DENEGADO); //Notificar al otro micro que no se recibira el dato
                    break;
                }
            }
            break;

        case 3:
            for (int i = 0; i < 3; i++) {//en las direccienes que van desde 0 a 50

                Rx = UART_read();

                if (Rx >= 48 && Rx <= 57) {

                    Rx -= 48;

                    switch (i) {
                        case 0:
                            dato = Rx;
                            dato *= 100;
                            break;

                        case 1:
                            dato += (Rx * 10);
                            break;

                        case 2:
                            dato += Rx;
                            break;

                        default:
                            break;
                    }

                } else {
                    //UART_printf("\r\n DATO NO RECIBIDO \r\n"); //comentar
                    datoIncorrecto = 1;
                    UART_write(SETEO_DENEGADO); //Notificar al otro micro que no se recibira el dato
                    break;
                }
            }
            break;

        default:
            break;
    }



    if (esperandoDatos || datoIncorrecto) {

        UART_printf("\nFALLO EL SETEO\r\n"); //comentar
        return SETEO_DENEGADO;
    } else
        return dato;

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
    //configBluetoothHC_06(); //Configurar el modulo Bluetooth | comentar una vez configurado
    //setRtcDefault(); //Comentar despues de programar el chip por primera vez y volver a programar

    TRISD = 0; //PUERTO D COMO SALIDA
    LATD = 0;

    TMR0 = VALOR_TIMER0; //Overflow cada 10 Segundos

    INTCONbits.TMR0IF = 1; //Inicializando Bandera del TIMER0

    T0CONbits.TMR0ON = 1; //Iniciar Timer 0

    mostrarMenu(); //comentar

    MODO_COMUNICACION = 1; //0 NORMAL  | 1 WIFI


    while (1) {

        if (datoRecibido) {

            datoRecibido = 0; //Bajando bandera
            byteUart -= 48; //Convirtiendo ASCII A ENTERO
            sistemaPrincipal(byteUart);

        }

        if (overflowTimer) {

            esperandoDatos = 0;
            overflowTimer = 0; //Bajando bandera
            sistemaRegado();
        }

    }
    return;
}
