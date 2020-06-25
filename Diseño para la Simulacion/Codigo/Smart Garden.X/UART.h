void UART_init(long BAUD);
unsigned char EUSART1_Read(void);
void UART_write(char dato);
void UART_printf(char* cadena);

unsigned char tiempoInactividadTrans = 0;
unsigned char byteUart = 0;
unsigned char esperaDatoConcluida = 0;
unsigned char esperandoDatos = 0;
int VALOR_TIMER0UART = 26473;

void UART_init(long BAUD) {
    long frecuenciaCristal = _XTAL_FREQ;
    TRISCbits.TRISC6 = 0; //TX OUTPUT
    TRISCbits.TRISC7 = 1; //RX INPUT

    //Baudios
    SPBRG = (frecuenciaCristal / 16 / BAUD) - 1;

    //Configuración 
    TXSTAbits.BRGH = 1; //High Speed
    TXSTAbits.SYNC = 0; //Asincrono
    RCSTAbits.SPEN = 1; //Habilitar Tx y Rx

    //Transmisión
    TXSTAbits.TX9 = 0; //8 bits
    TXSTAbits.TXEN = 1; //Activamos transmisión

    //Recepción
    RCSTAbits.RC9 = 0; //8 bits
    RCSTAbits.CREN = 1; //Activamos recepción
}

unsigned char UART_read(void) {

    TMR0 = VALOR_TIMER0UART; //Overflow cada 10 Segundos
    esperaDatoConcluida = 0; //Esperar solo 10 segundos a que el usuario ingrese datos
    esperandoDatos = 1; //Bandera que indica que estamos esperando dato por UART
    byteUart = 0; //Borrar Byte anterior
    tiempoInactividadTrans = 0; //Tiempo que ha transcurrido esperando un dato en decenas de segundo

    while (!PIR1bits.RCIF && !esperaDatoConcluida);

    if (!esperaDatoConcluida) {
        byteUart = RCREG;
        esperandoDatos = 0;
    }

    return byteUart;
}

void UART_write(char dato) {
    TXREG = dato;
    while (!TXSTAbits.TRMT);
}

void UART_printf(char* cadena) {
    while (*cadena) {
        UART_write(*cadena++);
    }
}