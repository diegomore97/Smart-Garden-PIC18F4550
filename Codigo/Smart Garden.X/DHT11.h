#define DHT11_pin_OUTPUT LATDbits.LATD7
#define DHT11_pin_INPUT PORTDbits.RD7
#define DIR_DHT11 TRISDbits.RD7

void DHT11_Start(void);
unsigned char check_response();
unsigned char DHT11_ReadData();

unsigned char DHT11_ReadData() {
    char i, data = 0;
    for (i = 0; i < 8; i++) {
        while (!(DHT11_pin_INPUT & 1)); /* wait till 0 pulse, this is start of data pulse */
        __delay_us(30);
        if (DHT11_pin_INPUT & 1) /* check whether data is 1 or 0 */
            data = ((data << 1) | 1);
        else
            data = (data << 1);
        while (DHT11_pin_INPUT & 1);
    }
    return data;
}

void DHT11_Start(void) {
    DIR_DHT11 = 0; //PIN COMO SALIDA

    DHT11_pin_OUTPUT = 0; /* send low pulse of min. 18 ms width */

    __delay_ms(20);

    DHT11_pin_OUTPUT = 1; /* pull data bus high */

    __delay_us(30);

    DIR_DHT11 = 1; //PIN COMO ENTRADA

}

unsigned char check_response() {

    unsigned char respuesta = 0;
    __delay_us(40);
    if (!DHT11_pin_INPUT) { // read and test if connection pin is low
        __delay_us(80);
        if (DHT11_pin_INPUT) { // read and test if connection pin is high
            __delay_us(50);
            respuesta = 1;
        }
    }

    return respuesta;
}

