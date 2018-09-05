#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_10DOF.h>

// Assign a unique ID to the sensors
Adafruit_10DOF dof = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(18001);

// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;

/******************************************************************************
 * UTIL
 ******************************************************************************/

void initSensors() {
    if (!accel.begin()) {
        // TODO flash red LED
        while (1);
    }

    if (!mag.begin()) {
        // TODO flash red LED
        while (1);
    }
}

/**
 * Since the orientation values are from -180 - 180 and only need 2 decimals
 * of precision, might as well compress the 32 bit float into a 16 bit number
 * so that the amount data sent over serial is reduced by ~half.
 * 
 * int      frac      sign
 * 0-255	0-128	  0-1   min/max range 
 * 0-180    0-99      0-1   expected range of values
 * 11111111	1111111	  1
 * 8		7		  1
 */
uint16_t compressFloat(float a) {
    double integerDec = 0.0f;
    double fractionDec = 0.0f;
    uint8_t integer = 0;
    uint8_t fraction = 0;
    uint16_t result = 0;

    fractionDec = modf(a, &integerDec);
    integer = (uint8_t)(integerDec);
    fraction = (uint8_t)(fractionDec * 100);

    // store 8 bit integer part on "left" half
    result |= (uint16_t)(integer << 8);

    // store 7 bit fractional part on "right" half
    result |= (uint16_t)((fraction & 0x7F) << 1); 

    if (a < 0) {
        result |= 0x01;
    } // else first bit is going to be 0 anyway (positive)

    return result;
}

/**
 * Build message to be sent over serial
 * 
 * args[4]:
 * [roll, pitch, heading, yAccel]
 * 
 * buffer[9]:
 * 0        1		2		3		4		5		6	    7       8
 * START	ROLL	ROLL	PTCH	PTCH	HDNG	HDNG    YACC    YACC
 * 0xFF 	
 */
void buildMsg(float *args, uint8_t *buffer) {
    buffer[0] = 0xFF;
    for (int i = 1, j = 0; i < 9; i += 2, j++) {
        uint16_t compressed = compressFloat(args[j]);
        buffer[i] = (uint8_t)((compressed & 0xFF00) >> 8);
        buffer[i + 1] = (uint8_t)(compressed & 0xFF);
    }
}

/**
 * Print serial message built by buildMsg()
 */
void printMsg(uint8_t *buffer) {
    for (int i = 0; i < 9; i++) {
        Serial.print(buffer[i], HEX);
    }
    Serial.print('\n');
}

/******************************************************************************
 * MAIN
 ******************************************************************************/

void setup() {
    Serial.begin(115200);
    Serial.println(F("yeet"));
    Serial.println("");

    initSensors();
}

void loop() {
    sensors_event_t acc_event;
    sensors_event_t mag_event;
    sensors_vec_t orientation;
    float msgArgs[4] = {0};
    uint8_t serialMsg[9] = {0};

    accel.getEvent(&acc_event);
    mag.getEvent(&mag_event);

    if (dof.fusionGetOrientation(&acc_event, &mag_event, &orientation)) {
        // roll pitch heading y-accel
        msgArgs[0] = orientation.roll;
        msgArgs[1] = orientation.pitch;
        msgArgs[2] = orientation.heading;
        msgArgs[3] = acc_event.acceleration.y;

        buildMsg(msgArgs, serialMsg);

        printMsg(serialMsg);
    }

    delay(32);
}
