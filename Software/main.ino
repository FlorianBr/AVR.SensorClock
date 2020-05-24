/*
 * Components:
 * - Mega2560
 * - HD44780 LCD 40x4
 * - RTC DS1307
 * 
 * Remarks:
 * - The 40x4 is technically a 80x2, so setCursor and line overflow won't work as expected
 * 
 * 
 * LCD Connections:
 *  8 .. RS
 *  9 .. EN
 *  4 .. DB4
 *  5 .. DB5
 *  6 .. DB6
 *  7 .. DB7
 */

#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>
#include <RTClib.h>
#include <Adafruit_MPL115A2.h>

// System
uint32_t            LastCall = 0;               // Last execution of the loop

// RTC
RTC_DS1307          rtc;
static char         daysOfTheWeek[7][3] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};

// LCD
hd44780_pinIO       lcd(8, 9, 4, 5, 6, 7);

// MPL115A2
Adafruit_MPL115A2   mpl115a2;
#define             ARRAYSIZE                   60          // Size of the array for Temp and pressure
float               ArrayPress[ARRAYSIZE];
float               ArrayTemp[ARRAYSIZE];
uint8_t             ArrayPos = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("[SYS] Build: " __DATE__ " - " __TIME__);
    Serial.println("[SYS] Starting Setup...");

    // Init LCD
    lcd.begin(80, 2);
    lcd.setCursor(0,0);

    // Init RTC
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        lcd.print("Couldn't find RTC!");
        while (1);
    }
    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running!");
        lcd.print("RTC is NOT running!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    rtc.writeSqwPinMode(DS1307_OFF);

    // MPL Sensor
    mpl115a2.begin();
    for (uint8_t i=0;i<ARRAYSIZE;i++) {
        ArrayPress[ARRAYSIZE] = 0.0;
        ArrayTemp[ARRAYSIZE] = 0.0;
        ArrayPos = 0;
    }

    Serial.println("[SYS] Setup complete");
}

void loop() {

    Serial.println("[SYS] Entering Loop!");

    // Power-On Infos
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(__DATE__);
    lcd.setCursor(0,1);
    lcd.print(__TIME__);
    delay(2000);  

    // Print out DS1307s NVRAM
    {
        uint8_t buffer[56];
        rtc.readnvram(&buffer[0],56,0);

        Serial.print("[RTC] NVRAM: ");
        for (uint8_t i=0;i<56;i++) {
            Serial.print(buffer[i],HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    lcd.clear();
    while (1) {
        DateTime now = rtc.now();

        if (now.secondstime() != LastCall) {    // Sync to seconds
            char cBuffer[40];
            float MplPress = 0.0;
            float MplTemp = 0.0;

            float ArraySum;

            LastCall = now.secondstime();

            // Line 1: Time
            lcd.setCursor(0,0);
            snprintf(&cBuffer[0],40,"%02d:%02d:%02d", now.hour(), now.minute(), now.second() );
            lcd.print(cBuffer);

            // Line 2: Date
            lcd.setCursor(0,1);
            snprintf(&cBuffer[0],40,"%02d.%02d.%02d (%s)", now.day(), now.month(), now.year(), daysOfTheWeek[now.dayOfTheWeek()]  );
            lcd.print(cBuffer);

            // Line 3: Temperature & Pressure
            lcd.setCursor(20,0);
            mpl115a2.getPT(&MplPress,&MplTemp);

            // Add values to arrays
            ArrayPress[ArrayPos] = MplPress;    
            ArrayTemp[ArrayPos]  = MplTemp;
            ArrayPos++;
            if (ArrayPos>ARRAYSIZE) ArrayPos=0;

            ArraySum = 0;
            for (uint8_t i=0;i<ARRAYSIZE;i++) ArraySum += ArrayTemp[i];
            ArraySum = ArraySum/ARRAYSIZE;

            // Temperature
            snprintf(&cBuffer[0],40,"%d.%1d", (int)ArraySum, (int)(ArraySum*10)%10);
            lcd.print(cBuffer);
            lcd.print((char)223);
            lcd.print("C");

            ArraySum = 0;
            for (uint8_t i=0;i<ARRAYSIZE;i++) ArraySum += ArrayPress[i];
            ArraySum = ArraySum/ARRAYSIZE;

            snprintf(&cBuffer[0],40," %04d hPa", (int)(10*ArraySum) );
            lcd.print(cBuffer);
        }

        // Serial input parser
        if (Serial.available() > 0) {
            int inByte;

            inByte = Serial.read();

            // say what you got:
            Serial.print("I received: ");
            Serial.println(inByte, DEC);
        }

        delay(25);
    } // while (1)
} // loop
