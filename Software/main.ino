/*
 * Components:
 * - Mega2560
 * - HD44780 LCD 40x4
 * - RTC DS1307
 * - BME280 (Humidty, Pressure, Temperature) at address 0x76
 * 
 * Remarks:
 * - The 40x4 is technically a 80x2, so setCursor and line overflow won't work as expected
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
#include <Adafruit_BME280.h>

static char         daysOfTheWeek[7][3] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};

// RTC
RTC_DS1307          rtc;
DateTime            rtc_now;
// LCD
// the 8 arrays that form each segment of the custom numbers
byte bar1[8] = { B11100, B11110, B11110, B11110, B11110, B11110, B11110, B11100 };
byte bar2[8] = { B00111, B01111, B01111, B01111, B01111, B01111, B01111, B00111 };
byte bar3[8] = { B11111, B11111, B00000, B00000, B00000, B00000, B11111, B11111 };
byte bar4[8] = { B11110, B11100, B00000, B00000, B00000, B00000, B11000, B11100 };
byte bar5[8] = { B01111, B00111, B00000, B00000, B00000, B00000, B00011, B00111 };
byte bar6[8] = { B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111 };
byte bar7[8] = { B00000, B00000, B00000, B00000, B00000, B00000, B00111, B01111 };
byte bar8[8] = { B11111, B11111, B00000, B00000, B00000, B00000, B00000, B00000 };

hd44780_pinIO       lcd(8, 9, 4, 5, 6, 7);
uint32_t            lcd_update = 0;               // Last LCD update
//BME
Adafruit_BME280     bme;
uint32_t            bme_update = 0;               // Last reading
float               bme_temp = 0.0;
float               bme_pres = 0.0;
float               bme_humi = 0.0;
#define             BME_RATE            60          // BME is updated every 60 secs


/*******************************************************
 * RTC
 *******************************************************/
void setup_rtc() {
    // Init RTC
    if (! rtc.begin()) {
        Serial.println("Couldn't find RTC");
        lcd.print("Error: RTC!");
        while (1); // loop forever
    }
    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    rtc.writeSqwPinMode(DS1307_OFF); // No Sqw-Output
} // setup_rtc

/*******************************************************
 * LCD
 *******************************************************/
void custom0(int col)
{ // uses segments to build the number 0
  lcd.setCursor(col, 0); 
  lcd.write(2);  
  lcd.write(8); 
  lcd.write(1);
  lcd.setCursor(col, 1); 
  lcd.write(2);  
  lcd.write(6);  
  lcd.write(1);
}

void custom1(int col)
{
  lcd.setCursor(col,0);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
  lcd.setCursor(col,1);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
}

void custom2(int col)
{
  lcd.setCursor(col,0);
  lcd.write(5);
  lcd.write(3);
  lcd.write(1);
  lcd.setCursor(col, 1);
  lcd.write(2);
  lcd.write(6);
  lcd.write(6);
}

void custom3(int col)
{
  lcd.setCursor(col,0);
  lcd.write(5);
  lcd.write(3);
  lcd.write(1);
  lcd.setCursor(col, 1);
  lcd.write(7);
  lcd.write(6);
  lcd.write(1); 
}

void custom4(int col)
{
  lcd.setCursor(col,0);
  lcd.write(2);
  lcd.write(6);
  lcd.write(1);
  lcd.setCursor(col, 1);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
}

void custom5(int col)
{
  lcd.setCursor(col,0);
  lcd.write(2);
  lcd.write(3);
  lcd.write(4);
  lcd.setCursor(col, 1);
  lcd.write(7);
  lcd.write(6);
  lcd.write(1);
}

void custom6(int col)
{
  lcd.setCursor(col,0);
  lcd.write(2);
  lcd.write(3);
  lcd.write(4);
  lcd.setCursor(col, 1);
  lcd.write(2);
  lcd.write(6);
  lcd.write(1);
}

void custom7(int col)
{
  lcd.setCursor(col,0);
  lcd.write(2);
  lcd.write(8);
  lcd.write(1);
  lcd.setCursor(col, 1);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
}

void custom8(int col)
{
  lcd.setCursor(col, 0); 
  lcd.write(2);  
  lcd.write(3); 
  lcd.write(1);
  lcd.setCursor(col, 1); 
  lcd.write(2);  
  lcd.write(6);  
  lcd.write(1);
}

void custom9(int col)
{
  lcd.setCursor(col, 0); 
  lcd.write(2);  
  lcd.write(3); 
  lcd.write(1);
  lcd.setCursor(col, 1); 
  lcd.write(7);  
  lcd.write(6);  
  lcd.write(1);
}
void setup_lcd() {
    lcd.begin(80, 2);

	lcd.createChar(1, bar1);
    lcd.createChar(2, bar2);
	lcd.createChar(3, bar3);
	lcd.createChar(4, bar4);
	lcd.createChar(5, bar5);
	lcd.createChar(6, bar6);
	lcd.createChar(7, bar7);
	lcd.createChar(8, bar8);

    lcd.setCursor(0,0);

} // setup_lcd
void lcd_printdigit(uint8_t Val, uint8_t Col) {
    switch (Val) {
        case 0: custom0(Col); break;
        case 1: custom1(Col); break;
        case 2: custom2(Col); break;
        case 3: custom3(Col); break;
        case 4: custom4(Col); break;
        case 5: custom5(Col); break;
        case 6: custom6(Col); break;
        case 7: custom7(Col); break;
        case 8: custom8(Col); break;
        case 9: custom9(Col); break;

    }
} // lcd_printdigit

void lcd_worker() {
    char cBuffer[40];
    uint8_t Digit1=0;
    uint8_t Digit2=0;


    if (rtc_now.secondstime() == lcd_update) return; // Not yet
    lcd_update = rtc_now.secondstime();

    lcd.clear();

    // Time
    Digit1 = rtc_now.hour()/10;
    Digit2 = rtc_now.hour() - (10*Digit1);
    lcd_printdigit(Digit1, 0);
    lcd_printdigit(Digit2, 3);

    lcd.setCursor(6,0);     lcd.print("o");
    lcd.setCursor(6,1);     lcd.print("o");

    Digit1 = rtc_now.minute()/10;
    Digit2 = rtc_now.minute() - (10*Digit1);
    lcd_printdigit(Digit1, 7);
    lcd_printdigit(Digit2, 10);

    lcd.setCursor(13,0);     lcd.print("o");
    lcd.setCursor(13,1);     lcd.print("o");

    Digit1 = rtc_now.second()/10;
    Digit2 = rtc_now.second() - (10*Digit1);
    lcd_printdigit(Digit1, 14);
    lcd_printdigit(Digit2, 17);

    // Date
    snprintf(&cBuffer[0],40,"%s, %02d.%02d.%02d",  daysOfTheWeek[rtc_now.dayOfTheWeek()], rtc_now.day(), rtc_now.month(), rtc_now.year()  );
    lcd.setCursor(26,0);
    lcd.print(cBuffer);

    // Temperature
    snprintf(&cBuffer[0],40,"%d.%dC %dhPa %d%%", (uint8_t)bme_temp, (uint8_t)(10*bme_temp)-(((uint8_t)bme_temp)*10), (uint16_t)bme_pres, (uint8_t)bme_humi );

    lcd.setCursor(20,1);
    if (strlen(cBuffer) < 20) for (uint8_t i=0;i<20-strlen(cBuffer);i++) lcd.print(" "); // right-align
    lcd.print(cBuffer);


} // lcd_worker
/*******************************************************
 * BME
 *******************************************************/
void setup_bme() {
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    
} // setup_bme
void bme_worker() {
    if (rtc_now.secondstime() <= bme_update+BME_RATE) return;   // Update time not yet reached

    bme_update = rtc_now.secondstime();
    Serial.print("[BME] Updateing ");

    bme.takeForcedMeasurement();
    bme_temp = bme.readTemperature();
    bme_pres = bme.readPressure()/100.0F;
    bme_humi = bme.readHumidity();

    Serial.print(bme_temp, DEC);
    Serial.print(" dC ");
    Serial.print(bme_pres, DEC);
    Serial.print(" PA ");
    Serial.print(bme_humi, DEC);
    Serial.print(" %");

    Serial.println();

} // bme_worker
/*******************************************************
 * Setup
 *******************************************************/
void setup() {
    Serial.begin(115200);
    Serial.println("[SYS] Build: " __DATE__ " - " __TIME__);
    Serial.println("[SYS] Starting Setup...");

    setup_lcd();
    setup_rtc();
    setup_bme();

    Serial.println("[SYS] Setup complete");
}
/*******************************************************
 * Aux-Functions
 *******************************************************/
void start_info() {
    uint8_t buffer[56];
    
    // Show Build-Info
    Serial.println("Build: " __DATE__ " " __TIME__ );
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Build: ");
    lcd.setCursor(0,1);
    lcd.print(__DATE__);
    lcd.setCursor(20,0);
    lcd.print(__TIME__);
    delay(1000);  

    // Print out DS1307s NVRAM
    rtc.readnvram(&buffer[0],56,0);

    Serial.print("[RTC] NVRAM:");
    for (uint8_t i=0;i<56;i++) {
        if (i%8==0) Serial.println();
        Serial.print(" ");
        if (buffer[i]<0x0A) Serial.print("0");
        Serial.print(buffer[i],HEX);
    }
    Serial.println();
    lcd.clear();
} // start_info
/*******************************************************
 * Loop
 *******************************************************/
void loop() {
    Serial.println("[SYS] Entering Loop!");

    start_info();

    while (1) {
        rtc_now = rtc.now();

        bme_worker();
        lcd_worker();

#if 0
        // Serial input parser
        if (Serial.available() > 0) {
            int inByte;

            inByte = Serial.read();

            // say what you got:
            Serial.print("I received: ");
            Serial.println(inByte, DEC);
        }
#endif
        delay(25);
    } // while (1)
} // loop
