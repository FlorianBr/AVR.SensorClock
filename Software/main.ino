/*
 * Components:
 * - Mega2560
 * - HD44780 LCD 40x4
 * - RTC DS1307
 * - BME280 (Humidty, Pressure, Temperature) at address 0x76
 * 
 * Remarks:
 * - The 40x4 is technically a 80x2, so setCursor and line overflow won't work as expected
 *   Line 3 is basically the second half of line 1, and line 4 is the second half of line 2
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

/*******************************************************
 * Defines
 *******************************************************/
#define             LCD_CUSTCHARS           8               //< The amount of custom chars for the LCD
#define             BME_RATE                60              //< The BME is updated every 60 secs
#define             LCD_LINEOFFSET          20              //< The offset to get into Line 3 and 4

/*******************************************************
 * Statics
 *******************************************************/
static char         lcd_DoW[7][3] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"}; //< The days for the LCD

static byte CustChars[LCD_CUSTCHARS][8] = {                 // The custom chars to create the big numbers
  { 0x03, 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x18, 0x1C, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x1F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00 },
  { 0x1F, 0x1E, 0x1C, 0x18, 0x00, 0x00, 0x00, 0x00 },
  { 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00 },
  { 0x03, 0x07, 0x0F, 0x1F, 0x00, 0x00, 0x00, 0x00 },
  { 0x0E, 0x0A, 0x0E, 0x00, 0x00, 0x0E, 0x0A, 0x0E }
};

static uint8_t CustNumbers[10][9] = {                       // The segments of the numbers 0..9
  { 0, 5, 1,    2, 32, 2,   3, 5, 4 }, // 0
  { 6, 2, 32,   32, 2, 32,  6, 5, 4 }, // 1
  { 6, 5, 1,    0, 5, 4,    5, 5, 4 }, // 2
  { 6, 5, 1,    32, 5, 2,   3, 5, 4 }, // 3
  { 2, 32, 2,   3, 5, 2,    32,32,5 }, // 4
  { 2, 5, 4,    5, 5, 1,    3, 5, 4 }, // 5
  { 0, 5, 4,    2, 5, 1,    3, 5, 4 }, // 6
  { 3, 5, 1,    32,32,2,    32,32,5 }, // 7
  { 0, 5, 1,    2, 5, 2,    3, 5, 4 }, // 8
  { 0, 5, 1,    3, 5, 2,    3, 5, 4 }  // 9
};

/*******************************************************
 * Local variables
 *******************************************************/
RTC_DS1307          rtc;                                    //< The RTC object
hd44780_pinIO       lcd(8, 9, 4, 5, 6, 7);                  //< The LCD object
Adafruit_BME280     bme;                                    //< The BME object
DateTime            time_now;                               //< The current time

uint32_t            lcd_update = 0;                         //< Timestamp of the last LCD update
uint32_t            bme_update = 0;                         //< Timestamp of the last BME reading
float               bme_temp = 0.0;                         //< The current temperature
float               bme_pres = 0.0;                         //< The current pressure
float               bme_humi = 0.0;                         //< The current humidity


/*******************************************************
 * RTC
 *******************************************************/
/********************************* Setup */
void setup_rtc() {
    // Init RTC
    if (! rtc.begin()) {
        Serial.println(F("[RTC] Couldn't find RTC"));
        lcd.print("Error: RTC!");
        while (1); // loop forever
    }
    if (!rtc.isrunning()) {
        Serial.println(F("[RTC] RTC is NOT running!"));
        // Adjust to build date/time
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    rtc.writeSqwPinMode(DS1307_OFF); // No Sqw-Output
} // setup_rtc
/*******************************************************
 * LCD
 *******************************************************/
/********************************* Print a Custom Number */
void lcd_printdigit(uint8_t Col, uint8_t Digit) {
  if (Digit>9) return;  // boundary check: Valid digits are 0..9
  if (Col>17) return;   // boundary check: only columns 0..17 are valid for this display

  lcd.setCursor(Col,0);
  lcd.write(CustNumbers[Digit][0]);
  lcd.write(CustNumbers[Digit][1]);
  lcd.write(CustNumbers[Digit][2]);
  lcd.setCursor(Col,1);
  lcd.write(CustNumbers[Digit][3]);
  lcd.write(CustNumbers[Digit][4]);
  lcd.write(CustNumbers[Digit][5]);
  lcd.setCursor(Col+LCD_LINEOFFSET,0);
  lcd.write(CustNumbers[Digit][6]);
  lcd.write(CustNumbers[Digit][7]);
  lcd.write(CustNumbers[Digit][8]);

  return;
} // lcd_printdigit
/********************************* Setup */
void setup_lcd() {
  // Init lcd object and create custom chars
  lcd.begin(80, 2);
  for (uint8_t i=0;i<LCD_CUSTCHARS; i++) lcd.createChar(i, CustChars[i]);
} // setup_lcd
/********************************* Worker */
void lcd_worker() {
  static uint8_t    dispmode=0;
  char              cBuffer[20];
  uint8_t           Digit1=0;
  uint8_t           Digit2=0;

  if (time_now.secondstime() == lcd_update) return; // Not yet
  lcd_update = time_now.secondstime();

  // Display mode: switch between temp, pressure, humidity every 5 secs
  if (lcd_update%5==0) {
    dispmode++;
    if (dispmode>2) dispmode=0;
  }

//  lcd.clear(); 
  lcd.setCursor(0,0);

  // Time
  Digit1 = time_now.hour()/10;
  Digit2 = time_now.hour() - (10*Digit1);
  lcd_printdigit(0, Digit1);
  lcd_printdigit(3, Digit2);

  lcd.setCursor(6,1);     lcd.write(7);

  Digit1 = time_now.minute()/10;
  Digit2 = time_now.minute() - (10*Digit1);
  lcd_printdigit( 7, Digit1);
  lcd_printdigit(10, Digit2);

  lcd.setCursor(13,1);     lcd.write(7);

  Digit1 = time_now.second()/10;
  Digit2 = time_now.second() - (10*Digit1);
  lcd_printdigit(14, Digit1);
  lcd_printdigit(17, Digit2);

  // Date
  snprintf(&cBuffer[0],20,"%s,%02d.%02d.%02d",  lcd_DoW[time_now.dayOfTheWeek()], time_now.day(), time_now.month(), time_now.year()-2000  );

  lcd.setCursor(LCD_LINEOFFSET,1); // Start of Line 4
  for (uint8_t i=0;i<(20-strlen(cBuffer));i++) lcd.print(" "); // print spaces to overwrite old stuff and make date right-align
  lcd.print(cBuffer);

  switch (dispmode) {
    case 0: // Temperature
      snprintf(&cBuffer[0],20,"%d.%dC", (uint8_t)bme_temp, (uint16_t)(10*bme_temp)-(((uint16_t)bme_temp)*10) );
      break;
    case 1:
      snprintf(&cBuffer[0],20,"%dhP", (uint16_t)bme_pres );
      break;
    case 2:
      snprintf(&cBuffer[0],20,"%d%%", (uint16_t)bme_humi );
      break;
    default:
      dispmode=0;
      break;
  }
  lcd.setCursor(LCD_LINEOFFSET,1); // Line 4
  lcd.print(cBuffer);
} // lcd_worker
/*******************************************************
 * BME
 *******************************************************/
/********************************* Setup */
void setup_bme() {
    if (!bme.begin(0x76)) {
        Serial.println(F("[BME] Could not find a valid BME280 sensor, check wiring!"));
        while (1);
    }
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    
} // setup_bme
/********************************* Worker */
void bme_worker() {
    if (time_now.secondstime() <= bme_update+BME_RATE) return;   // Update time not yet reached

    bme_update = time_now.secondstime();
    Serial.print(F("[BME] Updateing "));

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
 * Aux-Functions
 *******************************************************/
/********************************* Startup Info */
void start_info() {
    uint8_t buffer[56];
    
    // Show Build-Info
    Serial.println(F("Build: " __DATE__ " " __TIME__ ));
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Build: "));
    lcd.setCursor(0,1);
    lcd.print(F(__DATE__));
    lcd.setCursor(20,0);
    lcd.print(F(__TIME__));
    delay(1000);  

    // Print out DS1307s NVRAM
    rtc.readnvram(&buffer[0],56,0);

    Serial.print(F("[RTC] NVRAM:"));
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
 * Main stuff
 *******************************************************/
/********************************* Setup */
void setup() {
    Serial.begin(115200);
    Serial.println(F("[SYS] Build: " __DATE__ " - " __TIME__));
    Serial.println(F("[SYS] Starting Setup..."));

    setup_lcd();
    setup_rtc();
    setup_bme();

    Serial.println(F("[SYS] Setup complete"));
}
/********************************* Loop */
void loop() {
    Serial.println(F("[SYS] Entering Loop!"));

    start_info();

    while (1) {
        time_now = rtc.now();

        bme_worker();
        lcd_worker();

#if 1
        // Serial input parser
        if (Serial.available() > 0) {
            int   inByte;
            bool  UpdateTime = false;
            uint16_t  Year;
            uint8_t   Month;
            uint8_t   Day;
            uint8_t   Hour;
            uint8_t   Minute;
            uint8_t   Second;

            inByte = Serial.read();

            DateTime rtc_new = rtc.now();

            Year   = rtc_new.year();
            Month  = rtc_new.month();
            Day    = rtc_new.day();
            Hour   = rtc_new.hour();
            Minute = rtc_new.minute();
            Second = rtc_new.second();

            switch (inByte) {
              case 'y': Year--;     UpdateTime=true;  break;
              case 'Y': Year++;     UpdateTime=true;  break;
              case 'o': Month--;    UpdateTime=true;  break;
              case 'O': Month++;    UpdateTime=true;  break;
              case 'd': Day--;      UpdateTime=true;  break;
              case 'D': Day++;      UpdateTime=true;  break;
              case 'h': Hour--;     UpdateTime=true;  break;
              case 'H': Hour++;     UpdateTime=true;  break;
              case 'm': Minute--;   UpdateTime=true;  break;
              case 'M': Minute++;   UpdateTime=true;  break;
              case 's': Second--;   UpdateTime=true;  break;
              case 'S': Second++;   UpdateTime=true;  break;
            } // switch

            if (UpdateTime) {
              DateTime NewTime(Year, Month, Day, Hour, Minute, Second);
              rtc.adjust(NewTime);
            }
        }
#endif
        delay(25);
    } // while (1)
} // loop
