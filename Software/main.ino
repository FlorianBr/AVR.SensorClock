/*
 * Components:
 * - Mega2560
 * - HD44780 LCD 40x4
 * - RTC DS1307
 * - BME280 (Humidty, Pressure, Temperature) at address 0x76
 * - Dust-Sensor
 * 
 * Connections

 *   2 .. ENC_A
 *   3 .. ENC_B
 *   4 .. LCD_DB4
 *   5 .. LCD_DB5
 *   6 .. LCD_DB6
 *   7 .. LCD_DB7
 *   8 .. LCD_RS
 *   9 .. LCD_EN
 *
 *  11 .. DUST_LED
 * 
 *  18 .. ENC_Key
 * 
 *  20 .. SDA
 *  21 .. SCL
 * 
 *  A5 .. DUST_INPUT
 */

#include <hd44780.h>
#include <hd44780ioClass/hd44780_pinIO.h>
#include <RTClib.h>
#include <Adafruit_BME280.h>

/*******************************************************
 * Defines
 *******************************************************/
#define             LCD_CUSTCHARS           8               //< The amount of custom chars for the LCD
#define             BME_RATE                60              //< Update rate of the BME
#define             DUST_RATE               15              //< Update rate of the dust sensor
#define             LCD_RS                  8               //< GPIO for the LCD RS
#define             LCD_EN                  9               //< GPIO for the LCD EN
#define             LCD_D4                  4               //< GPIO for the LCD Data 4
#define             LCD_D5                  5               //< GPIO for the LCD Data 5
#define             LCD_D6                  6               //< GPIO for the LCD Data 6
#define             LCD_D7                  7               //< GPIO for the LCD Data 7
#define             ENC_A                   2               //< GPIO for the encoder A signal (Int required!)
#define             ENC_B                   10              //< GPIO for the encoder B signal
#define             ENC_KEY                 3               //< GPIO for the encoder key (Int required!)
#define             DUST_INPUT              A5              //< AnaIn for the dust sensor
#define             DUST_LED                11              //< LED-Out for the dust sensor
#define             DUST_COV_RATIO          0.166           //< Dust-Sensor: ug/mmm / mv
#define             DUST_NO_DUST            600             //< "No Dust" Voltage [mv]
#define             DUST_VOLTAGE            5000            //< Voltage of the dust sensor

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
hd44780_pinIO       lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); //< The LCD object
Adafruit_BME280     bme;                                    //< The BME object
DateTime            time_now;                               //< The current time
uint32_t            lcd_update = 0;                         //< Timestamp of the last LCD update
uint32_t            bme_update = 0;                         //< Timestamp of the last BME reading
uint32_t            dust_update = 0;                        //< Timestamp of the last Dust-Sensor reading
float               bme_temp = 0.0;                         //< The current temperature
float               bme_pres = 0.0;                         //< The current pressure
float               bme_humi = 0.0;                         //< The current humidity
volatile int32_t    enc_count = 0;                          //< The counter for the encoder
volatile bool       enc_keypressed = false;                 //< The encoder-key was pressed
float               dust_density = 0.0;                     //< The measured density

/*******************************************************
 * RTC
 *******************************************************/
/********************************* Setup */
void rtc_setup() {
    // Init RTC
    if (! rtc.begin()) {
        Serial.println(F("[RTC] Couldn't find RTC"));
        lcd.print("Error: RTC!");
        while (1); // loop forever
    }
    if (!rtc.isrunning()) {
        Serial.println(F("[RTC] RTC is NOT running!"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Adjust to build date/time
    }
    rtc.writeSqwPinMode(DS1307_OFF); // No Sqw-Output
} // rtc_setup
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
  lcd.setCursor(Col,2);
  lcd.write(CustNumbers[Digit][6]);
  lcd.write(CustNumbers[Digit][7]);
  lcd.write(CustNumbers[Digit][8]);

  return;
} // lcd_printdigit
/********************************* Setup */
void lcd_setup() {
  // Init lcd object and create custom chars
  lcd.begin(20, 4);
  for (uint8_t i=0;i<LCD_CUSTCHARS; i++) lcd.createChar(i, CustChars[i]);
} // lcd_setup
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
    if (dispmode>3) dispmode=0;
  }

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

  lcd.setCursor(0,3); // Start of Line 4
  for (uint8_t i=0;i<(20-strlen(cBuffer));i++) lcd.print(" "); // print spaces to overwrite old stuff and make date right-align
  lcd.print(cBuffer);

  switch (dispmode) {
    case 0: // Temperature
      snprintf(&cBuffer[0],20,"%d.%dC", (uint8_t)bme_temp, (uint16_t)(10*bme_temp)-(((uint16_t)bme_temp)*10) );
      break;
    case 1: // Pressure
      snprintf(&cBuffer[0],20,"%dhP", (uint16_t)bme_pres );
      break;
    case 2: // Humidity
      snprintf(&cBuffer[0],20,"%d%%", (uint16_t)bme_humi );
      break;
    case 3: // Dust density
      snprintf(&cBuffer[0],20,"%d.%d", (uint8_t)dust_density, (uint16_t)(10*dust_density)-(((uint16_t)dust_density)*10) );
      break;
    default:
      dispmode=0;
      break;
  }
  lcd.setCursor(0,3); // Line 4
  lcd.print(cBuffer);
} // lcd_worker
/*******************************************************
 * BME
 *******************************************************/
/********************************* Setup */
void bme_setup() {
    if (!bme.begin(0x76)) {
        Serial.println(F("[BME] Could not find a valid BME280 sensor, check wiring!"));
        while (1);
    }
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    
} // bme_setup
/********************************* Worker */
void bme_worker() {
  if (time_now.secondstime() <= bme_update+BME_RATE) return;   // Update time not yet reached
  bme_update = time_now.secondstime();

  Serial.print(F("[BME] Getting Measurements:  "));

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
 * Dust-Sensor
 *******************************************************/
/********************************* Setup */
void dust_setup() {
  pinMode(DUST_LED, OUTPUT);
} // dust_setup
/********************************* Worker */
void dust_worker() {
  int AdcValue;
  float density;
  float voltage;

  if (time_now.secondstime() <= dust_update+DUST_RATE) return;   // Update time not yet reached
  dust_update = time_now.secondstime();

  Serial.print(F("[DST] Getting Measurements:  "));


  digitalWrite(DUST_LED, HIGH);             // Sensor LED ON
  delayMicroseconds(280);                   // Wait 0.28ms for sampling
  AdcValue = analogRead(DUST_INPUT);        // Get Sample
  digitalWrite(DUST_LED, LOW);              // Sensor LED OFF

  // Calculate measured voltage
  voltage = (DUST_VOLTAGE / 1023.0) * AdcValue;

  // Calculate density
  if(voltage >= DUST_NO_DUST) {
    density = (voltage-DUST_NO_DUST) * DUST_COV_RATIO;
  }
  else
    density = 0;


  Serial.print(F("Raw: "));
  Serial.print(AdcValue, DEC);

  Serial.print(F(" Voltage: "));
  Serial.print(voltage, DEC);

  Serial.print(F(" Density: "));
  Serial.print(density, DEC);

  Serial.println(" ug/m3");

  dust_density = density;
} // dust_worker
/*******************************************************
 * Encoder
 *******************************************************/
/********************************* Setup */
void enc_setup() {
  pinMode(ENC_KEY, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_KEY), enc_intkey, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_A),   enc_inta,   CHANGE);
} // enc_setup
/********************************* Int: Key */
void enc_intkey() {
  enc_keypressed=true;
} // enc_intkey
/********************************* Int: A-Signal */
void enc_inta() {
  static int OldA = HIGH;

  // Falling Edge
  if (OldA==HIGH &&  digitalRead(ENC_A) == LOW ) {
    if (digitalRead(ENC_B)==HIGH) enc_count++;
    if (digitalRead(ENC_B)==LOW) enc_count--;
  }
  // Rising Edge
  if (OldA==LOW &&  digitalRead(ENC_A) == HIGH ) {
    if (digitalRead(ENC_B)==LOW) enc_count++;
    if (digitalRead(ENC_B)==HIGH) enc_count--;
  }

  OldA = digitalRead(ENC_A);

  return;
} // enc_inta
/********************************* Worker */
void enc_worker() {
  static int32_t OldCount = 0;

  // Handle keypresses
  if (enc_keypressed) {
    Serial.println(F("[ENC] Key was pressed!"));
    enc_keypressed=false;
  }

  // Handle Encoder
  if (OldCount!=enc_count) {
    Serial.print(F("[ENC] Rotation: "));
    Serial.println( (OldCount-enc_count), DEC);

    OldCount = enc_count;
  }
  return;
} // enc_worker
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
  lcd.setCursor(0,2);
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

    lcd_setup();
    rtc_setup();
    bme_setup();
    enc_setup();
    dust_setup();

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
        enc_worker();
        dust_worker();

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
