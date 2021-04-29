/*
 * gps controlled oszillator für oe4xlc in allhau
 * oe6rke, 2021-04
 * 
 * oszi0 2.5 MHz  (feedback)
 * oszi1 125 MHz  (HF Mixer)
 * oszi2 28.8 MHz (RTL Dongle Oszi)
 * 
 *  si5351 + nano + txco 25.00 (ppm2.5) + GPS NEO7M 
 *  
 * https://github.com/etherkit/Si5351Arduino
 * Modul hier https://www.sv1afn.com/si5351a.html,
 * http://ak2b.blogspot.com/2015/01/installing-libraries-and-running-code.html
 * 
 *  Based on the projects: 
 *  W3PM (http://www.knology.net/~gmarcus/)
 *  &
 *  SQ1GU (http://sq1gu.tobis.com.pl/pl/syntezery-dds/44-generator-si5351a)
 */

#include <TinyGPS++.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Wire.h>
#include <si5351.h>
#include <SoftwareSerial.h>



// The TinyGPS++ object
TinyGPSPlus gps;

// The Si5351 object
Si5351 si5351;

//pins
#define LED 12
#define ppsPin                   2  //D2
#define przycisk                 A2

#define CHA 3
#define CHB 7
volatile bool fired;
volatile bool up;

//XTAL freq
unsigned long XtalFreq = 100000000;
unsigned long XtalFreq_old = 100000000;

//freqs
unsigned long Freq1 = 12500000; // 125 MHz
unsigned long Freq2 = 2880000;  // 28.8 MHz

long stab;
long correction = 0;
byte stab_count = 44;
unsigned long mult = 0; 
int second = 0, minute = 0, hour = 0;
int day = 0, month = 0, year = 0;
int zone = 1;
unsigned int tcount = 0;
unsigned int tcount2 = 0;
int validGPSflag = false;
char c;
boolean newdata = false;
boolean GPSstatus = true;
boolean fixed = false;

byte new_freq = 1; //switch für neue qrg
unsigned long freq_step = 1000;

byte encoderOLD, menu = 0, band = 1, f_step = 1;
boolean time_enable = true;
unsigned long pps_correct;
byte pps_valid = 1;
float stab_float = 1000;

SoftwareSerial gpsSerial(11, 10); // RX=D11, TX=D10. on RX comes GPS Serial in


//*************************************************************************************
//                                    SETUP
//*************************************************************************************
void setup()
{
  bool i2c_found;
  Serial.begin(9600); 
  gpsSerial.begin(9600);
  
  Serial.println("GPSDO RTL coded by OE6RKE Version 2021-04-28");
                  
  //led status D12
  pinMode(LED, OUTPUT);

  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  pinMode(ppsPin, INPUT);                        // Inititalize GPS 1pps input
  digitalWrite(ppsPin, HIGH);

  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25000000, 10); //init with 25 mhz and offset
  
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  //si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA); //5db Output für clock0

  // Set CLK0 to output 2,5MHz
  si5351.set_freq(250000000ULL, SI5351_CLK0); //CLK0 = 2.5MHz

  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  // CHOSE YOUR FREQUENCY HERE
  //si5351.set_freq(12500000000ULL, SI5351_CLK1);
  si5351.set_freq(2880000000ULL, SI5351_CLK1);
   
  si5351.update_status();

  //version info
  digitalWrite(LED, HIGH);
  delay(4000);
  digitalWrite(LED, LOW);

  GPSproces(6000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    delay(5000);
    GPSstatus = false;
  }
  
  if (GPSstatus == true) {
    time_on_oled();
    sat_on_oled();
    do {
      GPSproces(1000);
    } while (gps.satellites.value() == 0);

    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();

    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();

    time_on_oled();
    sat_on_oled();
    attachInterrupt(0, PPSinterrupt, RISING); //attach 2 interrupt
    TCCR1B = 0;
    tcount = 0;
    mult = 0;
    validGPSflag = 1;
  }
  //display on
  freq_on_oled();
  sat_on_oled();
  time_on_oled();
  date_on_oled();

  Serial.println("GPSDO RTL init phase done");
}

//***************************************************************************************
//                                         LOOP
//***************************************************************************************
void loop()
{

  if (tcount2 != tcount) {
    tcount2 = tcount;
    pps_correct = millis();
  }
  if (tcount < 4 ) {
    GPSproces(0);
  }
  if (gps.time.isUpdated()) {
    hour = gps.time.hour() + zone;
    minute = gps.time.minute();
    second = gps.time.second();
  }
  
  if (gps.date.isUpdated()) {
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();
  }
  
  if (gps.satellites.isUpdated() && menu == 0) {
    sat_on_oled();
  }

  //do it always
  correct_si5351a();
  new_freq = 0;
  if (abs(stab_float)<1)  {
    digitalWrite(LED, HIGH);
    Serial.println("GPSDO locked");
    fixed = true;
  }
  if (abs(stab_float)>1) {
    digitalWrite(LED, LOW);
    Serial.println("GPSDO not locked");
    fixed = false;
  }

  if (millis() > pps_correct + 1200) {
    pps_valid = 0;
    pps_correct = millis();
    time_enable = false;
  }

   //wait not to overflow stuff
   delay(5000);
}


//**************************************************************************************
//                       INTERRUPT  1PPS
//**************************************************************************************
void PPSinterrupt()
{
  
  tcount++;
  stab_count--;
  if (tcount == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
  }
  if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;  
    if (pps_valid == 1) {
      XtalFreq_old = XtalFreq;      
      XtalFreq = mult * 0x10000 + TCNT1;         //Calculate correction factor, only if sat is present      
      //Serial.println(XtalFreq);
      new_freq = 1;
    }
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
    pps_valid = 1;
   
    stab_count = 44;
    stab_on_oled();
    //stab_on_serial();
  }
  if (validGPSflag == 1)                      //Start the UTC timekeeping process
  {
    second++;
    if (second == 60)                            //Set time using GPS NMEA data
    {
      minute++ ;
      second = 0 ;
    }
    if (minute == 60)
    {
      hour++;
      minute = 0 ;
    }
    if (hour == 24) hour = 0 ;
    if (time_enable) time_on_oled();
  }
}
//*******************************************************************************
// Timer 1 overflow intrrupt vector.
//*******************************************************************************
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}



//********************************************************************************
//                                STAB on oled stabilnośc częstotliwości
//********************************************************************************
void stab_on_oled() {
  long pomocna;
  time_enable = false;
  stab = XtalFreq - 100000000;
  stab = stab * 10 ;
  if (stab > 100 || stab < -100) {
    correction = correction + stab;
  }
  else if (stab > 20 || stab < -20) {
    correction = correction + stab / 2;
  }
  else correction = correction + stab / 4;
  pomocna = (10000 / (Freq1 / 1000000));
  stab = stab * 100;
  stab = stab / pomocna;
  stab_float = float(stab);
  stab_float = stab_float / 10;
  
  Serial.print("Freq. correction: ");
  Serial.print(stab_float);
  Serial.println(" Hz");

  //print location
  Serial.print("lat: ");
  Serial.print(gps.location.rawLat().negative ? "-" : "+");
  Serial.println(gps.location.lat(), 8);
  
  Serial.print("lon: ");
  Serial.print(gps.location.rawLng().negative ? "-" : "+");  
  Serial.println(gps.location.lng(), 8);
}



//********************************************************************************
//                                TIME on oled
//********************************************************************************
void time_on_oled()
{
  char sz[32];
  sprintf(sz, "%02d:%02d:%02d ", hour, minute, second);
}

//********************************************************************************
//                                DATE on oled
//********************************************************************************
void date_on_oled()
{
  char da[32];
  sprintf(da, "%02d/%02d/%02d ", day, month, year);
}

//********************************************************************************
//                                SAT nr. on oled
//********************************************************************************
void sat_on_oled()
{
  time_enable = false;
  time_enable = true;
}

//*********************************************************************************
//                             Freq on oled
//*********************************************************************************
void freq_on_oled() {
  time_enable = false;
}
//********************************************************************
//             NEW frequency  --- NOT USED, because fixed
//********************************************************************
void update_si5351a()
{
  si5351.set_freq(Freq1 * SI5351_FREQ_MULT, SI5351_CLK1);
}
//********************************************************************
//             NEW frequency correction
//********************************************************************
void correct_si5351a()
{
  si5351.set_correction(correction, SI5351_PLL_INPUT_XO);
}
//*********************************************************************
//                    Odczyt danych z GPS
//**********************************************************************
static void GPSproces(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}
//*********************************************************************
