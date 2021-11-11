#include <PinChangeInt.h>
#include <Wire.h>
#include <Adafruit_MCP23X08.h>
#include <arduinoFFT.h>
#include <SPI.h>
#include <RGBmatrixPanel.h>

//init audio amp
// 0x4B is the default i2c address
#define MAX9744_I2CADDR 0x4B

//#define ROT_SW A1         // rotary puhbutton
// Global variables that can be changed in interrupt routines
volatile int volume = 32;                     // current "position" of rotary encoder (increments CW)
volatile boolean rotary_change = false;      // will turn true if rotary_counter has changed
volatile boolean button_pressed = false;     // will turn true if the button has been pushed
volatile boolean button_released = false;    // will turn true if the button has been released (sets button_downtime)
volatile unsigned long button_downtime = 0L; // ms the button was pushed before release
static boolean button_down = false;
static unsigned long int button_down_start, button_down_time;
boolean muted = true; //Using this boole prevents us from having to infinitely write high or low to the pin if it doesn't need to be

//Init MCP23008
Adafruit_MCP23X08 mcp;

//Init GPIO controls
#define MUTE 13  //Digital 13
#define ROT_B 12 // rotary B
#define ROT_A 11 // rotary A

//These are the values for the bars
#define SAMPLES 64 //Must be a power of 2
#define xres 32    // Total number of  columns in the display, must be <= SAMPLES/2
#define yres 16    // Total number of  rows in the display

#define NUM_LEDS (xres * yres)

#define CLK 8 // USE THIS ON ARDUINO UNO, ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE 9
#define LAT 10
#define A A0
#define B A1
#define C A2

//Init the matrix panel
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);

byte yvalue;
byte displaycolumn, displayvalue;
int peaks[xres];
byte state = HIGH;        // the current reading from the input pin
byte previousState = LOW; // the previous reading from the input pin
byte displaycolor = 0;

//Arrays for samplig
double vReal[SAMPLES];
double vImag[SAMPLES];
byte data_avgs[xres];
arduinoFFT FFT = arduinoFFT(); // FFT object

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 100;  // the debounce time; increase if the output flickers

// EQ filter to attenuates bass and improves treble
// Useful on PC sound card which usually has many bass and poor high frequency
bool EQ_ON = true; // set to false to disable eq
byte eq[32] = {5, 10, 30, 70, 75, 80, 85, 95,
               100, 100, 100, 100, 100, 100, 100, 100,
               100, 100, 100, 100, 100, 100, 100, 100,
               115, 125, 140, 160, 185, 200, 225, 255};

void setup()
{

  //Initialize audio amplifer
  Serial.begin(9600);
  Serial.println("MAX9744 demo with volume modulated by rotary encoder.");
  Wire.begin();

  if (!setvolume(volume))
  {
    Serial.println("Failed to set volume, MAX9744 not found!");
    while (1)
      ;
  }

  //Init mcp
  /* if (!mcp.begin_I2C())
  {
    Serial.println("Cannot address mcp chip.");
    while (1)
      ;
  } */

  //Init matrix
  matrix.begin();
  ADCSRA = 0b11100101; // set ADC to free running mode and set pre-scalar to 32 (0xe5)
  ADMUX = 0b00000011;  // use pin A4 and internal voltage reference

  //Init GPIO
  //Rotary encoder
  pinMode(ROT_B, INPUT);
  digitalWrite(ROT_B, HIGH); // turn on weak pullup
  pinMode(ROT_A, INPUT);
  digitalWrite(ROT_A, HIGH); // turn on weak pullup
  //pinMode(ROT_SW, INPUT);
  //Interrupts for the rotary encoder
  //attachInterrupt(digitalPinToInterrupt(3), rotaryIRQ, CHANGE);
  PCintPort::attachInterrupt(ROT_A, &rotaryIRQ, CHANGE);
  //muting
  pinMode(MUTE, INPUT);
}


void rotaryIRQ()
{
  static unsigned char rotary_state = 0; // current and previous encoder states

  rotary_state <<= 2;                                               // remember previous state
  rotary_state |= (digitalRead(ROT_A) | (digitalRead(ROT_B) << 1)); // mask in current state
  rotary_state &= 0x0F;                                             // zero upper nybble

  Serial.println(rotary_state, HEX);

  if (rotary_state == 0x06) // from 10 to 01, increment counter. Also try 0x06 if unreliable
  {
    volume++;
    rotary_change = true;
  }
  else if (rotary_state == 0x0C) // from 00 to 11, decrement counter. Also try 0x0C if unreliable
  {
    volume--;
    rotary_change = true;
  }
}

boolean setvolume(int8_t v)
{
  Serial.print("Setting volume to ");
  Serial.println(v);
  Wire.beginTransmission(MAX9744_I2CADDR);
  Wire.write(v);
  if (Wire.endTransmission() == 0)
    return true;
  else
    return false;
}

void loop()
{
  if (rotary_change)
  {
    volume = constrain(volume, 0, 63);
    setvolume(volume);
    rotary_change = false;
  }
  if (!digitalRead(MUTE))
  { //If the hw board IS playing
    muted = false;
  }
  if (digitalRead(MUTE))
  { //If hw board is NOT playing
    muted = true;
  }

  // ++ Sampling
  for (int i = 0; i < SAMPLES; i++)
  {
    while (!(ADCSRA & 0x10))
      ;                    // wait for ADC to complete current conversion ie ADIF bit set
    ADCSRA = 0b11110101;   // clear ADIF bit so that ADC can do next operation (0xf5)
    int value = ADC - 512; // Read from ADC and subtract DC offset caused value
    vReal[i] = value / 8;  // Copy to bins after compressing
    vImag[i] = 0;
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  // -- FFT
  // ++ re-arrange FFT result to match with no. of columns on display (xres)
  int step = (SAMPLES / 2) / xres;
  int c = 0;
  for (int i = 0; i < (SAMPLES / 2); i += step)
  { //This just loops through 32
    data_avgs[c] = 0;
    for (int k = 0; k < step; k++)
    {
      data_avgs[c] = data_avgs[c] + vReal[i + k];
    }
    data_avgs[c] = data_avgs[c] / step;
    c++;
  }

  // ++ send to display according measured value
  for (int i = 0; i < xres; i++)
  { //again looping through 32
    if (EQ_ON)
      data_avgs[i] = data_avgs[i] * (float)(eq[i]) / 100; //apply eq filter
    data_avgs[i] = constrain(data_avgs[i], 0, 50);        // set max & min values for buckets to 0-80
    data_avgs[i] = map(data_avgs[i], 0, 50, 0, yres);     // remap averaged values to yres 0-8
    yvalue = data_avgs[i];
    peaks[i] = peaks[i] - 1; // decay by one light
    if (yvalue > peaks[i])
      peaks[i] = yvalue; //save peak if > previuos peak
    yvalue = peaks[i];
    displaycolumn = i;
    displayvalue = yvalue;
    matrix.drawLine(displaycolumn, 0, displaycolumn, 15, matrix.Color333(0, 0, 0)); //Clear the column first before drawing it again
    matrix.drawLine(displaycolumn, 15 - displayvalue, displaycolumn, 15, matrix.Color333(random(0, 8), random(0, 8), random(0, 8))); //Draw the column
  }
}
