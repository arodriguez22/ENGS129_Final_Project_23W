#include <TimerThree.h>
#include <SPI.h>

// This example uses the timer interrupt to blink an LED
// and also demonstrates how to share a variable between
// the interrupt and the main program.

#define baudRate 9600
byte data[2];

// SPI pins
const int CSPin   = 10;
const int MOSIPin = 11;
const int MISOPin0 = 12; // connect PMODAD1.D0 to Teensy.pin12
const int MISOPin1 = 39; // connect PMODAD1.D1 to Teensy.pin39
const int SCKPin  = 14;

//SPI Data
int ADCdata0;
int ADCdata1;

// SPI Settings: speed, mode and endianness
SPISettings settings(1000000, MSBFIRST, SPI_MODE2);  // 1MHz, MSB, 

//Digital Output pins
const int red_switch = 27;  // the pin connected to the RED switch
const int ir_switch = 28; // the pin connected to the IR switch
//Digital Output Values
int redState = LOW;
int irState = LOW;

//Analog Output pins
const int red = 29;  // the pin with the Red LED
const int ir = 30; // the pin with the IR 
//Analog Output Values
int pwm_output = 128;

//Analog Input pins
const int fb = 31; //pin where the feeback signal comes in
//Analog Input Values
const double alpha = 0.05;
double fb_val = 1.65;
double fb_val_new;

//const int T = 100;
//const long d = 0.25;

//PWM Algorithm
int highNum = 256;
int lowNum = 0;
int possibleNum = highNum + lowNum -1;
int possibleNumNew;
int thresh = 10; // Make higher

volatile unsigned long blinkCount = 0;
volatile unsigned long cycleCount = 0;

void setup(void)
{
  pinMode(CSPin, OUTPUT);
  
  // Configure SPI Pins
  SPI.begin();
  SPI.setMISO(MISOPin0);
  SPI.setMOSI(MOSIPin);
  SPI.setSCK(SCKPin);
  
  //Set the PWM frequency
  analogWriteFrequency(29, 4000);//4000hz for pin 29 and 30
  analogWriteFrequency(30, 4000);//4000hz for pin 29 and 30

  //Set pin modes
  pinMode(red_switch, OUTPUT);
  pinMode(ir_switch, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(ir, OUTPUT);
  pinMode(fb, INPUT);

  //Set/Run timer + interrupt
  Timer3.initialize(10); //set timer for 10um
  Timer3.attachInterrupt(count1); // count every 10 microseconds

  //fb_val = 3.3/1024 * analogRead(fb);
  Serial.begin(baudRate);
}


// The interrupt will increment the count and blink the LEDs depending on the current count
void count1(void) {
  blinkCount = blinkCount + 1;  // increase when LED turns on

  fb_val_new = 3.3/1024 * analogRead(fb);
  fb_val = (alpha * fb_val_new) + ((1- alpha) * fb_val);
  
  if (blinkCount == 1) {
    blinkRED();
  }
  else if ( blinkCount == 26) {
    blinkRED();
  }
  else if ( blinkCount == 51) {
    blinkIR();
  }
  else if ( blinkCount == 76) {
    blinkIR();
  }
  else if(blinkCount >= 100) {
    blinkCount = 0;
    count2();
  }
  
  //Serial.print("blinkCount = ");
  //Serial.println(blinkCount);
  //Serial.print("red state = ");
  //Serial.println(redState);
  //Serial.print("ir state = ");
  //Serial.println(irState);
  //Serial.print(" ");
}

void count2(void) {
  cycleCount = cycleCount + 1;
  if(cycleCount > 500){
    cycleCount = 0;
    changePWM();
  }
}

void changePWM(void){

  
  if (abs(analogRead(fb) - fb_val) >= thresh){
    possibleNum = highNum + lowNum -1;
    pwm_output = ceil(possibleNum / 2.0);
    highNum = 256;
    lowNum = 0;
  }
  
  if (fb_val > 1.7){
    highNum = pwm_output -1;
  }
  else if (fb_val < 1.6){
    lowNum = pwm_output + 1;
  }

  possibleNum = highNum + lowNum -1;
  pwm_output = ceil(possibleNum / 2.0);
  
//  possibleNumNew = highNum + lowNum -1;
//  
//  if(possibleNumNew == possibleNum){
//    highNum = 256;
//    lowNum = 0;
//  }

  //possibleNum = possibleNumNew;

//  if(pwm_output > 256) {
//    pwm_output = 256;
//  }
//  else if(pwm_output < 0) {
//    pwm_output = 0;
//  }
}

void blinkRED(void)
{
  //fb_val = 3.3/1024 * analogRead(fb);

  //Toggle the red LED
  if (redState == LOW) {
    redState = HIGH; 
  }
  else {
    redState = LOW;
  }
  
  if (redState == LOW) {
    analogWrite(red, 0);
    digitalWrite(red_switch, redState);
  }
  else {
    analogWrite(red, pwm_output);
    digitalWrite(red_switch, redState);
  }

  //Print Feedback value
  //Serial.print("Red feedback = ");
  //Serial.println(3.3/1024 * analogRead(fb));
}

void blinkIR(void)
{
  //fb_val = 3.3/1024 * analogRead(fb);
  
  if (irState == LOW) {
    irState = HIGH; 
  }
  else {
    irState = LOW;
  }
  
  if (irState == LOW) {
    analogWrite(ir, 0);
    digitalWrite(ir_switch, irState);
  }
  else {
    analogWrite(ir, pwm_output);
    digitalWrite(ir_switch, irState);
  }

  //Print Feedback Value
  //Serial.print("IR feedback = ");
  //Serial.println(3.3/1024 * analogRead(fb));
}

void getADC(byte* data, int whichMISO) {
    
    SPI.setMISO(whichMISO);    // set MISO pin
    
    SPI.beginTransaction(settings);
    digitalWrite(CSPin,LOW);   // pull CSPin low to start SPI communication
    data[0] = SPI.transfer(0); // grab upper byte
    data[1] = SPI.transfer(0); // grab lower byte    
    digitalWrite(CSPin,HIGH);  // set CSPin high to end SPI communication
    SPI.endTransaction();
}



void loop(void)
{

  //read ADC0
  getADC(data, MISOPin0);
  ADCdata0 = ((data[0] << 8) + data[1]);
  
  //read ADC1
  getADC(data, MISOPin1);  
  ADCdata1 = ((data[0] << 8) + data[1]);

  ////Print ADC data
  //Serial.print("ADC0 = ");
  //Serial.println(ADCdata0);
  //Serial.print("ADC1 = ");
  //Serial.println(ADCdata1);
  
  ////Print PWM values
  //Serial.print("PWM = ");
  //Serial.println(pwm_output);
  //Serial.print("High = ");
  //Serial.println(highNum);
  //Serial.print("Low = ");
  //Serial.println(lowNum);
  //Serial.print("Output = ");
  //Serial.println(3.3/256 * pwm_output);

  ////print Feedback value
  //Serial.print("feedback = ");
  //Serial.println(fb_val);
  delay(250);
}
