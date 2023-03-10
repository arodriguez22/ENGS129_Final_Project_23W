
#include <SPI.h>
// Online C compiler to run C program online
#include <stdio.h>

//window variables
const int window_size = 12;
int window[window_size];
int i = 0;

// threshold above the midpoint 1.65V that defines a peak
int midPoint = 10;
int thresh = 4;

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
int ADCdata0_old;
int ADCdata0;
int ADCdata0_new;

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
double threshHold = 3.0; // Make higher

volatile unsigned long blinkCount = 0;
volatile unsigned long cycleCount = 0;

//Heart Rate variables
double peak_time[12] = {0};
int num_peak = 0;
double avg_time = 0;

//Oxygen Variables
double SO2 = 0;
double red_level = 0;
double ir_level = 0;
int max_ir = 2048;
int min_ir = 2048;
int max_red = 2048;
int min_red = 2048;

#include <TimerThree.h>

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

  //Set timer + interrupt
  Timer3.initialize(10); //set timer for 10um
  Timer3.attachInterrupt(count1); // count every 10 microseconds

  //fb_val = 3.3/1024 * analogRead(fb);
  Serial.begin(baudRate);
}


// The interrupt will increment the count and blink the LEDs depending on the current count
void count1(void) {
  blinkCount = blinkCount + 1;  // increment everytime the timer finishes a period

  fb_val_new = 3.3/1024 * analogRead(fb); //read the new feedback values
  fb_val = (alpha * fb_val_new) + ((1- alpha) * fb_val); //add the value to the average feedback
  
  if (blinkCount == 1) {//at 10s
    blinkRED(); //turn the red LED on
  }
  else if ( blinkCount == 26) {//at 260us
    blinkRED(); //turn the red LED off
  }
  else if ( blinkCount == 51) {//at 510us
    blinkIR();//turn the ir LED on
  }
  else if ( blinkCount == 76) {//at 760us
    blinkIR();
  }
  else if(blinkCount >= 100) {//at 1ms
    blinkCount = 0;//reset the count (repeat the cycle
    count2();
  }
}

void count2(void) {
  cycleCount = cycleCount + 1;
  if(cycleCount > 500) {
    cycleCount = 0;
    changePWM();
  }
}

void changePWM(void){
  
  if (fb_val > 1.7){
    highNum = pwm_output -1;
  }
  else if (fb_val < 1.6){
    lowNum = pwm_output + 1;
  }
  else {
    highNum = pwm_output  + 5;
    lowNum = pwm_output - 5;
  }

  if (3.3/1024 * analogRead(fb) >= threshHold){
    //possibleNum = highNum + lowNum -1;
    //pwm_output = ceil(possibleNum / 2.0);
    highNum = 256;
    lowNum = 0;
    //Serial.println("triggered");
  }
  
  else {
    possibleNum = highNum + lowNum -1;
    pwm_output = ceil(possibleNum / 2.0);
  }
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
  ADCdata0_old = ADCdata0;
  ADCdata0 = ((data[0] << 8) + data[1]);
  
  //read ADC1
  getADC(data, MISOPin1);  
  ADCdata1 = ((data[0] << 8) + data[1]);

  if(ADCdata0 > max_ir) {
    max_ir = ADCdata0;
  }

  if(ADCdata0 < min_ir) {
    min_ir = ADCdata0;
  }

  if(ADCdata1 > max_red) {
    max_red = ADCdata0;
  }

  if(ADCdata1 < min_red) {
    min_red = ADCdata0;
  }

//  // Change 10 to the last index to count to
//  for (int i=0; i<12; i++){
//
//      // Print statements
//      Serial.print("Current element:");
//      Serial.println(ADCdata0);
//      Serial.println(upper);
//      
//      
//      // If upper threshold is enabled
//      if(upper){
//          if (ADCdata0>=(midPoint + hysto_thresh)){
//              printf("Upper Threshold Crossed\n");
//              upper = false; 
//          }
//      // If lower threshold is enabled    
//      } else if(!upper) {
//          if (ADCdata0<=(midPoint - hysto_thresh)){
//              printf("Lower Threshold Crossed\n");
//              upper = true;
//          }
//      }   
//  }
//  
  if(abs(ADCdata0 - 2048) < 5 && ADCdata0 < ADCdata0_old) {
    
    if(abs(peak_time[(num_peak-1)]-((double)millis() / 1000)) > 0.5){
      //Serial.print("Test: ");
      //Serial.println((double)millis() / 1000);
      
      peak_time[num_peak] = (double)millis() / 1000;
      num_peak += 1;

//      Serial.print("peak: ");
//      Serial.println((double)millis()/1000);
//      Serial.print("index: ");
//      Serial.println(num_peak%12);

      if(num_peak > 12){
        avg_time = 0;
        num_peak = 0;

        avg_time = peak_time[11] - peak_time[0];

        Serial.print("Heart Rate: ");
        Serial.println((12/avg_time)*60);

        ir_level = log10((double)((max_ir - min_ir)*3.3/4096)/sqrt(2));
        Serial.print("SO2: ");
        red_level = log10((double)((max_red - min_red)*3.3/4096)/sqrt(2));
        SO2 = red_level/ir_level;
        if(SO2*100 < 90){
          Serial.println("loading");
        }
        else{
          Serial.println(SO2*100);
        }
        //Serial.println(SO2*100);
        Serial.println(" ");

        max_ir = 2048;
        min_ir = 2048;
        max_red = 2048;
        min_red = 2048;
      }
      delay(5);
    }
  }

  
  
  ////Print ADC data
//  Serial.print("ADC0 = ");
//  Serial.println(ADCdata0);
//  Serial.print("ADC1 = ");
//  Serial.println(ADCdata1);
  
  ////Print PWM values
  Serial.print("PWM = ");
  Serial.println(pwm_output);
//  Serial.print("High = ");
//  Serial.println(highNum);
//  Serial.print("Low = ");
//  Serial.println(lowNum);
//  Serial.print("Output = ");
//  Serial.println(3.3/256 * pwm_output);

  ////print Feedback value
  //Serial.print("feedback = ");
  //Serial.println(fb_val);
}
