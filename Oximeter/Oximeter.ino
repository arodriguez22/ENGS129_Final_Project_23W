#include <TimerThree.h>
#include <SPI.h>
// Online C compiler to run C program online
#include <stdio.h>


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


//PWM Algorithm
int highNum = 256;
int lowNum = 0;
int possibleNum = highNum + lowNum -1;
int possibleNumNew;
double threshHold = 3.0; // Make higher


//Threshold Method
int max_peaks = 5;
int midPoint = 1.65;
bool peaking = false; 
int peaks[max_peaks] = {0};
int peak_times[max_peaks] = {0};
  // How many peaks recorded
int num_of_peaks = 0;


// Enough so that we just need to use the max function
int index_1_count = 100;


// VOltage arrays for each of the indices of the peaks array. We want it large enough to hit the other zero. 
int index_1[index_1_count] = {0};
int index_2[index_1_count] = {0};
int index_3[index_1_count] = {0};
int index_4[index_1_count] = {0};
int index_5[index_1_count] = {0};

// Counter for voltage array

int voltage_stream = 0;

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
    //heart_rate();
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

String window_print(){
    String output = "[ ";
    for (int k=0;k<window_size;k++){

        if (k==0){
            output = output + window[k] + " ";
        }else{
            output = output + window[k] + " ";
        }
    }
    output = output + " ]";

    return output;
}

void loop(void)
{

  

  //read ADC0
  getADC(data, MISOPin0);
  ADCdata0_old = ADCdata0;
  ADCdata0 = ADCdata0_new;
  ADCdata0_new = ((data[0] << 8) + data[1]);

  //read ADC1
  getADC(data, MISOPin1); 
  ADCdata1 = ((data[0] << 8) + data[1]);

// Checks if threashold cross while only counting every other cross and noting those times
  if(ADCdata0 >= midPoint && peaking == false){
    peaks[num_of_peaks] = ADCdata0;
    peak_times[num_of_peaks] = millis();
    num_of_peaks += 1;
    peaking == true;
    
    if (reading_voltage){

      
        
      switch(voltage_stream){
        
        case 0:
          index_1[num_of_peaks] = ADCdata0;
        case 1: 
          index_2[num_of_peaks] = ADCdata0;
        case 2:
          index_3[num_of_peaks] = ADCdata0;
        case 3:
          index_4[num_of_peaks] = ADCdata0;
        case 4:
          index_5[num_of_peaks] = ADCdata0;
        
        
        }
      
    }
     


    // We want to disable after we hit the other zero
    reading_voltage = true;
    
  }else if(ADCdata0 >= midPoint && peaking == true){
    peaking = false;

    voltage_stream += 1;

     // We only need this portion to get the Vpp 
    reading_voltage = false;
    // We can also do additional time processing here if needed
    //
    //
    //
  }


  if (num_of_peaks == 5){
    // Get Heart Rate

      // Convert ms to minute
      
      // take the time difference between each point and divide it by the peak
      
      // Average the values out

    // Get AC amplitude
      // PREV STEP: CREATE 5 VERY LARGE ARRAYS, KEEP TRACK OF THE VOLTAGE TILL THE THRESHOLD IS REACHED AGAIN
      // take the maxes of each

    int index_1_max = max(index_1);
    int index_2_max = max(index_2);
    int index_3_max = max(index_3);
    int index_4_max = max(index_4);
    int index_5_max = max(index_5);
      
      // subtract 1.65 from each value
      
      // AVerage them

      int heart_sum = (index_1_max-midPoint)+(index_2_max-midPoint)+(index_3_max-midPoint)+(index_4_max-midPoint)+(index_5_max-midPoint) 

      ////////// take time average 

      int time_sum;

      hr = (heart_sum)/time_sum;
      
  }
  }
  
}
