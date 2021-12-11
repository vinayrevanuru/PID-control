#include <PID_v1.h> //Include PID library
#define MotorEnable 6 //Motor Enable pin Runs on PWM signal
#define MotorForward 4 // Motor Forward pin
#define MotorReverse 7 // Motor Reverse pin
String readString; //stores the user input data
int User_Input = 0; // converts input string into integer
int encoderPin1 = 2; //Encoder Output 'A' must connected with interrupt pin of Arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with interrupt pin of arduino.
volatile int lastEncoded = 0; // updated value of encoder is stored.
volatile long encoderValue = 0; // Raw encoder value
int PPR = 1600; // Encoder Pulse per revolution.
int angle = 360; // Maximum degree of motion.
int REV = 0; // Set point 'REQUIRED ENCODER VALUE'
int lastMSB = 0; // Input from encoderpin1
int lastLSB = 0; // Input from encoderpin2
double kp = 5 , ki = 1 , kd = 0.01; // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
void setup() {
 pinMode(MotorEnable, OUTPUT);
 pinMode(MotorForward, OUTPUT);
 pinMode(MotorReverse, OUTPUT);
 Serial.begin(9600); //initialize serial comunication

 pinMode(encoderPin1, INPUT_PULLUP);
 pinMode(encoderPin2, INPUT_PULLUP);

 digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
 digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

 //call updateEncoder() function when any high/low changed
 //seen on interrupt pins i.e., pin 2 and pin 3
 attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
 attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);

 TCCR1B = TCCR1B & 0b11111000 | 1; // set 31KHz PWM to prevent motor noise

 myPID.SetMode(AUTOMATIC); //set PID in Auto mode
 myPID.SetSampleTime(1); // refresh rate of PID controller
 myPID.SetOutputLimits(-125, 125); // this is the MAX PWM value to move motor,here
 //change in value reflect change in speed of motor.
}


void loop() {
 while (Serial.available()) { // Check if the serial data is available.
 delay(3); // a small delay
 char c = Serial.read(); // storing input data
 readString += c; // accumulate each of the characters in readString
 }

 if (readString.length() >0) { // Verify that the variable contains information
 Serial.println(readString.toInt()); // Printing the input data in integer form
 User_Input = readString.toInt(); // Here input data is store in integer form
 }

 REV = map (User_Input, 0, 360, 0, 1600); // mapping user input degree into pulse
 Serial.print("this is REV - ");
 Serial.println(REV); // printing Required Encoder value

 setpoint = REV; // Setting the desired value
 input = encoderValue ; // data from encoder consider as a Process value
 Serial.print("encoderValue - ");
 Serial.println(encoderValue);

 myPID.Compute(); // calculate new output using the input
 pwmOut(output); //from the encoder,setpoint,kp,ki,kd
}

void pwmOut(int out) {
 if (out > 0) {
 analogWrite(MotorEnable, out);
 forward(); // calling motor to move forward
 }
 else {
 analogWrite(MotorEnable, abs(out)); .
 reverse(); // calling motor to move reverse
 }
 readString=""; // Cleaning User input, ready for new Input
}
void updateEncoder(){
 int MSB = digitalRead(encoderPin1); //MSB = most significant bit
 int LSB = digitalRead(encoderPin2); //LSB = least significant bit

 int encoded = (MSB << 1) |LSB; //converting the single bits to 2 bit binary
 int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

 // If Motor moves forward then increment the encodervalue
 if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
 // If Motor moves backward then decrement the encodervalue
 if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

 lastEncoded = encoded; //store this value for next time
}



void forward () {
 digitalWrite(MotorForward, HIGH);
 digitalWrite(MotorReverse, LOW);
}
void reverse () {
 digitalWrite(MotorForward, LOW);
 digitalWrite(MotorReverse, HIGH);
}
void finish () {
 digitalWrite(MotorForward, LOW);
 digitalWrite(MotorReverse, LOW);
}
