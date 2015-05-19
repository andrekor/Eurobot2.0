/*
* File: maxSonar.ino
* Author: André Kramer Orten
*
* Max sonar 
* Inputs: TX, PW, AN
* Filters: None, median, highest_mode, lowest_mode, simple, best
* 
* IR distance sensor
* Vo, Vcc, GND
* Triangulation
*/

#include <SoftwareSerial.h>

#define txPin 3
#define rxPin 2
#define ANALOGPIN 0 //sa,e as A0
#define SONE2 A5 //analog pin for the IR sensor (sone 2 - behind robot)
#define SONE3 A4 //analog pin for the IR sensor (sone 3 - lower front)
#define START 11
#define COLLECTOR 5
//#define INPUT_PIN A2 //analog pin for the IR sensor (sone 3 - lower front)
//#define INPUT_PIN A1 //analog pin for the IR sensor
#define PW_PIN 7 //Detection of the opponent (sone 1 - opponent detection)
#define PW_SCALE 50 //calculated pulse width scale
//#define AN_SCALE 

#define FULL_STOP 0
#define SLOW_DOWN 1
#define DANGER 2
#define NO_DANGER 3

SoftwareSerial sonarSerial(rxPin, txPin, true); //define serial port for receiving data, output from maxSonar is inverted requiring true to be set

String state[] = {"FULL STOP", "SLOW DOWN", "DANGER", "NO DANGER"};
double anVolt, inches, cm, pulse;
int sum = 0;
int avgRange = 60; //Quantity of values to average
int prev_state = FULL_STOP;
int curr_state = FULL_STOP;

int prevA = 0; //something wrong with the Ultra sound, so got 14 certain times even though it shouldn't
float a1, a2, a3 = 0;
void setup() {
	Serial.begin(9600);
	//sonarSerial.begin(9600); //Start serial for maxsonar
	pinMode(START, OUTPUT); 
	digitalWrite(START, HIGH);
	pinMode(COLLECTOR, INPUT_PULLUP); 
	delay(50);
	/*fills distance values in to a1, a2 and a3*/
	a1 = pw();
	delay(100);
	a2 = pw();
	delay(100);
	a3 = pw();
}

//Since the analog readings from maxsonar are very sensitiv
//it will be more accurate to average n samples
void loop() {
	/*Sone 1, opponent detection*/
	Serial.print("A");
	int opponent = (int) pw();//opponent distance
 	/*if ((opponent != 14 || (prevA == 14)) && (abs(prevA-opponent) < 200)) {//should get rid of values when the sensor jumps down to 14.
 		//Three previous measurements
		Serial.print(opponent); //Opponent detection with Ultrasound
	} else {
		Serial.print(prevA); //Invalid reading 
	}*/
	a3 = a2; a2 = a1; a1 = opponent;
	Serial.print(averageValues());
	prevA = opponent;

	/*Sone 2, Sensor behind the robot*/
	Serial.print("B");
	Serial.print(calculateIRdistance(SONE2)); //begind robot

	/*Sone 3, Sensors low in front of the robot*/
	Serial.print("C");
	Serial.print(calculateIRdistance(SONE3));

	/*Should robot start*/
	Serial.print("D");
	Serial.println(shouldStart());
	//opponentDistance();
	delay(100);
}

/*Moving average filter. windowsize 3*/
float averageValues() {
	return (a1+a2+a3)/3;
}

/*To read from the analog pin*/
void analog(){
	for (int i = 0; i < avgRange; i++) {
		//Used to read in the analog voltage output that is being sent by the maxSonar device
		//Scale facros is (Vcc/512) per inch. 5V supply yields ~9.8mV/in
		//Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to het the actual inhes
		anVolt = analogRead(ANALOGPIN);
		Serial.println(anVolt);
		sum += anVolt;
		delay(10);
	}
	inches = sum/avgRange;
	cm = inches*2.54;
	sum = 0; 
	delay(200);
}

/*
From calculation we have found a scalingfactor on pw/50 per cm. This is for white paper material. 
*/
double pw() {
	pinMode(PW_PIN, INPUT);
	//USed to read in the pulse that is being sent by the maxSonar device. 
	//Pulse width representation with a scale factor of 147 us per Inch, or 
	//as measured 50 us per cm

	pulse = pulseIn(PW_PIN, HIGH);
	cm = pulse/50;
	return cm;
}

double inchToCm(double inch) {
	return inch*2.54;
}

boolean stringComplete = false;

void serialRead() {
	int range = EZread();
	if (stringComplete) {
		stringComplete = false;
		Serial.print("Range");
		Serial.println(range);

	}
}

/*Read distance serial*/
int EZread() {
	int result; 
	char inData[4]; 	//read data in to char array
	int index = 0;
	sonarSerial.flush();
	while (!stringComplete) {
		if (sonarSerial.available()) {
			char rByte = sonarSerial.read();
			if (rByte == 'R') { //Serial data marked R at start of data
				while(index < 3) {
					if (sonarSerial.available()) {
						inData[index] = sonarSerial.read();
						index++;
					}
				}
				inData[index] = 0x00; //nullbyte at the end to use atoi
			}
			stringComplete = true;
			result = atoi(inData);
		}
	}
	return result;
}

//IR distance
double calculateIRdistance(int sensor) {
	float x = analogRead(sensor);
	if(x < 78) {
		return 0;
	}
   return  1.6250568241692588e+002 * pow(x,0)
        + -1.5201470521433809e+000 * pow(x,1)
        +  6.0156917251002716e-003 * pow(x,2)
        + -1.0418167954007743e-005 * pow(x,3)
        +  6.4526747802433565e-009 * pow(x,4);
}

//In what state the Robot is 
void opponentDistance() {
	if (cm < 15) {
		//Full stop, and calc a new route
		curr_state = FULL_STOP;
	} else if (cm < 20) {
		//Danger area, perhaps think of calculating new rout to avoid opponent
		curr_state = DANGER;
	} else if (cm < 40) {
		//Slow down area
		curr_state = SLOW_DOWN;
	} else {
		//No danger
		curr_state = NO_DANGER;
	}
	if (curr_state != prev_state) {
		prev_state = curr_state;
		Serial.println(state[curr_state]);
	}
}

bool shouldStart() { 
	return digitalRead(COLLECTOR); 
}

