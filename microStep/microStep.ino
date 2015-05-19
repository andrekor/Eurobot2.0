/*
	IR Remote benytter interrupt kanal 2, derfor kan vi bare benytte kanal 0 og 1
*/
#include <Stepper.h>
#include <IRremote.h>
//For testing, to map all of the angles
#include <QueueList.h>
#include "tienstra.h"

//1 if all the steps should be saved in Queue, 0 if not
//For debugging (width test)
#define AQueue 0
#define BQueue 0
#define CQueue 0
#define oneRevolution 1600 //Muligens må endre dette. Fra instructables.com ....
#define MICRO_DELAY 500

#define dirPin 7 //the pin that co32480ntrols the direction of the steppermotor
#define stepPin 8 //Output pin for the steppermotor

#define RECV_PIN 12

#define testPin 11

#define VALUE_BEACON_A 338
#define VALUE_BEACON_B 339
#define VALUE_BEACON_C 32480
float realAverage(QueueList<float>);

//Setting ip the IR receiver
IRrecv irrecv(RECV_PIN);
decode_results results;

int rCnt;

int stepCount;
//0 if not enough angles to calculate the position, 1 if enough
int firstAstep = -1; //First step when the receiver recognize beacon A
int firstBstep = -1; //First step when the receiver recognize beacon B
int firstCstep = -1; //First step when the receiver recognize beacon C
int aSteps = -1; //steps from start to beacon A
int bSteps = -1;  //steps from start to beacon B
int cSteps = -1; //steps from start to beacon C

/*Booleans to check wether we have should zero firstA or not*/
boolean A = true;
boolean B = true;
boolean C = true;

float resolution;

float averageA = 0;
float averageB = 0;
float averageC = 0;

int counter = 0;
int a_counter = 0;
int b_counter = 0;
int c_counter = 0;

int numAngles = 0; //Number of beacons seen that are been calculated, alpha, beta, gamma

unsigned long time;

boolean towerStop = false;
Tienstra *t;

// create a queue of strings messages.
QueueList <float> queueA;
QueueList <float> queueB;
QueueList <float> queueC;

void setup() {
	t = new Tienstra();
	t->initialization();
	//Setup the ir receiver
	irrecv.enableIRIn();

	//Setup for steppermotorprin
	pinMode(testPin, INPUT);
	pinMode(dirPin, OUTPUT); //output mode to direction pin
	pinMode(stepPin, OUTPUT); //output mode to step pin
	pinMode(13, OUTPUT);
	digitalWrite(dirPin, LOW); //Initialize dir pin
	digitalWrite(stepPin, LOW); //Initialize step pin

	resolution = 360.0/oneRevolution;
	stepCount = 0; 
	time = micros();
	Serial.begin(9600);
	randomSeed(analogRead(0));
	//Set the printer of the queue
	queueA.setPrinter (Serial);
	queueB.setPrinter (Serial);
	queueC.setPrinter (Serial);
	//Setup the interrupt
	 //set timer0 interrupt at 2kHz
	//interruptSetup();
	
/*Test Cases*/
	spinTest();
	//testRun2();
	//fakeTest();
	//testBeacon();
	//widthTest();
	//widthAverage();
	//irTest();
}

//To setup the interrupts. 
void interruptSetup() {
	cli(); //stops interrupts
	 //set timer0 interrupt at 2kHz
  	TCCR0A = 0;// set entire TCCR2A register to 0
  	TCCR0B = 0;// same for TCCR2B
  	TCNT0  = 0;//initialize counter value to 0
  	// set compare match register for 2khz increments
  	OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  	// turn on CTC mode
  	TCCR0A |= (1 << WGM01);
  	// Set CS01 and CS00 bits for 64 prescaler
  	TCCR0B |= (1 << CS01) | (1 << CS00);   
  	// enable timer compare interrupt
  	TIMSK0 |= (1 << OCIE0A);

  	//Set timer1 interrupt at 4000 hz
  	TCCR1A = 0; //Set entire TCCR2A register to 0
  	TCCR1B = 0; // same for TCCR2B
  	TCNT1 = 0; //Initialize counter value to 0
  	//Set compare match register for 16 KHz increments
  	OCR1A = 1999; // (16*10^6)/(1000*8) -1 (must be < 65000)
  	//Turn on CTC mode 
  	TCCR1B |= (1 << WGM12);
  	//Set CS12 for 8 bit prescaler
  	TCCR1B |= (1 << CS11);
  	//enable timer compare interrupt
  	TIMSK1 |= (1 << OCIE2A);
  	sei(); //allow interrupts
}

int i = 0;
void loop() {
}	


/*Test function for the second prototype, with slipring which allows the tower to 
rortate continualy in the same direction.
The function, steps, and sets the angle of each of the beacons. The direction is A -> B -> C -> A*/
void spinTest() {
	while(1) {
		step();
		receiveBeaconSignal();
	}
}

/*
Test program. Turns the tower one and a half round, and calculate the angle of the beacons. 
*/
void testRun() {
	while(1) {
		step(); //steps one step of the stepper 1/1600
		//checks for beacon signal one time per two steps 2/1600 = 1/800 = 0,45 degrees resolution
		receiveBeaconSignal();
		if (stepCount == 1601)
			break;
	}
	Serial.println("Angle on the beacons:");
	Serial.println(angle(aSteps));
	Serial.println(angle(bSteps));
	Serial.println(angle(cSteps));
	delay(5000);
	aSteps = 0; bSteps = 0; cSteps = 0;
	stepCount = 0; //nullstiller stepcount
	int dir = !digitalRead(dirPin);
	digitalWrite(dirPin, dir);
	while(1) {
		step();
		receiveBeaconSignal();
		if (stepCount == 1601)
			break;
	}
	Serial.print("A ");
	Serial.print(aSteps);
	Serial.print(" last A ");
	Serial.print(firstAstep);
	Serial.print(" -> ");
	Serial.println(angle(aSteps));
	Serial.print("B ");
	Serial.print(bSteps);
	Serial.print(" last A ");
	Serial.print(firstAstep);
	Serial.print(" -> ");
	Serial.println(angle(bSteps));
	Serial.print("C ");
	Serial.print(cSteps);
	Serial.print(" last A ");
	Serial.print(firstAstep);
	Serial.print(" -> ");
	Serial.println(angle(cSteps));

}
//Run the stepper one round, and read the angles. Then calculate the position
void testRun2() {
	while(1) {
		while(1) {
			receiveBeaconSignal();
			step();
			if (stepCount >= oneRevolution) {
				break;
			}
		}
		setAverage();
		if (digitalRead(dirPin))
			angleNegative();
		else 
			anglePositive();
		if (numAngles >= 3) {
		//	Serial.println("\n Calculateing the position: ");

			t->calculate(); //the calculated x and y position is public variables in tienstra
		}
			//Prints the position
		//Print for livePlot should work like the fakeTest functioncall
		livePlot(t->XR, t->YR);

		/* print for matlab
		Serial.print("[");
		Serial.print(t->XR);
		Serial.print(" ");
		Serial.print(t->YR);
		Serial.print("], [");
		Serial.print(t->gamma);
		Serial.print(" ");
		Serial.print(t->beta);
		Serial.print(" ");
		Serial.print(t->alpha);
		Serial.println("]");
		Serial.print("[");
		Serial.print(a_counter);
		Serial.print(" ");
		Serial.print(b_counter);
		Serial.print(" ");
		Serial.print(c_counter);
		Serial.println("] ");
*/
		int dir = !digitalRead(dirPin);
		digitalWrite(dirPin, dir);
		stepCount = 0;
		zeroCounters();
		delay(750);
	}
}

void fakeTest() {
	float x_pos = 150.00;
	float y_pos = 100.00;

	for (int i = 0; i < 100; i++) {
		livePlot(x_pos, y_pos);
		x_pos += 5*fRandom(0, 1.0);
		y_pos += 5*fRandom(0, 1.0);
		delay(1000);
	}
}

//Generates a double random, from a random int
double fRandom(double fMin, double fMax) {
	double ran = (double) random(1000)/1000.0;
	return (fMin + ran*(fMax-fMin))*(random(2) == 0 ? -1 : 1);
}

void livePlot(float x1, float y1) {
	String s1 = String(round(x1));
	String s2 = String(round(y1));

	String s3 = String(s1 + ",");
	Serial.println(s3 + s2);
	/*
	String xSend = String("x"+s1);
	String ySend = String("y"+s2);
	//String s3 = String(s1 + ",");
	//String pos = String(s3 + s2);
	//String pos = String(s4 + ";");
	Serial.println(xSend);
	Serial.println(ySend);*/
//	Serial.println(y1);
}



void printStuff(int first, int average, int last, int num)  {
	Serial.print("[");
	Serial.print(first);
	Serial.print(", ");
	Serial.print(average);
	Serial.print("," );
	Serial.print(last);
	Serial.print("]   [");
	Serial.print(angle(first));
	Serial.print(", ");
	Serial.print(angle(average));
	Serial.print("," );
	Serial.print(angle(last));
	Serial.print("]   ");
	Serial.println(num);
}

void widthAverage() {
	delay(2000);
	while(1) {
		while(stepCount < oneRevolution){
			receiveBeaconSignal();
			step();
		}	
		
		Serial.print(realAverage(1));
		Serial.print(" -> A  ");
		printStuff(firstAstep, averageA, aSteps, a_counter);
		Serial.print(realAverage(2));
		Serial.print(" -> ");
		printStuff(firstBstep, averageB, bSteps, b_counter);
		Serial.print(realAverage(3));
		Serial.print(" -> ");
		printStuff(firstCstep, averageC, cSteps, c_counter);

		//Change direction of stepper
		int d = !digitalRead(dirPin);
		digitalWrite(dirPin, d);
		zeroCounters();
		delay(2000);
	}
}

float average_angle(float first, float last) {
	return (first+last)/2;
}

void testBeacon() {
	while(1) {
		if (digitalRead(testPin)) {
		while(firstAstep < 0 && stepCount < oneRevolution){
			receiveBeaconSignal();
			step();
		}	

		Serial.print("First A step ");
		Serial.print(firstAstep);
		Serial.print("   =>   ");
		Serial.println(angle(firstAstep));
		Serial.println();

//		delay(3000);

		while(firstBstep < 0 && stepCount < oneRevolution){
			step();
			receiveBeaconSignal();
		}	

		Serial.print("First B step ");
		Serial.print(firstBstep);
		Serial.print("   =>   ");
		Serial.println(angle(firstBstep));
		Serial.println();

	//	delay(3000);

		while(firstCstep < 0 && stepCount < oneRevolution){
			step();
			receiveBeaconSignal();
		}	

		Serial.print("First C step ");
		Serial.print(firstCstep);
		Serial.print("   =>   ");
		Serial.println(angle(firstCstep));

	//	delay(500);

	//Steps to the start position
		while(stepCount < oneRevolution) 
			step();
		zeroCounters();
		Serial.println("Ready for next test");
	}
	int d = !digitalRead(dirPin);
	digitalWrite(dirPin, d);
	}
}

void zeroCounters() {
	aSteps = -1;
	bSteps = -1;
	cSteps = -1;
	firstAstep = -1;
	firstBstep = -1;
	firstCstep = -1;
	stepCount = -1;
	a_counter = 0;
	b_counter = 0;
	c_counter = 0;
	averageA = 0;
	averageB = 0;
	averageC = 0;
	numAngles = 0;
	rCnt = 0;
	//stepA = NULL; //Should free everything
}

void widthTest() {
	delay(2000);
	while(1) {
		stepCount = 0;
		while (stepCount < oneRevolution) {
			receiveBeaconSignal();
			step();
		}
		setAverage();
		if (digitalRead(dirPin))
			angleNegative();
		else 
			anglePositive();

		Serial.print("[");
		Serial.print(t->gamma);
		Serial.print(" ");
		Serial.print(t->beta);
		Serial.print(" ");
		Serial.print(t->alpha);
		Serial.println("]");
		printQueue();
		int d = !digitalRead(dirPin);
		digitalWrite(dirPin, d);
		delay(2000);
		zeroCounters();
	}
}

void printQueue() {	
	#if AQueue
		Serial.print("A: ");
		printAllAngles(1); //1 - A, 2 - B, 3 - C
	#endif
	#if BQueue
		Serial.print("B: ");
		printAllAngles(2); //1 - A, 2 - B, 3 - C
	#endif
	#if CQueue
		Serial.print("C: ");
		printAllAngles(3); //1 - A, 2 - B, 3 - C
	#endif
}

void printAllAngles(int num) {
	QueueList<float> q;
	Serial.print("[");
	switch (num) {
		case 1: 
			while(!queueA.isEmpty()) {
				if (queueA.peek() < 0) {
					queueA.pop();
				}
				else {
					Serial.print(angle(queueA.pop()));	
					Serial.print(" ");
				}
			}
			break;
		case 2:
			while(!queueB.isEmpty()) {
				Serial.print(angle(queueB.pop()));	
				Serial.print(" ");
			}	
			break;
		case 3:
			while(!queueC.isEmpty()) {
				Serial.print(angle(queueC.pop()));	
				Serial.print(" ");
			}
			break;
	}
	Serial.println("]");

}

void rotate() {
	digitalWrite(stepPin, HIGH);
	digitalWrite(stepPin, LOW);
	delayMicroseconds(MICRO_DELAY);
	stepCount = (stepCount == 1600 ? 0 : stepCount+1);
	i = stepCount/oneRevolution;
	if (i >= 2) {
//		Serial.println(i);
		Serial.println(stepCount);
		stepCount = 0;
		//Change the direction of the tower
		int dir = !digitalRead(dirPin);
		Serial.print("Direction ");
		Serial.println(dir);
		digitalWrite(dirPin, dir);	
	}
}

/*
To make one step with the stepper motor
*/
void step() {
	//Maybe need to or in the value for the pin
	//if ((micros() - time) > MICRO_DELAY) {
		//time = micros();
		digitalWrite(stepPin, HIGH);
		digitalWrite(stepPin, LOW);
		stepCount = (stepCount == 1600 ? 0 : stepCount+1);
		//stepCount++;
		delayMicroseconds(MICRO_DELAY);
	//}
}

void decode() {
  if (irrecv.decode(&results)) {
    //Serial.println(results.bits); //The length of the signa
    //Serial.println(results.value);
    int value = results.value;
    int bit = results.bits;
    Serial.print(value);
    Serial.print(", ");
    Serial.println(bit);
   if (value >= 0 && value <= 3)
    Serial.println(value);
    //Serial.println(results.value);
   // Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
  }
}

/*Method that receives the IR signals, and detects 
from which beacon the signal comes from*/
void receiveBeaconSignal() {
	rCnt++;
  if (irrecv.decode(&results)) {
  	int value = results.value; //lagrer IR-koden
  	//Serial.println(value);
  //	int length = results.bits;
    switch (value) {
        case VALUE_BEACON_A:
 			setStep(1);
 			a_counter++;
          break;
        case VALUE_BEACON_B:
        	setStep(2);
        	b_counter++;
          break;
        case VALUE_BEACON_C:
        	setStep(3);
        	c_counter++;
        	break;
        default:
        	break;
   	}
    	irrecv.resume();
  } 
    //continue to look for more beacons. 
} 

/*
steps will be the last step in the intervall where the tower sees the beacon. 
*/
void setStep(int beacon) {
 	if (beacon == 1) {
		//Adds the steps in a Linked List
		#if AQueue
			queueA.push(aSteps);
		#endif
		if (A) {//first time on this round that we receive a signal from the given beacon
			A = false;
			zeroOldestBeacon(1);
			firstAstep = aSteps; 
			numAngles++; 
		}
		aSteps = stepCount%oneRevolution; 
	} else if (beacon == 2) {
		#if BQueue
			queueB.push(bSteps);
		#endif
		if (B) {
			zeroOldestBeacon(2);
			B = false;
			firstBstep = bSteps;
			numAngles++; 
		}
		bSteps = stepCount%oneRevolution;
	} else {
		#if CQueue
			queueC.push(cSteps);
		#endif
		if (C) {
			zeroOldestBeacon(3);
			C = false;
			firstCstep = cSteps;
			numAngles++; 	
		}
		cSteps = stepCount%oneRevolution;
	}
}

void printPos() {
	Serial.print(t->XR);
	Serial.print(", ");
	Serial.println(t->YR);
}

void printAngles() {
	Serial.print(t->alpha);
	Serial.print(", ");
	Serial.print(t->beta);
	Serial.print(", ");
	Serial.println(t->gamma);
}

void printSteps() {
	Serial.print("A: ");
	Serial.print(averageA);
	Serial.print(",");
	Serial.print("B: ");
	Serial.print(averageB);
	Serial.print(",");
	Serial.print("C: ");
	Serial.println(averageC);
}

void zeroOldestBeacon(int beacon) {
	setAverage();
	anglePositive();
	t->calculate();
	//printPos();
	//printAngles();
	//printSteps();
	
	//Zero the oldest beacon angles
	switch (beacon) {
		case 1: //if we see first signal from beacon A, we are allowed to look for B and C beacon. (calculate averagec)
			B = true;
			C = true;
			if (abs(firstCstep-cSteps) < 1000)
				averageC = average_angle(firstCstep, cSteps);
			else 
				averageC = average_angle((oneRevolution-max(firstCstep, cSteps)), min(firstCstep, cSteps));
			break;
		case 2: //If we see first signal from beacon B, we are allowed to look for A and C beacon. (calculate averageA)
			A = true;
			C = true;
			if (abs(firstAstep-aSteps) < 1000)
				averageA = average_angle(firstAstep, aSteps);
			else 
				averageA = average_angle((oneRevolution-max(firstAstep, aSteps)), min(firstAstep, aSteps));
			break;	
		case 3: //If we see first signal from beacon C, we are allowed to look for A and B beacon. (calculate averageB)
			A = true;
			B = true;
			if (abs(firstBstep-bSteps) < 1000)
				averageB = average_angle(firstBstep, bSteps);
			else 
				averageB = average_angle((oneRevolution-max(firstBstep, bSteps)), min(firstBstep, bSteps));
			break;
		default:
			break;
	}

}

/*Calculates the angle in degree, when we know the how many steps we have gone*/
float calcDegree(int steps) {
	return steps*(oneRevolution/360);
}

/*
A->C->B->A
Starter ved høyeste verdi..
*/
void setAverage() {
	if (abs(firstAstep-aSteps) < 1000)
		averageA = average_angle(firstAstep, aSteps);
	else 
		averageA = average_angle((oneRevolution-max(firstAstep, aSteps)), min(firstAstep, aSteps));
	if (abs(firstBstep-bSteps) < 1000)
		averageB = average_angle(firstBstep, bSteps);
	else 
		averageB = average_angle((oneRevolution-max(firstBstep, bSteps)), min(firstBstep, bSteps));
	if (abs(firstCstep-cSteps) < 1000)
		averageC = average_angle(firstCstep, cSteps);
	else 
		averageC = average_angle((oneRevolution-max(firstCstep, cSteps)), min(firstCstep, cSteps));
}

float realAverage(int num) {
	float sum = 0;
	int i = 0;
	switch (num) {
		case 1: 
			while(!queueA.isEmpty()) {
				sum += queueA.pop();
			}
			break;
		case 2:
			while(!queueB.isEmpty()) {
				sum += queueB.pop();
			}	
			break;
		case 3:
			while(!queueC.isEmpty()) {
				sum += queueC.pop();
			}
			break;
	}
	return (i == 0 ? 0 : (sum/i));
}

void anglePositive() {
	if (averageA > averageB && averageA > averageC) {
		//A er størst, altså konfigurasjon null mellom A og C
		t->alpha = angle(averageC-averageB);
		t->beta = angle(averageA-averageC);
		t->gamma = angle(averageB+(oneRevolution-averageA));
	} else if (averageB > averageC) {
		//B er størst, altså null konfigurasjon mellom B og A
		t->beta = angle(averageA-averageC);
		t->gamma = angle(averageB-averageC);
		t->alpha = angle(averageC+(oneRevolution-averageB));
		//t->gamma = angle(averageA+(1600-averageB));
	} else {
		//C er størst, altså null konfigurasjon mellom C og B
		t->gamma = angle(averageB-averageA);
		t->alpha = angle(averageC-averageB);
		t->beta = angle(averageA+(oneRevolution-averageC));
	}
}

/*
A->B->C->A
Starter ved høyeste verdi..
*/
void angleNegative() {
	if (aSteps > averageB && aSteps > averageC) {
		//A er størst, altså konfigurasjon null mellom A og C
		t->alpha = angle(averageB-averageC);
		t->gamma = angle(averageA-averageB);
		t->beta = angle(averageC+(oneRevolution-averageA));
	} else if (averageB > averageC) {
		//B er størst, altså null konfigurasjon mellom B og A
		t->beta = angle(averageC-averageA);	
		t->alpha = angle(averageB-averageC);
		t->gamma = angle(averageA+(oneRevolution-averageB));
	} else {
		//C er størst, altså null konfigurasjon mellom C og B
		t->gamma = angle(averageA-averageB);
		t->beta = angle(averageC-averageA);
		t->alpha = angle(averageB+(oneRevolution-averageC));
	}
}

float angle(int steps) {
	/*Serial.print(steps);
	Serial.print(" * 360 /");
	Serial.println(oneRevolution);*/
	return (steps*resolution);
}

//Commented out the interrupts that disabled the delay function in arduino
//Interrupt service routine for Timer0, at 2KHz Used by the arduion
//ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
/*Steps the stepper motor. 2000 times a second: 
1600 steps per revolution => 1600/2000 = 0,8 seconds per revolution.
Maybe we need a it to be a bit faster.  */ 
//	cli();	//disable interrupts while we are here
	//step(); 
//	sei(); //Allow interrupts
//	step();
	/*Use the same interrupt for the receive 
	beacon, but with frequence of 2KHz/4=500Hz*/
	
	//step();
//}

/* Cannot use timer2 since its alerady used by the 
IRremote library */
/*
ISR(TIMER1_COMPA_vect) {
//	receiveBeaconSignal();
	//decode();
}
*/
