#include <stdio.h>
#include <iostream>
#include <wiringPi.h>

using namespace std;

int relayCh1Pin = 29;
int relayCh2Pin = 28;
int relayCh3Pin = 25;

int timeDelay = 50;

void setup() {
    pinMode(relayCh1Pin,OUTPUT);
    pinMode(relayCh2Pin,OUTPUT);
    pinMode(relayCh3Pin,OUTPUT);

    cout<<"Relay channels engaged"<<endl;
}

void loop() {
    digitalWrite(relayCh1Pin,HIGH);
	delay(timeDelay);
	digitalWrite(relayCh2Pin,HIGH);
	delay(timeDelay);
    digitalWrite(relayCh3Pin,HIGH);
	delay(timeDelay);
	digitalWrite(relayCh1Pin,LOW);
	delay(timeDelay);
	digitalWrite(relayCh2Pin,LOW);
	delay(timeDelay);
    digitalWrite(relayCh3Pin,LOW);
	delay(timeDelay);
}

int main(void) {
    if (wiringPiSetup() < 0) {
        cout<<"Setup failed"<<endl;
        return 1;
    }
    setup();
    while(1){
        loop();
    }
}