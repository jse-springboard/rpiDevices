#include <stdio.h>
#include <iostream>
#include <wiringPi.h>
// #include <rpiDevices.h>

using namespace std;

//------------------------ VARIABLE DEFINITIONS --------------------

#define relayCh1Pin  29
#define relayCh2Pin  28
#define relayCh3Pin  25

int timeDelay = 200;

//------------------------- CLASS DEFINITIONS ----------------------

class relay { // Class to use 3 channel RPi Relay Board HAT.
    public:
        // Pin number for the relay
        const int relayPin;

        // Store channel number as an int
        const int channelNum;

        // Relay state
        bool state = false;

        // Turn relay on
        void enable(void) {
            digitalWrite(relay::relayPin,HIGH);
            relay::state = true;
        }

        // Turn relay off
        void disable(void) {
            digitalWrite(relay::relayPin,LOW);
            relay::state = false;
        }
};

enum action {
    RELAY,
    LED,
    GPS
};
//--------------------------------------- SETUP -------------------

void setup() {
    pinMode(relayCh1Pin,OUTPUT);
    pinMode(relayCh2Pin,OUTPUT);
    pinMode(relayCh3Pin,OUTPUT);
    cout<<"Relay channels engaged"<<endl;

    relay relayCh1;
    relayCh1.relayPin = relayCh1Pin;

    relay relayCh2;
    relayCh2.relayPin = relayCh2Pin;

    relay relayCh3;
    relayCh3.relayPin = relayCh3Pin;
};

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
};

int main(void) {
    if (wiringPiSetup() < 0) {
        cout<<"Setup failed"<<endl;
        return 1;
    }
    setup();
    while(1){
        loop();
    }
};