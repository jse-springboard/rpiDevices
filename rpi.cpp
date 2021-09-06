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
        
        bool relayState; // Relay state
        int relayPin; // Pin number for the relay
        int channelNum; // Store channel number as an int

        public:
            // Contructor
            relay (int pin, int ch);
            
            // Change state variable
            void setState (bool newstate);

            bool getState (void);

            // Turn relay on
            void enable (void);

            // Turn relay off
            void disable(void);

            // Toggle relay state
            void toggle(void);
};

// Turn relay on
relay::relay (int pin, int ch) {
    digitalWrite(pin,LOW);
    relayState = false;
    relayPin = pin;
    channelNum = ch;
    // state = true;
};

// Set relay state
void relay::setState (bool newstate) {
    relayState = newstate;
};

// Get relay state
bool relay::getState (void) {
    return relayState;
};

// Turn relay on
void relay::enable (void) {
    digitalWrite(relayPin,HIGH);
    setState(true);
};

// Turn relay off
void relay::disable (void) {
    digitalWrite(relayPin,LOW);
    setState(false);
};

// Toggle relay state
void relay::toggle (void) {
    cout <<"Toggling pin "<<relayPin<<" from "<<getState()<<" to "<<!getState()<<endl;
    switch (getState()) {
        case true:
            disable();
            break;

        case false:
            enable();
            break;
    }
};

//--------------------------------------- SETUP -------------------

void setup() {
    pinMode(relayCh1Pin,OUTPUT);
    pinMode(relayCh2Pin,OUTPUT);
    pinMode(relayCh3Pin,OUTPUT);

    cout<<"Relay channels engaged"<<endl;
};

//--------------------------------------- MAIN --------------------

int main(int argc, char* argv[]) {
    if (wiringPiSetup() < 0) {
        cout<<"Setup failed"<<endl;
        return 1;
    }

    setup();

    relay relayCh1(relayCh1Pin, 1);
    relay relayCh2(relayCh2Pin, 2);
    relay relayCh3(relayCh3Pin, 3);
    
    cout << "Delaying ..." << endl;
    delay(2000);

    while(1){
        relayCh1.toggle();
        delay(timeDelay);
        relayCh2.toggle();
        delay(timeDelay);
        relayCh3.toggle();
        delay(timeDelay);
        // timeDelay = timeDelay/1.2;
    }
};