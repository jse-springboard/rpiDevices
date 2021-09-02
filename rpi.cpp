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
        // Relay state
        bool state;

        // Pin number for the relay
        int relayPin;

        // Store channel number as an int
        int channelNum;

        

        // Turn relay on
        void enable (void);

        // Turn relay off
        void disable(void);

        // Toggle relay state
        void toggle(void);
};

// Turn relay on
void relay::enable (void) {
    digitalWrite(relayPin,HIGH);
    state = true;
};

// Turn relay off
void relay::disable (void) {
    digitalWrite(relayPin,LOW);
    state = false;
};

// Toggle relay state
void relay::toggle (void) {
    switch (state) {
        case true:
            cout<<"DISABLING"<<endl;
            disable();
            break;

        case false:
            cout<<"ENABLING"<<endl;
            enable();
            break;
    }
};

relay relayCh1 = {false, relayCh1Pin, 1};
relay relayCh2 = {false, relayCh2Pin, 2};
relay relayCh3 = {false, relayCh3Pin, 3};

//--------------------------------------- SETUP -------------------

void setup() {
    pinMode(relayCh1Pin,OUTPUT);
    pinMode(relayCh2Pin,OUTPUT);
    pinMode(relayCh3Pin,OUTPUT);
    cout<<"Relay channels engaged"<<endl;
};

void loop(relay relayCh1, relay relayCh2, relay relayCh3) {
    relayCh1.toggle();
    delay(timeDelay);
    relayCh2.toggle();
    delay(timeDelay);
    relayCh3.toggle();
    delay(timeDelay);
};

int main(int argc, char* argv[]) {
    if (wiringPiSetup() < 0) {
        cout<<"Setup failed"<<endl;
        return 1;
    }
    setup();
    while(1){
        loop(relayCh1,relayCh2,relayCh3);
    }
};