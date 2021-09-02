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
            digitalWrite(relayPin,HIGH);
            state = true;
        }

        // Turn relay off
        void disable(void) {
            digitalWrite(relayPin,LOW);
            state = false;
        }

        // Toggle relay state
        void toggle(void) {
            switch (state) {
                case true:
                    disable();
                    break;

                case false:
                    enable();
                    break;
            }
        }
};

//--------------------------------------- SETUP -------------------

void setup() {
    pinMode(relayCh1Pin,OUTPUT);
    pinMode(relayCh2Pin,OUTPUT);
    pinMode(relayCh3Pin,OUTPUT);
    cout<<"Relay channels engaged"<<endl;

    relay relayCh1 = {relayCh1Pin, 1};
    relay relayCh2 = {relayCh2Pin, 2};
    relay relayCh3 = {relayCh3Pin, 3};
};

void loop(relay relayCh1, relay relayCh2, relay relayCh3) {
    // switch (argc) {
    //     case 1:
    //         cout<<"Select relay channel to toggle: "<<endl;
    //         cin>>x;

    //         switch (x) {
    //             case 1:
    //                 relayCh1.toggle();
    //                 break;

    //             case 2:
    //                 relayCh2.toggle();
    //                 break;

    //             case 3:
    //                 relayCh3.toggle();
    //                 break;
                
    //             default:
    //                 break;
    //         }
        
    //     case 2:
    //         cin>>key;

    //         switch (key) {
    //             case 
    //         }
    // }
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