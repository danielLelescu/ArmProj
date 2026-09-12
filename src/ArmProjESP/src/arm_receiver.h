#pragma once
 
#include <Arduino.h>
#include <BluetoothSerial.h>
 

// Parses one line of the form "a,b,c,d" into four angle globals
bool parseAngleLine(const String &line);

// Receiver setup function to be called in main's setup(), initializes Bluetooth link on board
void receiver_setup();

// Receiver routine to be run in main's loop(), returns pointer to array of parsed arm data
float* receiver_loop();