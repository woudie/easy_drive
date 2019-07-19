//Include Libraries
#include <Sabertooth.h>

//Initialize both Sabretooths and set address
Sabertooth FrontST(128); //Address 128 Dip Switches (000111)
Sabertooth RearST(129); //Address 130 Dip Switches (000101) Address 129 did not work for some reason

//define high and low output limits to keep the output under 12-13 volts
//Based on the 22V input straight from the battery
#define HIGHLIM 70
#define LOWLIM -70

/* Front Sabretooth, M1 will be Right, M2 will be left
 * Rear Sabretooth, M1 will be Left,  
 */
#define FR 1
#define FL 2
#define BL 1
#define BR 2

//SetupLoop
void setup()
{
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  FrontST.autobaud(); 
  RearST.autobaud(); // Send the autobaud command to the Sabertooth controller(s).
}

void loop()
{
  frontRight(0);
  frontLeft(0);
  rearRight(0);
  rearLeft(0);
  //Add some communications fault set output to 0
}

//Validate the input is within the allowable range
int validateInput(int input){
  if(input > HIGHLIM) {
    return HIGHLIM;
  }
  if(input < LOWLIM) {
    return LOWLIM;
  }
  return input;
}

//Validates input and sets the appropriate output
void frontRight(int input) {
    input = validateInput(input);
    FrontST.motor(FR, input);
    return;
}

//Validates input and sets the appropriate output
void frontLeft(int input) {
    input = validateInput(input);
    FrontST.motor(FL, input);
    return;
}

//Validates input and sets the appropriate output
void rearRight(int input) {
    input = validateInput(input);
    RearST.motor(BR, input);
    return;
}

//Validates input and sets the appropriate output
void rearLeft(int input) {
    input = validateInput(input);
    RearST.motor(BL, input);
    return;
}
