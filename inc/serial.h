/**
 * @file Serial.h
 * @brief Simple serial interface, for example to talk to Arduino.
 * @author: Michael Kaess
 */

#pragma once
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <unistd.h>

class Serial {

  int m_serialPort; // file description for the serial port
  
public:

  Serial() : m_serialPort(-1) {}
  //~Serial(){close(m_serialPort);}
  // open a serial port connection
  void open(const std::string& port, int rate = 115200);

  // read a single character
  int read() ;

  // read until special character up to a maximum number of bytes
  std::string readBytesUntil(unsigned char until, int length = 300);

  // send a string
  void print(std::string str) ;

  // send an integer
  void print(int num);

  // send a double
  void print(double num);

  // send a float
  void print(float num) ;

  int print(char* str,int len) ;
};
