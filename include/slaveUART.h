#ifndef SLAVE_UART_H
#define SLAVE_UART_H

#include <vector>
#include <string>
#include <sstream>
#include <defaultPiezo.h>


using namespace defaultPiezoProperties;
using namespace std;

// Encode the piezo properties for serial as type String()
String encodeSlaveUART(){
  String encodedUART;
  String del = ",";
  vector<float> vars = {frequency1, frequency2, amplitude1, amplitude2, phase1, phase2, (float)enable1, (float)enable2};
  for (size_t i = 0; i < vars.size(); i++){
    encodedUART += (String)vars[i];
    if (i != vars.size()-1) {
      encodedUART += del;
    }
  }
  return encodedUART;
}

// Decode the piezo properties from serial
void decodeSlaveUART(string incomingBytes){
  vector<float> v;
  stringstream ss(incomingBytes);
  
  // Split comma separated string into different elements and add to vector 
  while (ss.good()){
    string substr;
    float temp;
    getline(ss, substr, ',');
    istringstream ss2(substr);
    ss2 >> temp;
    v.push_back(temp);
  }
  
  // Take elements of vector and save to appropriate variables
  int i = 1;
  frequency1 = v[i]; i++;
  frequency2 = v[i]; i++;
  amplitude1 = v[i]; i++;
  amplitude2 = v[i]; i++;
  phase1     = v[i]; i++;
  phase2     = v[i]; i++;
  enable1    = (int)v[i]; i++;
  enable2    = (int)v[i];
}


#endif