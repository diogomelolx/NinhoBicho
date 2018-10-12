#ifndef DEBUG_H
#define DEBUG_H

//////////////////General
#define DEBUG

#ifdef DEBUG
  #if defined(__AVR_ATmega32U4__)
    #define DEBUG_ENABLE() Serial.begin(115200); while(!Serial)
  #else
    #define DEBUG_ENABLE() Serial.begin(115200)
  #endif
  
 #define DEBUG_PRINT(x)     Serial.println(x)
 #define DEBUG_PRINTDEC(x)     Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
 #define DEBUG_PRINTHEX(x)  Serial.print(x, HEX)
#else

 #define DEBUG_ENABLE()
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)

#endif

#endif
