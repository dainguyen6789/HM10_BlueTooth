//  =======================================================
//                  12 Oct 2017
//  =======================================================

//  =======================================================
// This code is used for the Slave HM10 BLE Module 
//  =======================================================


#include <SoftwareSerial.h>

SoftwareSerial mySerial(7, 8); // RX, TX

char c;
void setup()
{
    //  =======================================================
    // Open serial communications and wait for port to open:
    //  =======================================================

    Serial.begin(115200);
    
    while (!Serial) {
        ; // wait for serial port to connect. Needed for Leonardo only
    }
    Serial.println("I am Slave aaa!");
    
    //  =======================================================
    // set the data rate for the SoftwareSerial port
    //  =======================================================

    mySerial.begin(9600);
    delay(1000);
    // set slave
    mySerial.print("AT+RENEW");
    delay(1000);
    mySerial.print("AT+ROLE0");
    delay(1000);
    mySerial.print("AT+ADDR?");
    delay(1000);
}

void loop() // run over and over
{
    if (mySerial.available())
    {
      //c=mySerial.read();
      Serial.write(mySerial.read());
//      delay(50);    // if we do not set delay or delay(10), we will receive noise signal, BE CAREFUL WITH DELAY VALUE
//      mySerial.write(c);
    }
    if (Serial.available())
    mySerial.write(Serial.read());
}














