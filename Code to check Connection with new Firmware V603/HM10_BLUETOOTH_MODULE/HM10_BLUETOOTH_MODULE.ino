
//  =======================================================
//                  12 Oct 2017
//  =======================================================

//  =======================================================
// This code is used for the MASTER HM10 BLE Module
// Address of slave device is fixed D4:36:39:D7:11:FA
//  =======================================================

#include <SoftwareSerial.h> // this library is used to create Software Seial interface from IO pins of Arduino

//SoftwareSerial mySerial(2, 3); // RX, TX

// Initialize Software Serial Communication

SoftwareSerial mySerial(7, 8); // RX, TX
unsigned long time;

void setup()
{
    //  =============================================================
    //  Open  Hardware Serial communications and wait for port to open:
    //  =============================================================
    
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect.
    }
    Serial.println("I am Master!");
    
    //  ========================================================
    //  set the data rate for the SoftwareSerial port           
    //  ========================================================
    
    mySerial.begin(9600);
    delay(1000);
    mySerial.print("AT+RENEW");
    mySerial.print("AT+RENEW");
    mySerial.print("AT+RENEW");
    mySerial.print("AT+RENEW");
    mySerial.print("AT+RENEW");
//    mySerial.print("AT+IMME1" );
    delay(3000);
    // set Master mode
    mySerial.print("AT+ROLE1"); 
    delay(1000);
//    mySerial.print("AT+IMME0");
//    delay(1000);
//    mySerial.print("AT+IMME?");
//    delay(1000);
     mySerial.print("AT+MODE2");
    //mySerial.print("AT+HELP?"); 
    delay(1000);
    mySerial.print("AT+RESET");
    delay(3000);
    mySerial.print("AT+DISC?");
    delay(1000);
    mySerial.println("AT+CONN0");// THIS IS THE ADDRESS OF BLE MODULE #2 (D4:36:39:D7:11:FA)
    delay(1000);
//    mySerial.println("AT+START");
//    delay(1000);
    //mySerial.print("AT+CON64B853DA86D8");// THIS IS THE ADDRESS OF Samsung cell phone  (64:B8:53:DA:86:D8)
    //delay(1000);
    //float x=1.2;
    //mySerial.print("Float: ");
    //mySerial.print(x);
    //delay(1000);
    
}

void loop() // run over and over
{
    //float x=1.2;
    
    //====================================================
    //                BLE device reads data
    //====================================================
    
    if (mySerial.available())
    {
      Serial.write(mySerial.read());// BLE reads data

//      time = millis();
//
//      Serial.println(time);
    }
    
    //====================================================
    //                 BLE device writes data
    //====================================================
    
    if (Serial.available())
    {
      mySerial.write(Serial.read());// BLE writes data here
    }
}

























