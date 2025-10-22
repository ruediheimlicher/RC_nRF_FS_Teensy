#include <Arduino.h>
#include <avr/io.h>
#include <SPI.h>
#include <util/delay.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "printf.h"
#include <usb_joystick.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "display.h"

#define ACK                   1

#define LOOP_LED_PIN 2

#define RADIO_PIN 3

#define CE_PIN  9
#define CSN_PIN 10
#define MOSI_PIN  11
#define MISO_PIN 12
#define IRQ_PIN 9
#define SCK_PIN 13

#define RADIOSTARTED    1

#define LX_PIN  20
#define LY_PIN  21
#define RX_PIN  14
#define RY_PIN  15

#define LP_PIN  16
#define RP_PIN  17

#define LT_PIN  7
#define RT_PIN  4

#define YAW       0
#define PITCH     1
#define ROLL      2
#define THROTTLE  3


// ********************
// ACK Payload ********
bool newData = false;
uint8_t ackData[4] = {31,32,33,34};
// ********************
// ********************



#define OSZIA_PIN                                3

#define OSZIA_LO digitalWrite(OSZIA_PIN,LOW)
#define OSZIA_HI digitalWrite(OSZIA_PIN,HIGH)
#define OSZIA_TOGG digitalWrite(OSZIA_PIN,!(digitalRead(OSZIA_PIN)));

const uint64_t pipeIn = 0xABCDABCD71LL;  


uint16_t loopledcounter = 0;
uint16_t loopcounter = 0;

elapsedMillis pakettimer;

#define PAKETDELAY 20

// von example Getting Started
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

uint16_t resetcounter = 0;
uint16_t radiocounter = 1;

uint8_t radiostatus = 0;

uint16_t paketcounter = 0; // 20-ms

uint8_t channels[8];

// Used to control whether this node is sending or receiving
bool role = true;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0;
RF24 radio(CE_PIN, CSN_PIN); // select CE,CSN pin

LiquidCrystal_I2C lcd(0x27, 16, 2);

struct Signal 
{

byte throttle;
byte pitch;  
byte roll;
byte yaw;
byte aux1;
byte aux2;
    
};
Signal data;

void ResetData()
{

data.throttle = 0;                                         // Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
data.roll = 127;
data.pitch = 127;
data.yaw = 127;
data.aux1 = 0;                                              
data.aux2 = 0;
resetcounter++;                                               
}

//This function is used to map 0-1023 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Joystick values range from 0-1023. But its center value is not always 511. It is little different.
//So we need to add some deadband to center value. in our case 500-530. Any value in this deadband range is mapped to center 127.

int mapAndAdjustJoystickDeadBandValues(int value, bool reverse)
{
  if (value >= 530)
  {
    value = map(value, 530, 1023, 127, 254);
  }
  else if (value <= 500)
  {
    value = map(value, 500, 0, 127, 0);  
  }
  else
  {
    value = 127;
  }

  if (reverse)
  {
    value = 254 - value;
  }
  return value;
}

unsigned long lastRecvTime = 0;

void recvData()
{
  while ( radio.available() ) 
  {
    radiocounter++;
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();                                    // Receive the data | Data alınıyor

    if(ACK)
    {
          // ********************
    // ACK Payload ********
    radio.writeAckPayload(1, &ackData, sizeof(ackData));
    // ********************
    // ********************

    }
    

  }
}
uint8_t initradio(void)
{
  ResetData();                                             // Configure the NRF24 module  | NRF24 Modül konfigürasyonu
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  //radio.setChannel(100);
  radio.setChannel(124);

if(ACK)
{
  // ********************
  // ACK Payload ********
  //radio.setAutoAck(false);
  // ********************
  // ********************
   // ********************
  // ACK Payload ********
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
  // ********************
  
}
else
{
  radio.setAutoAck(false);
}
  

  //radio.setDataRate(RF24_250KBPS);    // The lowest data rate value for more stable communication  | Daha kararlı iletişim için en düşük veri hızı.
  radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
  radio.setPALevel(RF24_PA_MAX);                           // Output power is set for maximum |  Çıkış gücü maksimum için ayarlanıyor.
  radio.setPALevel(RF24_PA_MIN); 
  radio.setPALevel(RF24_PA_MAX); 
  
 
  radio.startListening(); 
   if (radio.failureDetected) 
  {
    radio.failureDetected = false;
    //delay(250);
    //Serial.println("Radio failure detected, restarting radio");
    //lcd_gotoxy(10,9);
    //lcd_puts("err");
    return 0;
  }
  else
  {
    //Serial.println("Radio OK"); 
    ResetData();
    return 1;

  }
}
// 
void init_LCD(void)
{
  //lcd.init();           // Initialisierung
  //lcd.backlight();      // Hintergrundlicht an
  //lcd.setCursor(0, 0);  // Erste Zeile
  //lcd.print("Hallo Welt!");
}

void setup() 
{
  Serial.begin(9600);

  //while (!Serial) { ; } 
  Serial.println("Hallo"); 
  delay(500);

  // initialize the transceiver on the SPI bus
   ResetData();                                            
  
  if(initradio())
  {
    radiostatus |= (1<<RADIOSTARTED);
  }

data.throttle = 10;                                         // Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
data.roll = 127;
data.pitch = 127;
data.yaw = 127;
data.aux1 = 1;                                              
data.aux2 = 1;
 Joystick.useManualSend(true);

/*
// To set the radioNumber via the Serial monitor on startup
  Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  while (!Serial.available()) {
    // wait for user input
  }
  */

  //initradio();
  //radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  //radio.stopListening(); //start the radio Transmitter 


  pinMode(LOOP_LED_PIN,OUTPUT);
  pinMode(RADIO_PIN,OUTPUT);


// analogeingaenge

  pinMode(OSZIA_PIN,OUTPUT);

/*
  pinMode(LX_PIN,INPUT); 
  pinMode(LY_PIN,INPUT);
  pinMode(RX_PIN,INPUT);
  pinMode(RY_PIN,INPUT);  

  pinMode(LP_PIN ,INPUT);
  pinMode(RP_PIN ,INPUT);

  // Pot
  pinMode(LP_PIN ,INPUT);
  pinMode(RP_PIN ,INPUT);
*/
  // Tasten

//init_LCD();

initDisplay();
   //u8g2clearDisplay(); 
   ////u8g2setFont(u8g2_font_helvR14_tr); // https://github.com/olikraus/u8g2/wiki/fntlist12
   //u8g2setFont(u8g2_font_t0_15_mr);  
   //u8g2setCursor(0, 12);
   //u8g2print(F("nRF24 T"));
   ////u8g2setFont(u8g2_font_ncenB10_tr);
   ////u8g2setFontMode(0);

//u8g2sendBuffer(); 

Serial.println("end setup");
//lcd.clear();

}
#define FAKTOR 4

void loop() 
{

  //data.roll = 127;


  channels[YAW] = data.yaw;
  channels[PITCH] = data.pitch;
  channels[ROLL] = data.roll;
  channels[THROTTLE] = data.throttle;

  Joystick.X(data.yaw*FAKTOR);
  Joystick.Y(data.pitch*FAKTOR);
  Joystick.Z(data.roll*FAKTOR);
  Joystick.Zrotate(data.throttle*FAKTOR);
  //uint8_t throttle = map(data.throttle,0,255,127,255);
  //Joystick.Zrotate(throttle);
  
  //Joystick.sliderLeft(data.throttle*FAKTOR);
  //Joystick.sliderRight(data.throttle*FAKTOR);
  //Joystick.slider(data.throttle*FAKTOR);

  if(pakettimer > PAKETDELAY)
  {
    paketcounter++;
    Joystick.send_now();
    pakettimer = 0;
  }
  

  
  loopledcounter++;
  if (loopledcounter > 0x2FFF)
  {
    //OSZIA_LO;
    loopcounter++;
    digitalWrite(LOOP_LED_PIN,!(digitalRead(LOOP_LED_PIN)));
    loopledcounter = 0;
    Serial.print(loopcounter);
    Serial.print(" ");
    //uint16_t lx = analogRead(LX_PIN);
    //Serial.print(lx);
    //lx = mapAndAdjustJoystickDeadBandValues(lx, false);
    /*
    Serial.print(" jaw: ");
    Serial.print(data.yaw);
    Serial.print(" jaw: ");
    Serial.print(data.pitch);
    Serial.print(" jaw: ");
    Serial.print(data.roll);
    Serial.print(" throttle: ");
    Serial.println(data.throttle);
    */
    //lcd.setCursor(0, 0);  // Erste Zeile
    //lcd.print(data.yaw);
    
    //lcd.setCursor(5, 0);
    //lcd.print(data.pitch);
    //lcd.setCursor(10, 0);
    //lcd.print(data.roll);
    //lcd.setCursor(15, 0);
    //lcd.print(data.throttle);

    ////lcd.setCursor(0, 1);
    ////lcd.print(paketcounter);
    //OSZIA_HI;
    //charh = u8g2getMaxCharHeight() ;
    //uint8_t charw = u8g2getMaxCharWidth() ;
    //oled_delete(0,24,72);
         // Yaw
    data.roll = 127;
      
      char buf0[4];
      sprintf(buf0, "%3d", data.yaw);
      //u8g2drawStr(0,28,buf0);
      
      //oled_delete(0,44,72);
      char buf1[4];
       // Pitch
      sprintf(buf1, "%3d", data.pitch);
      //u8g2drawStr(0,42,buf1);
      
     

      ////u8g2setCursor(0,42);
      
      ////u8g2print(data.pitch);
      char buf[4];
       //roll
      sprintf(buf, "%3d", data.roll);
      //u8g2drawStr(60,28,buf);

     
      // throttle
      sprintf(buf, "%3d", data.throttle);
      //u8g2drawStr(60,42,buf);
      ////u8g2setCursor(60,42);
      ////u8g2print(data.throttle);

    //u8g2sendBuffer();
  }
  
   if( radiostatus & (1<<RADIOSTARTED))
  {
    digitalWrite(RADIO_PIN,LOW);
    recvData();
    unsigned long now = millis();
    if ( now - lastRecvTime > 1000 ) 
    {
      OSZIA_LO;
      ResetData(); 
      OSZIA_HI;                                               // Signal lost.. Reset data | Sinyal kayıpsa data resetleniyor
    }
    
  }
  else
  {
    digitalWrite(RADIO_PIN,!digitalRead(RADIO_PIN));
  }
  





}
