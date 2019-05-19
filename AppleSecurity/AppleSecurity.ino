#include <Wire.h>
#include <VL53L0X.h>
#include <AWS_IOT.h>
#include <WiFi.h>

VL53L0X sensor;
AWS_IOT hornbill;


#define HIGH_SPEED

char WIFI_SSID[]="hackathon2019";
char WIFI_PASSWORD[]="fearlesscoder";
char HOST_ADDRESS[]="ae71f7t3bbbnb-ats.iot.us-west-2.amazonaws.com";
char CLIENT_ID[]= "client id";
char TOPIC_NAME[]= "$aws/things/clinton_toaster/shadow/update";
int toasterPin = 2;

int status = WL_IDLE_STATUS;
int tick=0,msgCount=0,msgReceived = 0;
char payload[512];
char rcvdPayload[512];

void mySubCallBackHandler (char *topicName, int payloadLen, char *payLoad)
{
    strncpy(rcvdPayload,payLoad,payloadLen);
    rcvdPayload[payloadLen] = 0;
    msgReceived = 1;
}


// Red and Green LEDs
int ledG = 27;
int ledR = 26;

// Pin definitions (Proximity Sensor)
int myLed = 2;
int intPin = 4;

int inPin = 18;
int val = 0;

int led = 5;

// Pin definitions (Motion Sensor)
int motPin = 18;
int motLed = 19;

int motVal = 0;

bool newData = false;

int distance;

bool foodPresent;

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

void setup() 
{
  pinMode(inPin, INPUT);
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  Serial.println("Hello, world!");
  delay(4000);

  pinMode(motPin, INPUT);
  pinMode(motLed, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledR, OUTPUT);


 pinMode(toasterPin, OUTPUT);
    Serial.begin(115200);
    delay(2000);

    while (status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(WIFI_SSID);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        // wait 5 seconds for connection:
        delay(5000);
    }

    Serial.println("Connected to wifi");

    if(hornbill.connect(HOST_ADDRESS,CLIENT_ID)== 0)
    {
        Serial.println("Connected to AWS");
        delay(1000);

        if(0==hornbill.subscribe(TOPIC_NAME,mySubCallBackHandler))
        {
            Serial.println("Subscribe Successfull");
        }
        else
        {
            Serial.println("Subscribe Failed, Check the Thing Name and Certificates");
            while(1);
        }
    }
    else
    {
        Serial.println("AWS connection failed, Check the HOST Address");
        while(1);
    }

    delay(2000);





  
 Wire.begin(21, 22, 400000); // SDA (21), SCL (22) on ESP32, 400 kHz rate
 
// Set up the led indicator
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);
  pinMode(intPin, INPUT);

  I2Cscan();
  
  delay(1000);
  
  sensor.init();
  sensor.setTimeout(500);

  #if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);  // minimum timing budget 20 ms
  #elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
  #endif

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

  attachInterrupt(intPin, myinthandler, FALLING);  // define interrupt for GPI01 pin output of VL53L0X
}

void loop() 
{
  if (newData) // wait for data ready interrupt
  {
     newData = false; // reset data ready flag
     Now = micros(); // capture interrupt time
     // calculate time between last interrupt and current one, convert to sample data rate, and print to serial monitor
     //Serial.print("data rate = "); Serial.print(1000000./(Now - lastUpdate)); Serial.println(" Hz");

     Serial.print(sensor.readRangeContinuousMillimeters()); // prit range in mm to serial monitor
     if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

     distance = sensor.readRangeContinuousMillimeters();
     
     if (distance < 500) {
       foodPresent = true;
       digitalWrite(ledG, HIGH);
       digitalWrite(ledR,LOW);
     } else {
       foodPresent = false;
       digitalWrite(ledR,HIGH);
       digitalWrite(ledG,LOW);
     }
     Serial.println();
  }

  if (foodPresent) {
    motVal = digitalRead(inPin);
    if (motVal != 0) {
      Serial.println("Motion detected");
      digitalWrite(motLed, HIGH);
    } else {
      digitalWrite(motLed, LOW);
    }
  } else {
    digitalWrite(motLed, LOW);
  }
  
  lastUpdate = Now;
  if(msgReceived == 1)
    {
        msgReceived = 0;
        Serial.print("Received Message:");
        Serial.println(rcvdPayload);

        digitalWrite(toasterPin, HIGH);
        delay(1000);
        digitalWrite(toasterPin, LOW);

    }
    if(tick >= 10)   // publish to topic every 5seconds
    {
        tick=0;
        //sprintf(payload,"{ \"state\": { \"desired\": {\"activate\":1}}}");
        //if(hornbill.publish(TOPIC_NAME,payload) == 0)
        //{        
        //    Serial.print("Publish Message:");
        //    Serial.println(payload);
        //}
        //else
        //{
        //    Serial.println("Publish failed");
        //}
    }  
    vTaskDelay(1000 / portTICK_RATE_MS); 
    tick++;
  
}

// Functions

void myinthandler()
{
  newData = true; // set the new data ready flag to true on interrupt
}

// I2C scan function
void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}
