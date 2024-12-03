/********************************************************************************
Pong Steps by Joshua Jacob Pothen
Experiement 4 - Creation and Computation 2024

Refference Code Links:
1. Distance Motion Dectection:
https://github.com/DigitalFuturesOCADU/CC2024/blob/31af1a8070f3d7e2629c496767633e48b1e6b189/experiment4/Arduino/Sensors/Distance/distance_motionDetection/distance_motionDetection.ino

2. DFpong Controller 2 Button:
https://github.com/DigitalFuturesOCADU/CC2024/blob/31af1a8070f3d7e2629c496767633e48b1e6b189/experiment4/Arduino/BLE/DFpong_controller_2button/DFpong_controller_2button.ino

********************************************************************************/

/*******************************************************************************
 * Distance Sensor Motion Detection System
 * 
 * Real-time distance and motion tracking system using rolling average smoothing.
 * Calculates instantaneous motion and accumulates total movement distance.
 * 
 * Key Control Variables:
 * readInterval    = 30ms   - Main processing/output interval
 * AVERAGE_WINDOW  = 5      - Number of samples in rolling average
 * MOTION_THRESHOLD = 0.3cm - Minimum change to register as movement
 * 
 * Output Format:
 * Raw Distance (cm) | Smoothed Distance (cm) | Current Motion (cm) | 
 * Motion State (TOWARD/AWAY/STILL) | Total Motion (cm)
 * 
 * Hardware Setup:
 * TRIGGER_PIN = 2
 * ECHO_PIN = 3
 *******************************************************************************/
#include <ArduinoBLE.h>
#include "ble_functions.h"
#include "buzzer_functions.h"
//Since code is split over multiple files, we have to include them here


#include <HCSR04.h>
//include library for distance sensor

#define TRIGGER_PIN 2 //defining pin-outs for distance sensor 1
#define ECHO_PIN 3

#define TRIGGER_PIN2 4 //defining pin-outs for distance sensor 2
#define ECHO_PIN2 5

UltraSonicDistanceSensor distanceSensor(TRIGGER_PIN, ECHO_PIN);
UltraSonicDistanceSensor distanceSensor2(TRIGGER_PIN2, ECHO_PIN2);

//Name your controller!
const char* deviceName = "Pong Steps";

// Pin definitions buzzer/LED
const int BUZZER_PIN = 11;       // Pin for haptic feedback buzzer
const int LED_PIN = LED_BUILTIN; // Status LED pin

// Movement state tracking
int currentMovement1 = 0;   //Distance sensor 1 current movement value
int currentMovement2 = 0;   //Distance sensor 2 current movement value
int currentMovement = 0;   // Output Current movement value (0=none, 1=up, 2=down, 3=handshake)

// Variables for Distance Sensor 1
float distance = 0.0f;
float smoothedDistance = 0.0f;
float lastSmoothedDistance = 0.0f;
float totalMotion = 0.0f;
unsigned long lastReadTime = 0;
const unsigned int readInterval = 30;
const float MOTION_THRESHOLD = 0.3;

// Variables for Distance Sensor 2
float distance2 = 0.0f;
float smoothedDistance2 = 0.0f;
float lastSmoothedDistance2 = 0.0f;
float totalMotion2 = 0.0f;
unsigned long lastReadTime2 = 0;
const unsigned int readInterval2 = 30;
const float MOTION_THRESHOLD2 = 0.3;

// Rolling average for Distance Sensor 1
const int AVERAGE_WINDOW = 5;
float readings[AVERAGE_WINDOW];
int readIndex = 0;
float totalValue = 0;

// Rolling average for Distance Sensor 2
const int AVERAGE_WINDOW2 = 5;
float readings2[AVERAGE_WINDOW2];
int readIndex2 = 0;
float totalValue2 = 0;

//Tile Pick for Distance Sensor 1
int tilePicker = 0;
int lastTileReadTime = 0;
int TileSelectInterval = 40000;

//Tile Pick for Distance Sensor 2
int tilePicker2 = 0;
int lastTileReadTime2 = 0;
int TileSelectInterval2 = 40000;

//LED Initialisation
//UP arrows 
int LED1 = A3;
int LED2 = A4;
int LED3 = A5;

//DOWN arrows 
int LED4 = A0;
int LED5 = A1;
int LED6 = A2;


void setup() 
{
  Serial.begin(9600);
    // Configure LED for connection status indication
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  
  // Initialize Bluetooth Low Energy with device name and status LED
  setupBLE(deviceName, LED_PIN);
  
  // Initialize buzzer for feedback
  setupBuzzer(BUZZER_PIN);

  for(int i = 0; i < AVERAGE_WINDOW; i++) readings[i] = 0;
  for(int j = 0; j < AVERAGE_WINDOW2; j++) readings2[j] = 0;

  Serial.println("Distance Motion Detection Started");
  Serial.println("--------------------------------");
}

//Randomize function to decide which of the three tiles to pick
void tilePick()
  {
  unsigned long currentTime2 = millis(); //Start current time
  
  //Logic that turns on the LED linked to the correct tile, based on the randomizer
  if (tilePicker == 0)
  {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
  }
  else if (tilePicker == 1) 
  {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
  }
  else if (tilePicker == 2) 
  {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, LOW);
  }
  else if (tilePicker == 3) 
  {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, HIGH);
  }
  //Picks a random value between 1 & 3
  if (currentTime2 - lastTileReadTime >= TileSelectInterval) {
    // Picks value only if the time elapsed is greater than the time interval selected
    tilePicker = random(1,4);
    // Update values if new random value is selected
    lastTileReadTime = currentTime2;
    Serial.print("Tile 1 Picked: ");
    Serial.println(tilePicker);
  }
}
void tilePick2()
//repeat of the function above for the down arrow tiles
  {
  unsigned long currentTime4 = millis();
   if (tilePicker2 == 0)
  {
    digitalWrite(LED4, HIGH);
    digitalWrite(LED5, HIGH);
    digitalWrite(LED6, HIGH);
  }
  else if (tilePicker2 == 4) 
  {
    digitalWrite(LED4, HIGH);
    digitalWrite(LED5, LOW);
    digitalWrite(LED6, LOW);
  }
  else if (tilePicker2 == 5) 
  {
    digitalWrite(LED4, LOW);
    digitalWrite(LED5, HIGH);
    digitalWrite(LED6, LOW);
  }
  else if (tilePicker2 == 6) 
  {
    digitalWrite(LED4, LOW);
    digitalWrite(LED5, LOW);
    digitalWrite(LED6, HIGH);
  }
  if (currentTime4 - lastTileReadTime2 >= TileSelectInterval2) {
    // Read the distance
    tilePicker2 = random(4,7);
    // Update values if reading is valid
    lastTileReadTime2 = currentTime4;
    Serial.print("Tile 2 Picked: ");
    Serial.println(tilePicker2);
  }
}

void processDistance() {
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime >= readInterval) {
    float newDistance = distanceSensor.measureDistanceCm();
    
    if (newDistance > 0) {
      distance = newDistance;
      
      // Update rolling average
      totalValue = totalValue - readings[readIndex];
      readings[readIndex] = distance;
      totalValue = totalValue + distance;
      readIndex = (readIndex + 1) % AVERAGE_WINDOW;
      lastSmoothedDistance = smoothedDistance;
      smoothedDistance = totalValue / AVERAGE_WINDOW;
      
      // Calculate motion
      float change = smoothedDistance - lastSmoothedDistance;
      totalMotion += abs(change);
      
      /*
      // Print verbose status
      Serial.print("Raw Distance: ");
      Serial.print(distance);
      Serial.print(" cm | ");
      
      Serial.print("Smoothed: ");
      Serial.print(smoothedDistance);
      Serial.print(" cm | ");
      
      Serial.print("Current Motion: ");
      Serial.print(abs(change));
      Serial.print(" cm | ");
      
      Serial.print("State: ");
      if(abs(change) < MOTION_THRESHOLD) {
        Serial.print("STILL");
      } else if(change > 0) {
        Serial.print("AWAY ");
      } else {
        Serial.print("TOWARD");
      }
      Serial.print(" | ");
      
      Serial.print("Total Motion: ");
      Serial.print(totalMotion);
      Serial.println(" cm");
      */
    }
    lastReadTime = currentTime;
  }
}

void processDistance2() {
  unsigned long currentTime3 = millis();
  if (currentTime3 - lastReadTime2 >= readInterval2) {
    float newDistance2 = distanceSensor2.measureDistanceCm();
    
    if (newDistance2 > 0) {
      distance2 = newDistance2;
      
      // Update rolling average
      totalValue2 = totalValue2 - readings2[readIndex2];
      readings2[readIndex2] = distance2;
      totalValue2 = totalValue2 + distance2;
      readIndex2 = (readIndex2 + 1) % AVERAGE_WINDOW2;
      lastSmoothedDistance2 = smoothedDistance2;
      smoothedDistance2 = totalValue2 / AVERAGE_WINDOW2;
      
      // Calculate motion
      float change2 = smoothedDistance2 - lastSmoothedDistance2;
      totalMotion2 += abs(change2);
      /*
      // Print verbose status
      Serial.print("Raw Distance2: ");
      Serial.print(distance2);
      Serial.print(" cm | ");
      
      Serial.print("Smoothed2: ");
      Serial.print(smoothedDistance2);
      Serial.print(" cm | ");
      
      Serial.print("Current Motion2: ");
      Serial.print(abs(change2));
      Serial.print(" cm | ");
      
      Serial.print("State2: ");
      if(abs(change2) < MOTION_THRESHOLD2) {
        Serial.print("STILL");
      } else if(change2 > 0) {
        Serial.print("AWAY ");
      } else {
        Serial.print("TOWARD");
      }
      Serial.print(" | ");
      
      Serial.print("Total Motion2: ");
      Serial.print(totalMotion2);
      Serial.println(" cm");
      */
    }
    lastReadTime2 = currentTime3;
  }
}

void handleInput()
  {
  //flipped read method because of INPUT_PULLUP 

  int threshold1 = 28;
  int threshold2 = 56;
  int threshold3 = 80;
  
  if (tilePicker == 1) //Checks if tile 1 is selected from the randomiser
  {
    if (smoothedDistance < threshold1) //Checks if tile 1 is pressed
    {
      currentMovement1 = 1;         // UP movement
    } 
    else
    {
      currentMovement1 = 0;         // NO movement
    } 
  }
  else if (tilePicker == 2) //Checks if tile 2 is selected from the randomiser
  {
    if ((smoothedDistance > threshold1)&&(smoothedDistance < threshold2)) 
    {
      currentMovement1 = 1;         // UP movement
    } 
    else
    {
      currentMovement1 = 0;         // NO movement
    } 
  }
  else if (tilePicker == 3) //Checks if tile 3 is selected from the randomiser
  {
    if ((smoothedDistance > threshold2)&&(smoothedDistance < threshold3)) 
    {
      currentMovement1 = 1;         // UP movement
    } 
    else
    {
      currentMovement1 = 0;         // NO movement
    } 
  }
    else
  {
    currentMovement1 = 0;
  }
  //Serial.print("Current Movement: ");
  //Serial.println(currentMovement);
}
void handleInput2()
  {
  //flipped read method because of INPUT_PULLUP 

  int threshold4 = 28;
  int threshold5 = 56;
  int threshold6 = 80;
  
  if (tilePicker2 == 4)
  {
    if (smoothedDistance2 < threshold4) 
    {
      currentMovement2 = 2;         // DOWN movement
    } 
    else
    {
      currentMovement2 = 0;         // NO movement
    } 
  }
  else if (tilePicker2 == 5)
  {
    if ((smoothedDistance2 > threshold4)&&(smoothedDistance2 < threshold5)) 
    {
      currentMovement2 = 2;         // DOWN movement
    } 
    else
    {
      currentMovement2 = 0;         // NO movement
    } 
  }
  else if (tilePicker2 == 6)
  {
    if ((smoothedDistance2 > threshold5)&&(smoothedDistance2 < threshold6)) 
    {
      currentMovement2 = 2;         // DOWN movement
    } 
    else
    {
      currentMovement2 = 0;         // NO movement
    } 
  }
  else
  {
    currentMovement2 = 0;
  }
  //Serial.print("Current Movement 2: ");
  //Serial.println(currentMovement);
}

//Function to determine the CuurentMovement output based on the state of the sensors and the output of each sensors current movement
void cmTest()
{
  if (currentMovement1 == currentMovement2)
  {
  currentMovement=0;
  }
  else if (currentMovement2>0)
  {
    currentMovement=2;
  }
  else if (currentMovement1>0)
  {
    currentMovement=1;
  }
  Serial.println(currentMovement);
}
void loop() {
  
  // Update BLE connection status and handle incoming data
  updateBLE();

  //read the inputs to determine the distance from sensor 1
  processDistance();
  //read the inputs te determine the distance from sensor 2
  processDistance2();

  //Handles the logic to detemine current movement 
  handleInput();
  //Handles the logic to detemine current movement 
  handleInput2();

  //picks a random tile for up movement 
  tilePick();

  //picks a random tile for down movement 
  tilePick2();

  //Compare values between two sensors and send the correct state to P5
  cmTest();

  //send the movement state to P5  
  sendMovement(currentMovement);

  //make the correct noise
  updateBuzzer(currentMovement);
}