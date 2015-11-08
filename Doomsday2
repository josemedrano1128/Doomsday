
#include <avr/pgmspace.h>
#include <Servo.h>


/**********************************************************************
 * THINGS YOU MAY WANT TO CUSTOMIZE
 **********************************************************************/
// This is the distance from the Ping sensor an object must be for the 
// laser turret to be able to lock-on and fire at it. 
#define INTRUDER_RANGE_DETECTION_IN_FEET                   10  

/**********************************************************************
 * PIN ASSIGNMENTS: (Due to the waveboard we have to use Analog pins)
 * NOTE: Pins 11,12, and 13 are the SPI connection to the SD card. 
 * Pin 10 is the chip select pin for the SD card.
 * You can use analog pins as digital pins. To do this you must refer 
 * to the analog pins by their digital name 14, 15, 16, 17, 18, and 19
 **********************************************************************/
#define INFRARED_MOTION_SENSOR      10 // This is how you reference Analog Pin A0 digitally
#define PING_PROXIMITY_SENSOR       11 // This is how you reference Analog Pin A1 digitally
#define firePin           9 // This is how you reference Analog Pin A2 digitally
#define fireLED           4
#define laserPin          2
#define statusLED          12
#define BASESERVO                        6 // This is an available PWM pin

// TIMING (DELAYS AND PAUSES)
//#define DIALOGUE_PAUSE             500 // Time (in ms) to wait after each command spoken by turret
#define CALL_OUT_TO_TARGET_DELAY  5000 // Time (in ms) to wait between commands spoken in "SEARCHING_STATE"
#define NO_MOTION_DETECTED           15000
#define PIR_SETUP_TIME              30 // Time in seconds needed for PIR Sensor to stabalize on power-up

// TURRET STATES
#define SLEEPING_STATE               1
#define SEARCHING_STATE              2
#define TARGET_AQUIRED_STATE         3
#define FIRING_STATE                 4
#define TARGET_LOST_STATE            5
#define SLEEP_MODE_ACTIVATED         6

// MEASUREMENTS
#define FOOT_IN_INCHES                                    12
#define INTRUDER_RANGE_DETECTION_IN_INCHES    FOOT_IN_INCHES * INTRUDER_RANGE_DETECTION_IN_FEET
#define MICROSECONDS_PER_INCH                         73.746

//SERVO
#define SERVO_MIN_POSTION                  0
#define SERVO_MIDDLE_POSTION              90
#define SERVO_MAX_POSTION                180
#define SERVO_DELTA                       10 // The amount we want to increment or decrement the servo 

/**********************************************************************
 * VARIABLES
 **********************************************************************/
// FLAGS
boolean motionDetected         = false; // Holds the high/low value of the PIR sensor used to detect IR movement
boolean targetSeenByPingSensor = false; // Holds the high/low value of the PING sensor used to detect whether object is in front of turret

// OTHER VARIABLES
int currentState               = SLEEPING_STATE; // Keeps track of the current state the portal turret is in.

// TIME TRACKERS
unsigned long currentMillis    = 0;
unsigned long previousMillis   = currentMillis;
unsigned long startOfSearch    = 0;    // Used to keep track of when we started our current search

// SERVO
Servo   baseservo; // Create servo Object
int             baseservoPosition = SERVO_MIDDLE_POSTION;
boolean         sweepingRight = true;

Servo fireservo;


/**********************************************************************
 * SETUP - BY CONVENTION, THIS IS THE 1ST FUNCTION ARDUINO ALWAYS CALLS.
 * WE USE IT TO SETUP THE HARDWARE
 **********************************************************************/
void setup() {

  // set up serial port
  Serial.begin(9600);
  
  // Set the pins to input or output as needed
  pinMode(INFRARED_MOTION_SENSOR, INPUT);
  pinMode(PING_PROXIMITY_SENSOR, INPUT);
  //pinMode(firePin, OUTPUT); //make this a servo
  
  baseservo.attach(BASESERVO);
  fireservo.attach(firePin);
  
  // Make sure the turret is facing forward on power-up
  moveTurretHome();
  
  // PIR Motion sensor needs to sample abient conditions for at least 30 seconds  
  // to establish a baseline before it's ready. This loop will flash an LED to 
  // provide a visible indication to the user. 
  for (int i = 0; i < (PIR_SETUP_TIME * 2); i++)
  { 
    digitalWrite(statusLED, HIGH);
    delay(250); 
    digitalWrite(statusLED, LOW);
    delay(250); 
  }  
}




/**********************************************************************
 * LOOP - THIS IS THE SECOUND FUNCTION ARDUINO CALLS. THIS IS WHERE WE 
 * DO ALL OUR WORK. THE DEVICE WILL COUNTINOUSLY CYCLE THROUGH THIS 
 * LOOP UNTIL THE POWER IS PULLED.
 **********************************************************************/

void loop() 
{

  // Check the status of the infrared sensor. Is a carbon-based life-form moving within range?
  motionDetected = digitalRead(INFRARED_MOTION_SENSOR);

  // Check PING Motion Sensor to see if something is all up in the turrets grill.
  // NOTE: Only do this when the turret is awake 
  if (currentState != SLEEPING_STATE)
  {
      delay(40); // Make sure servo has stopped before checking otherwise we will get a false detection
      targetSeenByPingSensor = searchForObjectInRange();
   
  }
  
  // Enter the state machine where the turret performs specific actions depending on the state it is in.
  switch (currentState)
  {
    case SLEEPING_STATE:

      // Just stay sleeping unless motion is detected
      Serial.println("SLEEPING_STATE");
    
      if(motionDetected)
      {
        Serial.println("SLEEPING_STATE - Motion Detected");
        
        // Turn on laser  
        digitalWrite(laserPin , HIGH);
       
        // Go to next state
        currentState = SEARCHING_STATE;
      }

      break;
      
    case SEARCHING_STATE:
      
      // Let's get the current time in milliseconds so we know when to abandon our search.
      // NOTE: Need to make sure that we set this variable (startOfSearch to zero prior to 
      // leaving this state so the next time we enter it we can get the time again).
      if (startOfSearch == 0)
        startOfSearch = millis();
        
      // Print debug message
      Serial.println("SEARCHING_STATE");
       
      if (targetSeenByPingSensor)
      {
        Serial.println("SEARCHING_STATE - Target Seen");
        currentState = TARGET_AQUIRED_STATE;
        startOfSearch = 0; // Since we are changing state we need to reset this
      }
      else // Continue to search for a little while
      {
        Serial.println("SEARCHING_STATE - Searching");
        
        // Figure out how much time has passed
        currentMillis = millis();

        //moveTurret(servo, servoPosition, sweepingRight);
        moveTurret();
        
        if (currentMillis > previousMillis + CALL_OUT_TO_TARGET_DELAY)
        {
          // Decide what we are going to say. 
          if (!motionDetected && currentMillis > startOfSearch + NO_MOTION_DETECTED)
          {
            Serial.println("SEARCHING_STATE - Switch to Sleep Mode");
            currentState = SLEEP_MODE_ACTIVATED;
            startOfSearch = 0;  // Since we are changing state we need to reset this
          } 
          else
          {
            Serial.println("SEARCHING_STATE - Call out to intruder");
           
            //delay(DIALOGUE_PAUSE);
           
            previousMillis = millis();  
          }
        }
      }  
    
      break;
      
    case TARGET_AQUIRED_STATE:
      
      Serial.println("TARGET_AQUIRED_STATE - Acknowledge Object Before Firing");

      currentState = FIRING_STATE;

      break;
    
    case FIRING_STATE:
    
      if (targetSeenByPingSensor)
      {  
        Serial.println("FIRING_STATE - Shoot to kill");
        //playcomplete(turretAttackWavFiles[(rand()%3)]);
      }
      else
      {
          currentState = TARGET_LOST_STATE;
      }
    
      break;
 
    case TARGET_LOST_STATE:
    
      Serial.println("TARGET_LOST_STATE");
      
      //playcomplete(turretSearchingWavFiles[4]);
      //delay(2 * DIALOGUE_PAUSE);
      
      currentState = SEARCHING_STATE;
      
      break;

    case SLEEP_MODE_ACTIVATED:

      Serial.println("SLEEPING_MODE_ACTIVATED");     
      moveTurretHome();
      
      // Turn off wake LED
      digitalWrite(laserPin , LOW);
      
      //playcomplete(turretPowerDownWavFiles[rand()%4]);
      //delay(DIALOGUE_PAUSE);
      
      currentState = SLEEPING_STATE;
      
      break;
      
    default:
      Serial.println("DEFAULT");
        
  } // End Switch/Case 

} // End loop()



/**********************************************************************
 * PING SENSOR FUNCTIONS
 **********************************************************************/
 
boolean searchForObjectInRange()
{
  long duration;
  float inches;
  boolean result = false;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(PING_PROXIMITY_SENSOR, OUTPUT);
  digitalWrite(PING_PROXIMITY_SENSOR, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PROXIMITY_SENSOR, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PROXIMITY_SENSOR, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(PING_PROXIMITY_SENSOR, INPUT);
  duration = pulseIn(PING_PROXIMITY_SENSOR, HIGH);
  
  // convert the time into a distance
  inches = microsecondsToInches(duration);

  if (inches <= (INTRUDER_RANGE_DETECTION_IN_INCHES))
    result = true;
  
  return (result);
}

float microsecondsToInches(float microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / MICROSECONDS_PER_INCH / 2;
}



/**********************************************************************
 * SERVO FUNCTIONS
 **********************************************************************/ 

void moveTurret()
{
  if (sweepingRight)
  {
    if (baseservoPosition < SERVO_MAX_POSTION)
    {
      baseservoPosition += SERVO_DELTA;
    }
    else
    {
      sweepingRight = false;
      baseservoPosition -= SERVO_DELTA;
    }
  }
  else // Sweep Left
  {
    if (baseservoPosition > SERVO_MIN_POSTION)
    {
      baseservoPosition -= SERVO_DELTA;
    }
    else
    {
      sweepingRight = true;
      baseservoPosition += SERVO_DELTA;
    }
  }
  
  Serial.println("Moving Servo To Postion ");
  Serial.println(baseservoPosition);

  // Move the survo
  baseservo.write(baseservoPosition);
  delay(20);
  //SoftwareServo::refresh();
}


void moveTurretHome()
{
  unsigned long tStart, tEnd;
  
  Serial.println("Moving Servo Home: ");
  Serial.println(SERVO_MIDDLE_POSTION);
  
  
  // Move the survo home
  baseservo.write(SERVO_MIDDLE_POSTION);

  tStart = millis();
  tEnd = tStart;
  
  while (tStart < tEnd + 2000)
  {
    delay(20); 
    //SoftwareServo::refresh();
    tStart = millis();
  }
}

