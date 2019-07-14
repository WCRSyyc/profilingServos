/* Continuous Turn Servo profiling

This code is used to determine the values to use to set the rotation speed
of continuous tern servos.  It also generates some extra profile information

Pin numbers, and code segments, are taken from the omni3-follower sketch
https://github.com/WCRSyyc/omni3-follower/blob/master/omni3-follower.ino

Regular / typical continuous turn servos have a control range from about 1000 to
2000 microseconds.  The lower values cause clockwise rotation, the higher values
caise counterclockwise rotation, with some value between setting no rotation.

For logging (Serial Monitor is awkward to capture from)
putty -serial -sercfg 8,n,1,115200,N -title 'Servo Speed Calibration' \
 -geometry 80x40+10+10 -cs 'UTF-8' -log Servo_«A¦B¦C»_calibration.csv /dev/ttyACM0
* sometimes /dev/ttyACM1
*/

#include <Servo.h>

// Serial Communication settings
// 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 748800, 115200
const long serSpeed = 115200; // Baud rate
// SERIAL_«5..8»«NEO»«12» // 5 to 8 data bits; no, even, odd parity; 1 or 2 stop bits
const int serConfig = SERIAL_8N1; // default: 8 data bits, no parity; 1 stop bit

// The pin numbers attached to LEDs that can be used for displaying the program
// state, where Serial is not wanted.

const int ledPin[] = {31,32,33,34,35,36,37,38,39,40,41,42,43};
const int LEDS = sizeof( ledPin ) / sizeof( int );
// Named indexes for led pins
#define ABORT_BLINK 12
#define TIMING_INIT 0
#define WAIT_BEGIN 1
#define WAIT_MOST 2
#define WAIT_END 3
#define SENSE_EGHO 11
#define BIT_0 5
#define BIT_1 6
#define BIT_2 7
#define BIT_3 8
#define AUTO_RUNNING 12
#define AUTO_SEEK 11
#define AUTO_WAIT_1 10
#define AUTO_WAIT_2 9

#define RUNNING_TEST 12
#define STATE_0 0
#define STATE_1 1
#define STATE_2 2
#define STATE_3 3
#define STATE_4 4
#define STATE_5 5
#define STATE_6 6
#define STATE_7 7
#define STATE_8 8
#define STATE_9 9
#define STATE_10 10
#define STATE_11 11

// Profiling abort error codes
#define ABRT_ZERO_INVERTED 1
#define ABRT_ZERO_LARGE 2
#define ABRT_COARSE_EARLY 3
#define ABRT_COARSE_SPLIT 4
#define ABRT_STOP_AFTER_EDGE 5
#define ABRT_FINE_START_UNCOVERED 6
#define ABRT_FINE_MISSED_BAR 7

// Full speed clockwise and counter CW settings for typical continious turn servo
#define MAX_CW 1000
#define MAX_CCW 2000

// Maximum unsigned long value to use for flag conditions
#define INFINITE_TIME -1ul
// Maximum time (milliseconds) for servo to reach final speed after target set
// (somewhat) arbitrary, and conservative 0.1 second
#define BASE_SETTLE 100
// Slow / coast time before reversing direction
#define BASE_BRAKING 20
unsigned int SETTLE_MILLIS;
unsigned int BRAKING_MILLIS;

#define TEST_STOP 1466

// The pin numbers attached to each of the servos (though only one used at a time)
const int servoPin[] = {22,23,24};
const int SERVOS = sizeof( servoPin ) / sizeof( int );
#define SERVO_A 0
#define SERVO_B 1
#define SERVO_C 2

// Pin connected to opto interrupter (sensor)
const int sensePin = A0; // Analog pin, but treated as digital
// IDEA check if digitalRead on a digtial pin is faster than on analog
// IDEA see if direct port read is faster than digitalRead, even with mask for single pin

Servo tstServo;
#define MAX_ROTATIONS 50
unsigned long rotationTime[ MAX_ROTATIONS ];

// Run Configuation
const int servoIdx = SERVO_C;
// const int SAMPLES = MAX_ROTATIONS; // Might not be max later
// const int SAMPLES = 30; // Might not be max later
const int SAMPLES = 1; // for first cut at the middle curve

typedef struct {
  unsigned long low;
  unsigned long high;
  unsigned long total;
  double highFraction;
} PartialTimes;
#define PART_POINTS 2
PartialTimes rotationParts[ PART_POINTS ];

char gBuf[ 80 ]; // buffer to simplify DEBUG printing
// start DEBUG block
const char cwStr[] = "CW";
const char ccwStr[] = "CCW";
const char * barEdge;
// end DEBUG block

void setup( void )
{
  Serial.begin( serSpeed, serConfig );
  while (!Serial) {} // wait for serial port to connect. Needed for native USB port only

  // Set all of the display LED pins to output, and turn them off to start
  for ( int i = 0; i < LEDS; i++ ) {
    pinMode( ledPin[ i ], OUTPUT );
    digitalWrite( ledPin[ i ], LOW );
  }

  pinMode( sensePin, INPUT_PULLUP ); // Make sure to include the pullup resistor

  tstServo.attach( servoPin[ servoIdx ]);
  tstServo.write( 90 ); // Approximately Stop

  sprintf_P ( gBuf, PSTR ( "Start of Servo Speed Calibration\nServo %c, Samples/Set = %d\n" ),
    'A' + servoIdx, SAMPLES );
  Serial.println ( gBuf );
}// ./setup(…)

void loop( void )
{
  unsigned int zeroCw, zeroCcw, fullStop;
  // int wrkStep; // TESTING
  unsigned long elapsedTime[ 4 ]; // TESTING

  SETTLE_MILLIS = BASE_SETTLE;
  BRAKING_MILLIS = BASE_BRAKING;

  // HPD start of huntForZero
  elapsedTime[ 0 ] = seekNextEdge( HIGH, 1000, 0, -1ul, NULL ); // Rotate encoder bar over sensor
  elapsedTime[ 1 ] = seekNextEdge( LOW, 1000, 0, -1ul, NULL ); // Rotate encode bar past sensor
  elapsedTime[ 2 ] = seekNextEdge( HIGH, 1000, 1466, SETTLE_MILLIS, NULL );
  elapsedTime[ 3 ] = seekNextEdge( HIGH, 1000, 0, BRAKING_MILLIS, NULL );// Wait until stopped
  sprintf_P ( gBuf, PSTR ( "Times: [%lu, %lu, %lu, %lu]" ),
    elapsedTime[ 0 ], elapsedTime[ 1 ], elapsedTime[ 2 ], elapsedTime[ 3 ]);
  Serial.println ( gBuf );
  delay ( 10000 ); // TESTING 10 seconds
  return; // TESTING

  // huntForZero( & zeroCw, & zeroCcw );
  // fullStop = zeroCw + div2rounded( zeroCcw - zeroCw );
  // brakeToStop( digitalRead( sensePin ), fullStop, BRAKING_MILLIS ); // DEBUG
  // sprintf_P ( gBuf, PSTR ( "cw zero@%u, ccw zero@%u, zero@%u" ),
  //   zeroCw, zeroCcw, fullStop );
  // Serial.println ( gBuf );
  // delay ( 5000 );

  // Since the hunting always does timing in CW and CCW pairs, the system
  // **should** always end up with the bar just CCW from the sensor.
  // fineTuneZeros();

  // start DEBUG block
  digitalWrite( ledPin[ RUNNING_TEST ], HIGH );
  delay ( 5000 );
  // Fast and manual setup for testing fineTuneZeros
  zeroCw = 1466;
  zeroCcw = 1467;
  // timeBarCrossing( 1450, 1466 ); // Slow CW, ignore time result
  // brakeToStop( digitalRead( sensePin ), 1466, BRAKING_MILLIS ); // DEBUG
  // Serial.println (F( "done initial timeBarCrossing" ));
  // delay ( 5000 );
  // Stopped with bar just CCW from sensor
  fullStop = fineTuneZeros( & zeroCw, & zeroCcw );
  brakeToStop( digitalRead( sensePin ), fullStop, BRAKING_MILLIS ); // DEBUG
  sprintf_P ( gBuf, PSTR ( "cw zero@%u, ccw zero@%u, zero@%u" ),
    zeroCw, zeroCcw, fullStop );
  Serial.println ( gBuf );
  delay ( 2000 );
  allLEDoff();
  // end DEBUG block


  // Looking usable: good curve in spreadsheet chart
  // SETTLE_MILLIS = 50;
  // BRAKING_MILLIS = 150;
  // profileSpeed( zeroCw, zeroCcw );
  //
  delay ( 60000 ); // 60 seconds

  // autoCalibrateSensor();
  // delay( 5000 ); // 5 seconds
  // Serial.println (F( "ClockWise" ));
  // timeRotations( 900, 1220, 20 ); // Fast CW
  // Serial.println (F( "CounterClockWise" ));
  // timeRotations( 1620, 2100, 20 ); // Fast CCW
  // Serial.println (F( "start ClockWise" ));
  // timeRotations( 1220, 1620, 20 ); // CW slowing, then CCW speeding


  // tstServo.writeMicroseconds(); // 1000, 1500, 2000 | 700 .. 2300
  // // start TESTING block
  // do {
  //   Serial.println (F( "\nfindEncoderBarEdge:CCW" )); // TESTING
  //   wrkStep = 1; // counterclockwise // TESTING
  //   findEncoderBarEdge( 1466 + 12 * wrkStep, 1466 - 12 * wrkStep, 1466, 1000000 ); // TESTING
  //   brakeToStop( digitalRead( sensePin ), 1466, BRAKING_MILLIS ); // TESTING
  //   Serial.println (F( "Edge found: bar sitting CCW from sensor" )); // TESTING
  //   delay ( 10000 ); // DEBUG
  //
  //   Serial.println (F( "findEncoderBarEdge:CW" )); // TESTING
  //   wrkStep = -1; // clockwise // TESTING
  //   findEncoderBarEdge( 1466 + 12 * wrkStep, 1466 - 12 * wrkStep, 1466, 1000000 ); // TESTING
  //   brakeToStop( digitalRead( sensePin ), 1466, BRAKING_MILLIS ); // TESTING
  //   Serial.println (F( "Edge found: bar sitting CW from sensor" )); // TESTING
  //   delay ( 20000 ); // DEBUG
  // } while ( true );
  // // end TESTING block

  // // start TESTING block
  // do {
  //   int timeOut;
  //   unsigned long tstStart, sawAt;
  //   tstStart = micros();
  //   timeOut = seekEdgeOfBar( 1490, 1466, & sawAt );
  //   // timeOut = seekEdgeOfBar( 1490, 1466, NULL ); // (with NULL)
  //   // sawAt = micros();
  //   brakeToStop( digitalRead( sensePin ), 1466, BRAKING_MILLIS ); // DEBUG
  //   sprintf_P ( gBuf, PSTR ( "After seekEdgeOfBar test: %lu; %d" ), sawAt - tstStart, timeOut );
  //   Serial.println ( gBuf );
  //   delay ( 5000 );
  // } while ( true );
  // // end TESTING block

  // sprintf_P ( gBuf, PSTR ( "%u to %u" ), 1450, 1445 ); // TESTING
  // abortAndReport( ABRT_ZERO_INVERTED ); // TESTING
  // sprintf_P ( gBuf, PSTR ( "%u" ), 12 ); // TESTING
  // abortAndReport( ABRT_ZERO_LARGE ); // TESTING
  // abortAndReport( 8 ); // TESTING
  // for ( int i = 1; i < 10; i ++ ) { blinkCode ( i ); delay ( 3000 ); } // TESTING
  // estimateZeroSetting( 1000u, 2000u, 70150ul, 72000ul ); // TESTING
  // tstGetRotParts(); // TESTING
  // tstCSlow(); // TESTING
  // tstDispCnt(); // TESTING
  // senseEcho(); //  * Locate the center setting, that gives 'full stop' for the continuous turn sero
  // ledTest(); // TESTING
}// ./loop(…)


/**
 * Fine tune the zero rotaton setting profile
 *
 * NOTE The values passed to this function might not be either actual zero
 *  speed setting values, or the only zero speed setting values.  The only thing
 *  that can be said (assuming this is called after 'huntForZero'), is that the
 *  program timed out looking for the edge of the encoder bar with those
 *  settings, while stepping (with decreasing step sized) toward the center
 *  from (near) maximum clockwise and counterclockwise settings.  This function
 *  is intended to more accurately locate the motion limits, and profile the
 *  servo at the *near* zero speed settings.
 *
 * NOTE the «cw¦ccw»Zero parameters are both input and output.  They are used as
 *  the base values, then updated if better information is found.
 *
 * @param cwZero - address of int with first (lowest) detected zero motion setting
 * @param ccwZero - address of int with last (hightest) detected zero motion setting
 * @returns the best available 'actual' full stop, zero motion, servo setting
 */
unsigned int fineTuneZeros( unsigned int * cwZero, unsigned int * ccwZero )
{
  // Long enough to test for the very short move distance, but short enough to
  // hopefully timeout if moving the wrong way
  const unsigned long COARSE_ZERO_WAIT = 1000000; // µs (1 second)
  const unsigned long FINE_ZERO_WAIT = 30000000; // µs (30 seconds)
  unsigned int baseStop, slowestCw, slowestCcw;

  baseStop = fineTuneSanityCheck( * cwZero, * ccwZero );

  // Do coarse search for slowest (still moving) CW and CCW servo settings
  slowestCw = coarseZeroBoundary( true, baseStop, COARSE_ZERO_WAIT );
  brakeToStop( digitalRead( sensePin ), baseStop, BRAKING_MILLIS ); // DEBUG
  Serial.println (F( "Done CW coarseZeroBoundary" )); // DEBUG

  slowestCcw = coarseZeroBoundary( false, baseStop, COARSE_ZERO_WAIT ); // counterclockwise
  brakeToStop( digitalRead( sensePin ), baseStop, BRAKING_MILLIS ); // DEBUG
  Serial.println (F( "Done CCW coarseZeroBoundary" )); // DEBUG

  * cwZero = slowestCw + 1;
  * ccwZero = slowestCcw - 1;

  // Use the new zero limits to (safely) have a closer look at the edges of
  // of the range that currently seems to set zero rotation.

  // After the CCW course zero boundary detection above, the encoder bar was
  // left not covering, and just CW from the sensor locaton.
  // HPD rotate CCW to cover sensor: slowestCcw + 1;
  //  rotate CW to uncover sensor: slowestCW
  //  rotate CCW to cover sensor: slowestCcw, long timeout
  //  slowestCcw--
  // Repeat until timeout covering

  slowestCw = fineZeroBOundary( * ccwZero, * cwZero, baseStop, FINE_ZERO_WAIT );

  return 1466;// TODO final mid-point, or other calculation
}// ./unsigned int fineTuneZeros(…)


/**
 * Do basic sanity checks on the date (to be) used by the fineTuneZeros function
 *
 * @param minZero - minimum servo setting (µs) initially detected as servo
 *   stopped.  Nearest value to clockwise rotation settings.
 * @param maxZero - maximum servo setting (µs) initially detected as servo
 *   stopped.  Nearst value to counterclockwise rotation settings.
 * @returns initial calculated mid point to use for full stop setting
 */
unsigned int fineTuneSanityCheck( const unsigned int minZero,
    const unsigned int maxZero )
{
  const unsigned int MAX_ZERO_COUNT = 10;
  unsigned int midZero;

  if ( minZero >= maxZero) { // cw zero should be less than ccw zero
    // No or inverted range
    sprintf_P ( gBuf, PSTR ( "%u to %u" ), minZero, maxZero );
    abortAndReport( ABRT_ZERO_INVERTED );
  }

  midZero = maxZero - minZero;
  // cw to ccw zero range is expected to be SMALL
  if ( midZero > MAX_ZERO_COUNT ) {
    // too many zero speed values; something likely wrong in hardware or hunt
    sprintf_P ( gBuf, PSTR ( "%u" ), midZero );
    abortAndReport( ABRT_ZERO_LARGE );
  }

  return minZero +( div2rounded( midZero ));
}// ./unsigned int fineTuneSanityCheck(…)


/**
 * Starting from the slowest (stopped) rotation speed value, increase speed in
 * the specified direction until the encoder bar blocks the sensor in a
 * reasonable time.
 *
 * @param isCw - true when search for clockwise boundary
 * @param stopSetting - servo setting to search searching from
 * @param timeLimit - microseconds to wait before increasing speed
 * @returns setting for lowest speed that does not time out.
 */
unsigned int coarseZeroBoundary( const bool isCw,
    const unsigned int stopSetting, const unsigned long timeLimit )
{
  int wrkSpeed; // Really unsigned, but well in range of signed to simplify ±
  int wrkStep;
  unsigned long startRef = INFINITE_TIME, edgeReached = INFINITE_TIME;
  bool timeOut;

  wrkStep = RotationDirectionStep ( isCw );
  // start DEBUG block
  if ( isCw ) {
    barEdge = ccwStr; // Opposide from working direction // DEBUG
  } else {
    barEdge = cwStr; // Opposide from working direction // DEBUG
  }
  // end DEBUG block

  // Find the Encoder bar edge by rotating backward from the direction being worked
  timeOut = findEncoderBarEdge( stopSetting - 12 * wrkStep,
    stopSetting + 12 * wrkStep, stopSetting, timeLimit );
  if ( timeOut ) { // Encoder bar was slow getting away from sensor
    // Might have been bad starting conditions.  Try one more time from what
    // should now be a better starting point.
    timeOut = findEncoderBarEdge( stopSetting - 12 * wrkStep,
      stopSetting + 12 * wrkStep, stopSetting, timeLimit );
    sprintf_P ( gBuf, PSTR ( "First %s findEncoderBarEdge timed out" ), barEdge ); // DEBUG
    Serial.println ( gBuf ); // DEBUG
  }
  // start DEBUG block
  if ( timeOut ) {
    sprintf_P ( gBuf, PSTR ( "%s findEncoderBarEdge retry also timed out" ), barEdge );
    Serial.println ( gBuf );
  }
  brakeToStop( digitalRead( sensePin ), stopSetting, BRAKING_MILLIS ); // DEBUG
  sprintf_P ( gBuf, PSTR ( "Rotation stopped with encoder bar just %s from sensor" ), barEdge );
  Serial.println ( gBuf );
  delay ( 5000 );
  // end DEBUG block

  // Starting from the base zero rotation setting value, increase speed in the
  // selected direction until the encoder bar blocks the sensor in a reasonable time.

  // one backward step so first step in loop gets back to zero
  wrkSpeed = stopSetting - wrkStep;
  while( digitalRead( sensePin ) == LOW ) { // bar is not covering sensor
    wrkSpeed += wrkStep; // one step (faster)
    startRef = micros();
    tstServo.writeMicroseconds( wrkSpeed );
    while( digitalRead( sensePin ) == LOW ) { // bar is not covering sensor
      edgeReached = micros();
      if (( edgeReached - startRef )> timeLimit ) { // Give up at this setting
        break;
      }
    }// ./while( digitalRead( sensePin ) == LOW )
    sprintf_P ( gBuf, PSTR ( "Done coarse check @%u" ), wrkSpeed ); // DEBUG
    Serial.println ( gBuf ); // DEBUG
  }// ./while( digitalRead( sensePin ) == LOW )
  if ( startRef == INFINITE_TIME ) { abortAndReport( ABRT_COARSE_EARLY ); }
  if ( edgeReached == INFINITE_TIME ) { abortAndReport( ABRT_COARSE_SPLIT ); }
  sprintf_P ( gBuf, PSTR (
    "Took %lu µs coarse rotating to block sensor@%u, starting from %u" ),
    edgeReached - startRef, wrkSpeed, stopSetting ); // DEBUG
  Serial.println ( gBuf ); // DEBUG

  // Sensor is covered by encoder bar

  return (unsigned int)wrkSpeed;
}// ./unsigned int coarseZeroBoundary(…)


/**
 * Find the lowest speed setting in the forward direction that can be measured
 *
 * @param forwardSetting - servo setting for (near) zero in forward direction
 * @param backwardSetting - servo setting for (near) zero in reverse direction
 * @param fullStop - servo setting that really is for zero rotation speed
 * @param timeLimit - microseconds to wait before increasing speed
 * @returns setting for lowest speed that does not time out.
 */
unsigned int fineZeroBOundary( const int forwardSetting, const int backwardSetting,
    const int fullStop, const long timeLimit )
{
  // int wrkSpeed; // Really unsigned, but well in range of signed to simplify ±
  int wrkStep;
  bool forwardIsCw;//, timeOut;
  unsigned long seekElapsed;//, startRef = INFINITE_TIME;//, edgeReached = INFINITE_TIME;

  BRAKING_MILLIS = BASE_BRAKING;

  forwardIsCw = forwardSetting < backwardSetting;
  wrkStep = RotationDirectionStep ( forwardIsCw );
  // start DEBUG block
  if ( forwardIsCw ) {
    barEdge = ccwStr; // Opposide from working direction // DEBUG
  } else {
    barEdge = cwStr; // Opposide from working direction // DEBUG
  }
  // end DEBUG block

  // Sanity check
  if( digitalRead( sensePin ) == LOW ) { // encoder bar is NOT covering sensor
    abortAndReport ( ABRT_FINE_START_UNCOVERED );
  }
  sprintf_P ( gBuf, PSTR ( "Forward %u, %u stopped %u, %u, backward" ),
    forwardSetting + wrkStep, forwardSetting, backwardSetting, backwardSetting - wrkStep ); // DEBUG
  Serial.println ( gBuf ); // DEBUG

  // Find the encoder bar edge by rotating backward from the direction being worked
  seekElapsed = seekNextEdge ( HIGH, backwardSetting - wrkStep, fullStop, timeLimit, NULL );
  // timeOut = ( seekElapsed == INFINITE_TIME );
  if ( seekElapsed == INFINITE_TIME ) {
    sprintf_P ( gBuf, PSTR ( "%lu" ), backwardSetting - wrkStep );
    abortAndReport ( ABRT_FINE_MISSED_BAR );
  }
  sprintf_P ( gBuf, PSTR ( "Found backward edge in %lu µs" ), seekElapsed ); // DEBUG
  Serial.println ( gBuf ); // DEBUG
  delay ( BRAKING_MILLIS ); // come to a complete stop

  seekElapsed = seekNextEdge ( LOW, forwardSetting + wrkStep, fullStop, timeLimit, NULL );
  if ( seekElapsed == INFINITE_TIME ) {
    sprintf_P ( gBuf, PSTR ( "%lu" ), forwardSetting + wrkStep );
    abortAndReport ( ABRT_FINE_MISSED_BAR );
  }
  sprintf_P ( gBuf, PSTR ( "Found forward edge in %lu µs" ), seekElapsed ); // DEBUG
  Serial.println ( gBuf ); // DEBUG
  delay ( BRAKING_MILLIS ); // come to a complete stop
  // HPD IDEA loop back and for a few times to check timing variability
  // HPD reduce forward speed and repeat

  return forwardSetting; // TODO calculate new value
}// ./unsigned int fineZeroBOundary(…)


/**
 * Calculate step value to increase speed by minimal amount
 *
 * @param rotIscW - true when a servo setting is for clockwise rotation
 * @returns ±1 value to increase rotation speed one step in the specified direction
 */
int RotationDirectionStep ( const bool rotIsCw )
{
  if ( rotIsCw ) { return -1; }
  return 1;
}// ./int RotationDirectionStep (…)


/**
 * Attempt to detect an edge of the encoder bar : sensor state transition
 *
 * @param startSenseState - the current sensor state, to detect transistion away from
 * @param seekSetting - servo setting (µs) to use for attempt, or 0 to continue using current
 * @param endSetting - servo setting (µs) to set after success or fail, 0 for no change
 * @param timeLimit - maximum time (µs) to wait for transition
 * @param refTime - address of reference time
 *   when not null, and seekSetting == zero, contains start time to measure from
 *   when not null, and endSetting == zero, will hold detected transition time
 * @returns elapsed microseconds for search
 *   -1ul on timeout
 *   0 when initial state is already past the expected transistion
 */
unsigned long
seekNextEdge ( const bool startSenseState, const unsigned int seekSetting, const unsigned int endSetting,
  const unsigned long timeLimit, unsigned long * refTime )
{
  unsigned long startRef, transitionRef, seekLimit; // Time stamps (µs)

  // Get the starting time stamp reference used to check for timeout
  if ( seekSetting != 0 ) {
    tstServo.writeMicroseconds( seekSetting );
    startRef = micros();
  } else if ( refTime == NULL ) {
    startRef = micros();
  } else {
    startRef = * refTime;
  }
  seekLimit = startRef + timeLimit; // Time stamp reference when timeout will occur
  // TODO if seekLimit < startReff abortAndReport (ABRT_MICROS_WRAPPED)

  if( digitalRead( sensePin ) != startSenseState ) { // Transition already past
    updateDetectionTime( endSetting, refTime, startRef );
    return 0ul;
  }

  while( digitalRead( sensePin ) == startSenseState ) { // wait for transistion (edge)
    if ( micros() > seekLimit ) { // Taking too long, not moving (fast enough)
      // Update target speed and save starting time, in case want to continue
      // timing after timeout
      updateDetectionTime( endSetting, refTime, startRef );
      return INFINITE_TIME; // Timeout occurred
    }// ./if ( micros() > seekLimit )
  }// ./while( digitalRead( senselapsedMicrosePin ) == startSenseState )
  transitionRef = micros(); // Encoder edge detected
  updateDetectionTime( endSetting, refTime, transitionRef ); // Set speed; save time stamp

  return transitionRef - startRef; // elapsed time (µs)
}// ./unsigned long seekNextEdge (…)


/**
 * Bring servo to a full stop
 *
 * Fatal error if sensor state changes before braking time (timeout)
 *
 * @param startSenseState - the current sensor state when braking was started
 * @param stopSetting - servo target setting that should mean zero rotation
 * @param brakeTime - the time (µs) to wait before assuming target speed reached
 */
void brakeToStop ( const bool startSensorState, const unsigned int stopSetting,
  const unsigned long brakeTime )
{
  unsigned int elapsedTime;

  elapsedTime = seekNextEdge( startSensorState, stopSetting, 0, brakeTime, NULL );
  if ( elapsedTime != INFINITE_TIME ) {
    sprintf_P ( gBuf, "%d to %d in %lu µs (< %lu)",
      startSensorState, !startSensorState, elapsedTime, brakeTime );
    abortAndReport ( ABRT_STOP_AFTER_EDGE );
  }
}// ./void brakeToStop (…)


/**
 * Change the servo to a new target setting, and save the reference time stamp
 *
 * @param targetSetting - the new servo speed setting: 0 for no change
 * @param timestamp - address to hold reference time only when changing speed
 * @param refTime - the reference time for the setting change (µs)
 */
void updateDetectionTime( const unsigned int targetSetting,
  unsigned long * timestamp, const unsigned long refTime )
{
  if ( targetSetting != 0 ) {
    tstServo.writeMicroseconds( targetSetting ); // Change to new setting speed
  } else if ( timestamp != NULL ) {
    * timestamp = refTime;
  }
}// ./updateDetectionTime(…)


/**
 * Rotate servo to position the encoder bar next to, but not (quite) blocking
 * the sensor
 *
 * When the forward speed is a CW setting, the encoder bar will be positioned
 * just (barely) CW from the sensor location.  Otherwise, CCW from there.
 *
 * @param foreSpeed - servo setting for rotating forward
 * @param backSpeed - servo setting for rotating backward
 * @param baseStop - servo setting that should really / safely stop all rotation
 * @param timeLimit - time (µs) to wait for a transistion before adjusting speed
 * @returns true when at least one of the physical state transitions did not
 *   occur within the specified time limit
 */
bool findEncoderBarEdge ( const unsigned int foreSpeed, const unsigned int backSpeed,
    const unsigned int baseStop, const unsigned long timeLimit )
{
  unsigned int timeoutSpeed;
  unsigned long startTime, elapsedTime;
  bool timeOut = false;
  Serial.print (F( "Forward/backward speed: " ));
  Serial.print ( foreSpeed );
  Serial.print (F( "/" ));
  Serial.println ( backSpeed );

  // Rotate encoder bar forward enough to get bar clear of the sensor
  elapsedTime = seekNextEdge ( HIGH, foreSpeed, baseStop, timeLimit, NULL );
  if ( elapsedTime == 0 ) { // Sensor not currently covering sensor
    // Rotate backward enough for the encoder bar to block the sensor
    elapsedTime = seekNextEdge ( LOW, backSpeed, baseStop, timeLimit, NULL );
    if ( elapsedTime == INFINITE_TIME ) {
      elapsedTime = seekNextEdge ( LOW, getBoostSpeed( backSpeed, baseStop),
        baseStop, INFINITE_TIME, NULL );
    }
    Serial.println (F( "done backward seekEdgeOfBar" ));
    // if ( elapsedTime == 0 ) { abortAndReport(); }
    brakeToStop( HIGH, baseStop, BRAKING_MILLIS );

    // Try rotate forward again
    elapsedTime = seekNextEdge ( HIGH, foreSpeed, baseStop, timeLimit, NULL );
  }
  timeOut = elapsedTime == INFINITE_TIME;
  if ( timeOut ) {
    elapsedTime = seekNextEdge ( HIGH, getBoostSpeed( foreSpeed, baseStop),
      baseStop, timeLimit, NULL );
  }
  // if ( elapsedTime == 0 ) { abortAndReport(); }

  // The encoder bar is not (any longer) blocking the sensor, and rotation stopped

  return timeOut;
}// ./bool findEncoderBarEdge (…)


/**
 * Build setting to rotation speed profile for servo
 *
 * NOTE for now, just store the time information for a common rotation angle.
 *   Do not even need to know the angle.  The relative time from step to step
 *   is what is useful.  Real world time, speed, distance can be calculated
 *   after the fact, from a few more accurate reference settings.
 *
 * @param zeroCw - highest (cw) servo setting known to not finish timing pass
 * @param zeroCcw - lowest (ccw) servo setting known to not finish timing pass
 */
#define CW_HARD_LIMIT 800
#define CCW_HARD_LIMIT 2200
#define PROFILE_STEP 20
void profileSpeed( const unsigned int zeroCw, const unsigned int zeroCcw )
{
  const int PROFILE_STEPS = 150;
  unsigned int fullStop = zeroCw + div2rounded( zeroCcw - zeroCw );
  unsigned int cwTest, ccwTest, cwDelta, ccwDelta;
  cwTest = zeroCw;
  ccwTest = zeroCcw;
  Serial.println (F( "cw setting, cw µs, ccw setting, ccw µs" ));
  // High resolution profiling for the main section where speed changes quickly
  for ( int i = 0; i < PROFILE_STEPS; i++ ) {
    cwTest--;
    ccwTest++;

    saveProfileTime( cwTest, ccwTest, fullStop );
  }// ./for ( int i = 0; i < PROFILE_STEPS; i++ )

  // Low resolution profiling for the 'long tail' of the curve, where speed
  // changes slowly
  do {
    // Stop stepping when a limit gets reached
    if ( cwTest > CW_HARD_LIMIT ) { // If clockwise (lower) limit not reached yet
      cwDelta = cwTest - CW_HARD_LIMIT; // Cacluate Deltas first, since
      ccwDelta = CCW_HARD_LIMIT - ccwTest; // ccw not updated yet
      cwTest -= PROFILE_STEP; // Decrease setting by standard step size
      if ( cwDelta > ccwDelta ) { // cw further from limit than ccw
        cwTest--; // increase cw step (same direction as base step)
      } else if ( cwDelta != ccwDelta ) { // Not identical
        cwTest++; // decrease cw step
      }// ./else if ( cwDelta != ccwDelta )
    }// ./if ( cwTest > CW_HARD_LIMIT )
    if ( ccwTest < CCW_HARD_LIMIT ) { // if CCW (upper) limit not reached yet
      ccwTest += PROFILE_STEP; // Increase setting by standard step size
      cwDelta = cwTest - CW_HARD_LIMIT; // Calculate Deltas last, since
      ccwDelta = CCW_HARD_LIMIT - ccwTest; // cw already updated
      if ( cwDelta > ccwDelta ) { // cw further from limit than ccw
        ccwTest--; // decrease ccw step (oppose direction from base step)
      } else if ( cwDelta != ccwDelta ) { // Not identical
        ccwTest++; // increase ccw step
      }// ./else if ( cwDelta != ccwDelta )
    }// ./if ( ccwTest < CCW_HARD_LIMIT )

    saveProfileTime( cwTest, ccwTest, fullStop );
  } while ( cwTest > CW_HARD_LIMIT || ccwTest < CCW_HARD_LIMIT );
  // NOTE loop will go one (partial) step past the limits
}// ./void profileSpeed(…)


/**
 * Measure and record times for the specified cw and ccw crossing of the
 * encoder bar past the sensor
 *
 * @param cwTest - clockwise servo setting to time
 * @param zeroCcw - counter clockwise servo setting to time
 * @param fullStop - servo setting for zero speed
 */
void saveProfileTime( const unsigned int cwTest, const unsigned int ccwTest, const unsigned int fullStop )
{
  unsigned long xingTimeCw, xingTimeCcw;

  xingTimeCw = timeBarCrossing( cwTest, fullStop );
  xingTimeCcw = timeBarCrossing( ccwTest, fullStop );

  // IDEA Could fill in a sparse / associative array structure instead of printing
  sprintf_P ( gBuf, PSTR ( "%u, %lu, %u, %lu" ),
    cwTest, xingTimeCw, ccwTest, xingTimeCcw );
  Serial.println ( gBuf );
}// ./void saveProfile%Time(…)


/**
 * Locate the center setting, that gives 'full stop' for the continuous turn sero
 *
 * Will do this by reversing directions accross the encoder 'bar', measuring the
 * times taken, and searching for the minimum from both ends (maximum ClockWise
 * and maximum counterclockwise)
 *
 * NOTE Since the steps size is not necessarily 'one', the search result Could
 *  return values that are not actually the first and last zero motions settings.
 *  It would skip over others that are also zero.
 *
 * @param cwZero - address of int to store first (lowest) zero motion setting
 * @param ccwZero - address of int to store last (hightest) zero motion setting
 */
void huntForZero ( unsigned int * cwZero, unsigned int * ccwZero )
{
  // Variables to hold target servo settings values
  unsigned int tentativeZeroPoint, cw, ccw;
  unsigned long cwTime = 0, ccwTime = 0;//, junkTime;

  cw = MAX_CW;   // Start at the full (maximum) range both clockwise and
  ccw = MAX_CCW; // counterclockwise.  (Approximately maximum speed for each)
  // Assume (to start) that full stop is mid way between full range CW and CCW
  tentativeZeroPoint = MAX_CW + div2rounded( MAX_CCW - MAX_CW );

  Serial.println (F( "searching..." ));
  // Find the encoder bar, and rotate it a bit past the sensor location
  timeBarCrossing( ccw, tentativeZeroPoint );
  // Need to find the falling edge of the sensor state transistions, then continue
  // for additional time.
  // HPD
  seekNextEdge( HIGH, ccw, 0, -1ul, NULL ); // Rotate encoder bar over sensor
  seekNextEdge( LOW, ccw, 0, -1ul, NULL ); // Rotate encode bar past sensor
  seekNextEdge( HIGH, ccw, tentativeZeroPoint, SETTLE_MILLIS, NULL );
  seekNextEdge( HIGH, ccw, 0, BRAKING_MILLIS, NULL );// Wait until stopped

  do {
    if ( cwTime == INFINITE_TIME ) {
      // Already know reached stop point, so shorten timing
      // junkTime =
      timeBarCrossing( cw + 8, tentativeZeroPoint );
    } else {
      cwTime = timeBarCrossing( cw, tentativeZeroPoint );
    }
    if ( ccwTime == INFINITE_TIME ) {
      // Already know reached stop point, so shorten timing
      // junkTime =
      timeBarCrossing( ccw - 8, tentativeZeroPoint );
    } else {
      ccwTime = timeBarCrossing( ccw, tentativeZeroPoint );
    }
    tentativeZeroPoint = estimateZeroSetting( cw, ccw, cwTime, ccwTime );
    rptZeroSearch( cw, ccw, cwTime, ccwTime, tentativeZeroPoint );

    if ( cwTime != INFINITE_TIME ) { // No adjust if timeout last time
      cw += divRounded( tentativeZeroPoint - cw, 3 ); // 1/3 way to zero estimate
      // cw += div2rounded( tentativeZeroPoint - cw ); // half way to zero estimate
    }
    if ( ccwTime != INFINITE_TIME ) { // No adjust if timeout last time
      ccw -= divRounded( ccw - tentativeZeroPoint, 3 );
      // ccw -= div2rounded( ccw - tentativeZeroPoint );
    }
  } while ( ccwTime != INFINITE_TIME || cwTime != INFINITE_TIME );// Still detecting motion
  // tentativeZeroPoint = estimateZeroSetting( cw, ccw, cwTime, ccwTime );
  // sprintf_P ( gBuf, PSTR ( "End Search@%u,%u@%u" ), cw, ccw, tentativeZeroPoint );
  // Serial.println ( gBuf );
  * cwZero = cw;
  * ccwZero = ccw;
}// ./void huntForZero(…)


/**
 * Time how long it takes the encoder bar to cross the sensor at the specified
 * speed setting
 *
 * @param timetarget - servo speed setting to use for timing
 * @param stopTarget - (best guess) speed setting for full stop
 * @returns crosing time in microsecnds (µs)
 */
unsigned long timeBarCrossing( const int timeTarget, const int stopTarget )
{
  // const unsigned long ACCEL_TIMEOUT = 5000000; // µs (5 seconds)
  // Extra speed to get past timeout, and to make sure slow cases move bar far
  // enough past sensor to get a good reading the reverse direction
  unsigned long startXsing, endXsing;//, baseRef;
  unsigned int accelSpeed;
  bool timeOut;

  accelSpeed = getBoostSpeed( timeTarget, stopTarget );
  timeOut = seekEdgeOfBar( timeTarget, stopTarget, & startXsing );
  // With a *good* stop value, and starting with sensor blocked, the following
  // waits forever
  while( digitalRead( sensePin ) == HIGH ) {} // Wait until sensor goes back LOW
  endXsing = micros();

  if ( !timeOut ) { // Speed up slightly while moving bar away from the sensor
    tstServo.writeMicroseconds( accelSpeed );
  }
  delay ( SETTLE_MILLIS ); // move past far enough to get 'to speed' on the way back

  tstServo.writeMicroseconds( stopTarget ); // Slow / stop before reversing
  delay ( BRAKING_MILLIS );

  if ( timeOut ) {
    return INFINITE_TIME; // Too slow to get a usable time reading
  }
  return endXsing - startXsing;
}// ./unsigned long timeBarCrossing(…)


/**
 * Rotate the servo until encoder bar transitions from not covering, to covering
 * the sensor.
 *
 * Find the leading (based on rotation direction) edge of the encoder bar
 *
 * NOTE On exit, the servo will still be rotating, either at the intially
 *  requested speed, or the faster adjusted speed, if a timeout occured
 *
 * @param seekSpeed - servo setting to use while looking for the edge of the bar
 * @param stopTarget - (best guess) speed setting for full stop
 * @param foundTime - address to store the timestamp encoder bar was encountered
 * @returns true if the edge of the bar was not detected before timeout
 */
bool seekEdgeOfBar( const unsigned int seekSpeed, const unsigned int stopTarget,
    unsigned long * foundTime )
{
  const unsigned long SEEK_TIMEOUT = 5000000; // µs (5 seconds)
  const unsigned long EXITBAR_TIMEOUT = 1000000; // µs (1 second)
  unsigned long seekStart, elapsedTime;
  unsigned int timeoutSpeed;
  bool timeOut = false;

  timeoutSpeed = getBoostSpeed( seekSpeed, stopTarget );

  seekStart = micros(); // For timeout testing
  tstServo.writeMicroseconds( seekSpeed );
  if( digitalRead( sensePin ) == HIGH ) { // Starting with sensor path blocked
    // Choices:
    //   1) rotate in the 'seekSpeed' direction to clear the bar before
    //      starting the core part of the seek
    //   2) rotate in the reverse direction to clear the bar, then contine far
    //      enough to have room to get back up to speed before encountering the
    //      the bar again.
    //        This *should* generally be faster, but that 'far enough' qualifier
    //        is not easy to determine.  The usual point that this code is
    //        called from does not yet have good information about the rotation
    //        speed, or whether the stopTarget is really stopped.
    // HPD alt code for refactor to ?common? seekNextEdge()

    // Wait until encoder bar uncovers sensor
    elapsedTime = seekNextEdge( HIGH, 0, 0, EXITBAR_TIMEOUT, & seekStart );
    if ( elapsedTime == INFINITE_TIME ) { // Timeout in intial seek
      // Continue at faster speed
      elapsedTime = seekNextEdge( HIGH, timeoutSpeed, 0, INFINITE_TIME, & seekStart );
    }
  }// ./if( digitalRead( sensePin ) == HIGH )

  // Servo is set to the requested rotation speed (or more if already timed
  // out), and the bar is (now) NOT covering the sensor.

  // Wait until encoder bar is over sensor
  seekStart = micros(); // For timeout testing
  elapsedTime = seekNextEdge( LOW, 0, 0, SEEK_TIMEOUT, & seekStart );
  timeOut = (elapsedTime == INFINITE_TIME);
  if ( timeOut ) {
    elapsedTime = seekNextEdge( LOW, timeoutSpeed, 0, SEEK_TIMEOUT, & seekStart );
  }
  if ( foundTime != NULL ) {
    * foundTime = seekStart; // Save timestamp when the edge of the bar was found
    // * foundTime = micros(); // Save timestamp when the edge of the bar was found
  }

  return timeOut; // Let caller know if a time out occurred while seeking
}// ./bool seekEdgeOfBar(…)


/**
 * Calculate a slightly faster, in the same direction, servo speed setting
 *
 * @param baseSpeed - 'in motion' servo speed setting (µs)
 * @param stopSpeed - servo speed setting (µs) for full stop (zero rotation)
 * @returns adjusted 'in motion' servo speed setting that is a little faster
 *   in the same direction as baseSpeed
 */
unsigned int getBoostSpeed( const unsigned int baseSpeed,
    const unsigned int stopSpeed )
{
  const unsigned int SPEEDUP = 8; // µs; enough setting delta to get moving

  if ( baseSpeed > stopSpeed ) { // Should be moving CCW
    return baseSpeed + SPEEDUP; // More CCW
  }// ./if ( seekSpeed > stopSpeed )

  return baseSpeed - SPEEDUP; // More CW
}// ./unsigned int getBoostSpeed(…)


/**
 * Calculate a new estimate for the full stop setting for the servo
 *
 * Base value is mid way between the cw and ccw setting values, but weighted
 * toward the longer time.
 *
 * @param cw - clockwise servo setting
 * @param ccw - counterclockwise servo setting
 * @param cwTime - time (micoseconds) for CW crossing
 * @param ccwTime - time (micoseconds) for CCW crossing
 */
unsigned int estimateZeroSetting( const unsigned int cw, const unsigned int ccw,
    const unsigned long cwTime, const unsigned long ccwTime )
{
  unsigned int settingRange, halfRange, midSetting, zeroEstimate;
  unsigned int midAdjust;//, midAdjust1;
  unsigned long wrkLarge, wrkSmall;//, wrk1, wrk2;

  settingRange = ccw - cw;
  halfRange = div2rounded( settingRange );
  midSetting = cw + halfRange;
  // sprintf_P ( gBuf, PSTR ( "mid:%u|%u=%u; tm:%lu/%lu; %s" ),
  //   ccw, cw, midSetting, max( cwTime, ccwTime ), min( cwTime, ccwTime ),
  //   (ccwTime > cwTime) ? "c" : "" ); // DEBUG
  // Serial.print ( gBuf ); // DEBUG

  if ( cwTime == ccwTime ) {
    // midAdjust1 = 0; // DEBUG
    // midAdjust = 0; // DEBUG
    // wrk1 = 0; // DEBUG
    // wrk2 = 0; // DEBUG
    zeroEstimate = midSetting;
  } else { // NOT ( cwTime == ccwTime )
    if ( cwTime > ccwTime ) {
      wrkLarge = cwTime;
      wrkSmall = ccwTime;
    } else {
      wrkLarge = ccwTime;
      wrkSmall = cwTime;
    }
    // wrk1 = halfRange * wrkSmall;
    // wrk2 = wrk1 / wrkLarge;
    // midAdjust1 = halfRange - wrk2;
    midAdjust = halfRange -( halfRange * wrkSmall )/ wrkLarge;

    if ( cwTime > ccwTime ) {
      zeroEstimate = max( cw, midSetting - midAdjust ); // prevent going outside current range
    } else {
      zeroEstimate = min( ccw, midSetting + midAdjust );
    }
  }// ./else NOT ( cwTime == ccwTime )
  // sprintf_P ( gBuf, PSTR ( "cw biggest; wrk:%lu,%lu; adj:%u,%u" ),
  //   wrk1, wrk2, midAdjust1, midAdjust, zeroEstimate ); // DEBUG
  // Serial.println ( gBuf ); // DEBUG

  return zeroEstimate;
}// ./unsigned int estimateZeroSetting(…)


void rptZeroSearch( unsigned int cw, unsigned int ccw,
    unsigned long cwTime, unsigned long ccwTime, unsigned int zero )
{
  sprintf_P ( gBuf,
    PSTR ( "Target range: %u to %u; Times %lu, %lu: new estimate = %u" ),
    cw, ccw, cwTime, ccwTime, zero );
  Serial.println ( gBuf );
}// void rptZeroSearch(…)


/**
 * Get rotation timings for different servo target settings
 *
 * @param minPulse - minimum servo target pulse time (µs)
 * @param maxPulse - maximum servo target pulse time (µs)
 * @param stepPulse - microseconds between servo target steps
 */
void timeRotations( const unsigned int minPulse, const unsigned int maxPulse, const unsigned int stepPulse )
{
  unsigned int pulseTime;

  Serial.println (F( "target, min, max, delta, total, average" ));
  for (pulseTime = minPulse; pulseTime <= maxPulse; pulseTime += stepPulse)
  {
    getSamples( pulseTime, -1 );
  }
}// ./void timeRotations(…)


/**
 * Collect and report a set of rotation timing samples for servo speed setting
 *
 * @param speedSetting - servo microseconds pulse width setting
 * @param refMicros - starting time stamp for first (in progress) rotation
 */
void getSamples( const unsigned int speedSetting, const unsigned long refMicros )
{
  unsigned long startTime, endTime;
  int rotNum; // Rotation number (in set)

  // Set the servo target (speed) for the sample set
  tstServo.writeMicroseconds( speedSetting );

  if (refMicros == -1u) {
    // No previous end time available, or do not want the error introduced by
    // the delay «de¦ac»celerating to the new setting
    startTime = syncRotation();
  } else {
    startTime = refMicros;

    // IDEA: good place to report previous timing information, but needs to have
    // the previous speedSetting available
  }

  for (rotNum = 0; rotNum < SAMPLES; rotNum++) {
    endTime = timeOneRotation();
    rotationTime[ rotNum ] = endTime - startTime;
    // rptRotationTimes( rotNum, startTime, endTime );
    startTime = endTime;
  }

  // reporting the timming information (here) increases the time (and error)
  // if want to got straight to another sample set
  rptTimingSet( speedSetting );
}// ./void getSamples(…)


/**
 * Get the micoseconds time stamp at the end of a rotation.
 *
 * End of a rotation is when the sense pin transitions from HIGH to LOW.
 *
 * To be called just after the previous sense transition from HIGH to LOW.
 *
 * @param startMicors - microseconds time stamp at the start of the rotation
 * @returns microseconds time stamp at the end of the rotation
 */
unsigned long timeOneRotation( void )
{
  unsigned long endMicros;

  // Sense pin should have just gone LOW, to mark the start of a rotation.
  digitalWrite( ledPin[ WAIT_MOST ], HIGH );

  // Wait for the sense pin to go HIGH
  while ( !digitalRead( sensePin )) {}

  // In the final stretch to time the current rotation
  digitalWrite( ledPin[ SENSE_EGHO ], HIGH );
  digitalWrite( ledPin[ WAIT_MOST ], LOW );
  digitalWrite( ledPin[ WAIT_END ], HIGH );

  // Wait for Sense to go LOW, to end the complete rotation
  while ( digitalRead( sensePin )) {}
  // Sense just went LOW, at the end of a rotation
  endMicros = micros();
  digitalWrite( ledPin[ SENSE_EGHO ], LOW );
  digitalWrite( ledPin[ WAIT_END ], LOW );

  return endMicros;
}// ./unsigned long timeOneRotation(…)


/**
 * Get the microseconds timestamp for the start of a rotation
 *
 * The start of a rotation occurs when the sense pin transitions from HIGH to LOW
 *
 * @returns microseconds time stamp at the start of the rotation
 */
unsigned long syncRotation( void )
{
  unsigned long startMicros;

  // Don't know what part of the rotation the system is when this is called, so
  // get the sense pin display in sync first.
  digitalWrite( ledPin[ SENSE_EGHO ], digitalRead( sensePin )); // Echo sense reading

  digitalWrite( ledPin[ TIMING_INIT ], HIGH );
  // Get into a consistent starting state: wait for sense to go HIGH (if not already)
  while ( !digitalRead( sensePin )) {}

  // Sense is now HIGH
  digitalWrite( ledPin[ TIMING_INIT ], LOW );
  digitalWrite( ledPin[ WAIT_BEGIN ], HIGH );
  digitalWrite( ledPin[ SENSE_EGHO ], HIGH );
  // Wait for sense to go LOW to mark the start point of a rotation
  while ( digitalRead( sensePin )) {}
  // Sense just went LOW, at the start of a rotation
  startMicros = micros();
  digitalWrite( ledPin[ SENSE_EGHO ], LOW );
  digitalWrite( ledPin[ WAIT_BEGIN ], LOW );

  return startMicros;
}// ./unsigned long syncRotation(…)


/**
 * Report (debug) information for the times in a single rotation sample
 *
 * @param idx - timing sample index number
 * @param stTm - rotation start time microseconds time stamp
 * @param enTm - rotation end time microseconds time stamp
 */
void rptRotationTimes( const int idx, const unsigned long stTm, const unsigned long enTm )
{
  sprintf_P ( gBuf, PSTR ( "Rotation %d: elapsed = %lu: start = %lu: end = %lu"),
    idx, enTm - stTm, stTm, enTm );
  Serial.println ( gBuf );
}// ./void rptRotationTimes(…)


/**
 * Analyze and report summary for the rotation timing measurements
 *
 * Format reported data as CSV, to allow easy loading to a spreadsheet.
 *
 * @param pulseTime - width (µs) of the target pulse (speed setting)
 */
void rptTimingSet( const unsigned int pulseSetting )
{
  unsigned long minElapsed, maxElapsed, totElapsed;

  // Collect the minimum, maximum and total elapsed times
  minElapsed = rotationTime[ 0 ];
  maxElapsed = rotationTime[ 0 ];
  totElapsed = rotationTime[ 0 ];
  for (int sample = 1; sample < SAMPLES; sample++) {
    if (rotationTime[ sample ] < minElapsed ) {
      minElapsed = rotationTime[ sample ];
    }
    if (rotationTime[ sample ] > maxElapsed ) {
      maxElapsed = rotationTime[ sample ];
    }
    totElapsed += rotationTime[ sample ];
  }
  sprintf_P ( gBuf, PSTR ( "%u, %u, %u, %u, %u, %u" ),
    pulseSetting, minElapsed, maxElapsed, minElapsed - maxElapsed, totElapsed,
    divRounded( totElapsed, SAMPLES ));
  Serial.println ( gBuf );
}// ./void rptTimingSet(…)



/**
 * Initial testing of the opto interrupter sensor
 *
 * Determine the ratio of the on time to the total time for a full rotation, so
 * that (later) it is possible to calculate servo rotation speeds with less than
 * a full roation (less than half a rotation).  This needs to be done
 * dynamically, because the interrupter sensor mounting and encoder are home
 * brew, and not consistent.  The ratio could easily vary from one run to the
 * next.
 */
void autoCalibrateSensor( void )
{
  unsigned long accumLow, accumHigh;
  // Pick a couple of microsecond values that should be in the main 'full speed'
  // range for normal servos.
  const int testSpeed[] = { 1350, 2000 };
  const int REPEAT_COUNT = 5;
  Serial.println (F( "Starting speed sensor automatic calibration" ));
  digitalWrite( ledPin[ AUTO_RUNNING ], HIGH );

  for ( int i = 0; i < PART_POINTS; i++ ) {
    accumLow = 0;
    accumHigh = 0;
    for ( int j = 0; j < REPEAT_COUNT; j++ ) {
      getRotationParts ( testSpeed[ i ], i );
      accumLow += rotationParts[ i ].low;
      accumHigh += rotationParts[ i ].high;
      // start DEBUG block
      tstServo.writeMicroseconds( TEST_STOP ); // Tentative STOP setting
      sprintf_P ( gBuf, PSTR ( "%lu,%lu@%u" ),
        rotationParts[ i ].low, rotationParts[ i ].high, testSpeed[ i ]);
      Serial.println ( gBuf );
      // end DEBUG block
    }
    rotationParts[ i ].low = ( accumLow * 2 + REPEAT_COUNT )/( 2 * REPEAT_COUNT );
    rotationParts[ i ].high = ( accumHigh * 2 + REPEAT_COUNT )/( 2 * REPEAT_COUNT );

    // start DEBUG block
    delay( 1000 ); // 1 second
    // end DEBUG block
  }

  Serial.println (F( "Have data for sensor calibration" )); // DEBUG
  for ( int i = 0; i < PART_POINTS; i++ ) {
    rotationParts[ i ].total = rotationParts[ i ].low + rotationParts[ i ].high;
    rotationParts[ i ].highFraction = (double) rotationParts[ i ].high / rotationParts[ i ].total;

    sprintf_P ( gBuf, PSTR ( "Speed: %u, low: %lu, high: %lu, total: %lu, high franction: " ),
      rotationParts[ i ].low, rotationParts[ i ].high );
    Serial.print ( gBuf );
    Serial.println ( rotationParts[ i ].highFraction, 5 ); // sprintf not handle float
  }
  digitalWrite( ledPin[ AUTO_RUNNING ], LOW );
}// ./void autoCalibrateSensor(…)


/**
 * Get low and high times for a single rotation, with specified speed setting
 *
 * @param speedSetting - µs target setting for the servo
 * @param idx - the slot (in rotationParts) to store the collected timing data
 */
void getRotationParts( const int speedSetting, const int idx )
{
  // conservative µs for servo to come up to full speed for setting
  const unsigned long stablizeTime = 100000; // 0.1 second
  int senseState;
  unsigned long refMicros, startHIGHmicros, startLOWmicros, endHIGHmicros, endLOWmicros;

  tstServo.writeMicroseconds( speedSetting );
  endLOWmicros = micros(); // arbitrary variable to track how long it takes to
  // get to an encoder boundary after the servo is up to speed.

  digitalWrite( ledPin[ AUTO_SEEK ], HIGH );
  do {
    senseState = digitalRead( sensePin );
    // dbgDispCnt( dbgCnt ); // DEBUG show binary value on LEDs

    // Wait for a sense state transition, to get to one of the encoder boundaries
    while( senseState == digitalRead( sensePin )) {} // until sensor state changes
    refMicros = micros(); // Time stamp for when the boundary was seen
  } while ( refMicros - endLOWmicros < stablizeTime );// until find boundary that
  // is ALSO at least stablizeTime after the servo target (speed) was set.
  digitalWrite( ledPin[ AUTO_SEEK ], LOW );

  // refMicros is the start time stamp for a state change / boundary crossing
  if ( senseState == LOW ) {
    startHIGHmicros = refMicros; // The detected change was from LOW to HIGH
    senseState = HIGH; // The sensor state was LOW, and is now HIGH
  } else {
    startLOWmicros = refMicros; // The detected change was from HIGH to LOW
    senseState = LOW; // The sensor state was HIGH, and is now LOW
  }

  digitalWrite( ledPin[ AUTO_WAIT_1 ], HIGH );
  // wait for the next state change / boundary
  while( senseState == digitalRead( sensePin )) {}
  refMicros = micros(); // Time stamp for when the boundary was seen
  digitalWrite( ledPin[ AUTO_WAIT_1 ], LOW );

  // refMicros is the end time stamp for the previous boundary, and the start
  // time stamp for the next one.
  if ( senseState == LOW ) {
    endLOWmicros = refMicros; // The detected change was from LOW to HIGH
    startHIGHmicros = refMicros;
    senseState = HIGH; // The sensor state was LOW, and is now HIGH
  } else {
    endHIGHmicros = refMicros; // The detected change was from HIGH to LOW
    startLOWmicros = refMicros;
    senseState = LOW; // The sensor state was HIGH, and is now LOW
  }

  digitalWrite( ledPin[ AUTO_WAIT_2 ], HIGH );
  // Wait for the final state change / boundary, for the end of a full rotation
  while( senseState == digitalRead( sensePin )) {}
  refMicros = micros(); // Time stamp for when the boundary was seen
  digitalWrite( ledPin[ AUTO_WAIT_2 ], LOW );

  // refMicros is the end time stamp for the final (full rotation) boundary
  if ( senseState == LOW ) {
    endLOWmicros = refMicros; // The detected change was from LOW to HIGH
    // senseState = HIGH; // The sensor state was LOW, and is now HIGH
  } else {
    endHIGHmicros = refMicros; // The detected change was from HIGH to LOW
    // senseState = LOW; // The sensor state was HIGH, and is now LOW
  }

  // Now have all the start and end times, to calculated partial and full
  // rotation times.
  rotationParts[ idx ].low = endLOWmicros - startLOWmicros;
  rotationParts[ idx ].high = endHIGHmicros - startHIGHmicros;
}// ./void getRotationParts(…)
void tstGetRotParts( void )
{
  const int tstIdx = 0;
  const unsigned int tstSetting[] = {1434,1500}; // slower speeds for visual state inspection
  const int rptCount = 5;

  for ( int i = 0; i < 2; i++ ) {
    for ( int j = 0; j < rptCount; j ++ ) {
      getRotationParts ( tstSetting[ i ], tstIdx );
      tstServo.writeMicroseconds( TEST_STOP ); // Tentative STOP setting
      sprintf_P ( gBuf, PSTR ( "%lu,%lu@%u" ),
        rotationParts[ tstIdx ].low, rotationParts[ tstIdx ].high, tstSetting[ i ]);
      Serial.println ( gBuf );
      delay( 5000 ); // 5 seconds
    }
  }
}// ./void tstGetRotParts(…)


// Servo C specific test
#define C_SLOW_CW 1462
// Servo C full stop range: 1463 … 1466
#define C_FULL_STOP 1466
// #define C_VERY_SLOW_CCW 1467 // VS_CCW 2 'ticks' in 15 seconds
// #define C_VERY_SLOW_CCW 1468 // VS_CCW 3 'ticks' in 15 seconds
// #define C_VERY_SLOW_CCW 1469
// #define C_VERY_SLOW_CCW 1470 // VS_CCW 5 'ticks' in 15 seconds
// #define C_VERY_SLOW_CCW 1471
#define C_SLOW_CCW 1472
// #define C_SLOW_CCW 1473 // faster than C_SLOW_CW
// C_SLOW_CW is faster than C_CLOW_CCW (about 2 x)
/**
 * Manual tunning to check the mid range slow, stop, slow for Servo C
 */
void tstCSlow( void )
{
  tstServo.writeMicroseconds( C_FULL_STOP ); // Tentative STOP setting
  digitalWrite( ledPin[ 12 ], HIGH );
  delay ( 1000 );
  digitalWrite( ledPin[ 12 ], LOW );
  digitalWrite( ledPin[ 1 ], HIGH );
  tstServo.writeMicroseconds( C_SLOW_CW );
  delay ( 15000 );
  digitalWrite( ledPin[ 1 ], LOW );
  digitalWrite( ledPin[ 11 ], HIGH );
  tstServo.writeMicroseconds( C_SLOW_CCW );
  delay ( 15000 );
  digitalWrite( ledPin[ 11 ], LOW );
}// ./void tstCSlow( void )


/**
 * Show the low (4) bits of an integer value as binary value on LEDs
 *
 * DEBUG
 *
 * @param val - value to display
 */
void dbgDispCnt( const unsigned int val )
{
  digitalWrite( ledPin[ BIT_0 ], (val & B0001) != 0 );
  digitalWrite( ledPin[ BIT_1 ], (val & B0010) != 0 );
  digitalWrite( ledPin[ BIT_2 ], (val & B0100) != 0 );
  digitalWrite( ledPin[ BIT_3 ], (val & B1000) != 0 );
}// ./void dbgDispCnt(…)
void tstDispCnt( void )
{
  digitalWrite( ledPin[ AUTO_RUNNING ], HIGH );
  for ( int v = 0; v < 16; v++ ) {
    dbgDispCnt( v );
    delay ( 500 ); // half second
  }
  digitalWrite( ledPin[ AUTO_RUNNING ], LOW );
  delay ( 2000 ); // 2 seconds
}// ./void tstDispCnt(…)


/**
 * Get and display the state of the opto interrupter
 *
 * Interrupter state goes HIGH when the optical path is blocked.  The LED is on
 * when the path is blocked.
 */
void senseEcho( void )
{
  digitalWrite( ledPin[ SENSE_EGHO ], digitalRead( sensePin )); // Echo sense reading
}// ./void senseEcho(…)


/**
 * Learn / verify which LED matches the pin numbers
 *
 * First pin is Blue, in the circle next to the 'odd' pin that is not in the circle
 * Following pins go clockwise around the circle
 *  yellow, green, red, blue, yellow, yellow, red, blue, green, green, red
 * Final pin is the LED that is not in the circle: green
 */
void ledTest( void )
{
  int i; // loop counter : led pin index

  // Make the first pin flicker to tell where the start of the pattern/sequence is
  for ( i = 0; i < 10; i++ ) {
    pulsePin( ledPin[0], 100, 100 );
  }

  // Blink the rest of the pins one at a time
  for ( i = 1; i < LEDS; i++ ) {
    pulsePin( ledPin[i], 500, 500 );
  }
}// ./void ledTest(…)


// Basic blink code, using blocking delay
void pulsePin( const int pin, const unsigned long onMillis, const unsigned long offMillis )
{
  digitalWrite( pin, HIGH );
  delay( onMillis );
  digitalWrite( pin, LOW );
  delay( offMillis );
}// ./void pulsePin(…)


/**
 * Calculate half of the supplied unsigned integer value, rounded
 *
 * @param val - unsigned value to divided by 2, and round
 * @reeturns round( val / 2 )
 */
inline unsigned int div2rounded( const unsigned int val )
{
  return divRounded( val, 2ul );
  // return ( val * 2 + 1 )/ 4;
  // return ( val + val | 1 )/ 2;
}// ./unsigned int div2rounded(…)


/**
 * Do integer divsion with rounding, without any floating point expressions
 *
 * NOTE This technique only works for unsigned values
 * NOTE Overflows if (numerator * 2 + denominator) > -1ul, or (denominator * 2) > -1ul
 *
 * @numerator - numerator of divsion expression
 * @denominator - denominator of divsion expression
 * @returns round( numerator / denominator )
 */
inline unsigned long divRounded( const unsigned long numerator, const unsigned long denominator )
{
  return ( numerator * 2 + denominator )/( denominator * 2 );
}// ./inline unsigned long divRounded(…)


void allLEDoff( void )
{
  for ( int i = 0; i < LEDS; i++ ) {
    digitalWrite( ledPin[ i ], LOW );
  }
}// ./void allLEDoff(…)


/**
 * Report information about the program / physical state that meant that the
 * code does not know a reasonable way of continuing.
 *
 * Use Serial print output for verbose information, but also use blink codes for
 * when no Serial connection might be available.
 *
 * NOTE This function should NEVER return
 *
 * NOTE May also use the global gBuf for extra input reporting information
 *
 * @param abortStatus - code for the error that trigger the profiling abort
 */
void abortAndReport( const unsigned int abortStatus )
{
  const unsigned long REPEAT_DELAY = 10000; // 10 seconds
  const unsigned long MINIMUM_SEP_DELAY = 3000; // 3 seconds

  char msgBuf[ 100 ]; // use local buffer, so global can be used to pass case
  unsigned long msgStartTime, msgWaitTime;

  digitalWrite( ledPin[ ABORT_BLINK ], LOW );

  // Create the message for the abort case one time
  switch ( abortStatus ) {
    case ABRT_ZERO_INVERTED:
      sprintf_P ( msgBuf, PSTR ( "Starting servo zero motion range %s is inverted" ), gBuf );
      break;
    case ABRT_ZERO_LARGE:
      sprintf_P ( msgBuf, PSTR ( "Starting servo zero motion range %s is too large" ), gBuf );
      break;
    case ABRT_COARSE_EARLY:
      sprintf_P ( msgBuf, PSTR ( "sensor blocked in FineTuneZero CCW check before timing started" ));
      break;
    case ABRT_COARSE_SPLIT:
      sprintf_P ( msgBuf, PSTR ( "sensor blocked in FineTuneZero CCW check before end timer" ));
      break;
    case ABRT_FINE_START_UNCOVERED:
      sprintf_P ( msgBuf, PSTR ( "Sensor not covered when starting fineZeroBoundary" ));
      break;
    case ABRT_STOP_AFTER_EDGE:
      sprintf_P ( msgBuf, PSTR ( "Did not stop fast enough: Sensor changed from %s" ), gBuf );
      break;
    case ABRT_FINE_MISSED_BAR:
      sprintf_P ( msgBuf, PSTR ( "Did not locate encoder bar with setting %s" ), gBuf );
      break;
    default:
      sprintf_P ( msgBuf, PSTR ( "Unhandled abort code %u encountered" ), abortStatus );
      break;
  }// ./select ( abortStatus )


  // Make sure the LED used to blink the abort code has been off long enough to
  // not confuse the first blink pattern
  delay ( MINIMUM_SEP_DELAY );

  // Keep repeating the abort message information, forever
  while ( true ) { // do forever
    msgStartTime = millis();

    Serial.println ( msgBuf );
    blinkCode ( abortStatus );

    msgWaitTime = millis();
    if ( msgWaitTime < msgStartTime ) {
      // Been sitting in abortAndReport a LONG time: millis wrapped past zero
      // Fudge the time used for reporting
      msgStartTime = 0;
      msgWaitTime = MINIMUM_SEP_DELAY;
    }

    // do nothing for the remainder of the message cycle (REPEAT_DELAY) time
    delay ( max( MINIMUM_SEP_DELAY,
      REPEAT_DELAY - ( msgWaitTime - msgStartTime )));
  }// .while ( true )
}// ./void abortAndReport(…)


/**
 * Blink and LED to display a positive integer value
 *
 * Currently this blinks the LED as many times as the value.  If the values
 * end up getting larger, that will need to get smarter.  Perhaps longer blinks
 * for tens (or second hex) digit.  Or prefix with longer blink(s) to show
 * digit position?
 *
 * @param val - value to blink
 */
void blinkCode ( const unsigned int val )
{
  const unsigned long BLINK_ON = 20;
  const unsigned long BLINK_OFF = 250;
  const unsigned long BLINK_SEP = 500;
  const unsigned int BLINK_GROUP = 5;
  unsigned int blinkCount = 0;

  // TODO blink abortStatus : as decimal? hex?, sequence?, mixed?

  // blink out the value as a series of groups, to make counting easier
  do {
    digitalWrite( ledPin[ ABORT_BLINK ], HIGH );
    delay ( BLINK_ON );
    digitalWrite( ledPin[ ABORT_BLINK ], LOW );
    blinkCount++;
    if ( blinkCount >= val ) { break; } // All done

    // Insert delay before next blink
    delay ( BLINK_OFF );
    if (( blinkCount % BLINK_GROUP ) == 0 ) { // at boundary between groups
      // Insert delay before next group / block
      delay ( BLINK_SEP );
    }// ./if (( blinkCount % BLINK_GROUP ) == 0 )
  } while ( true );
}// ./void blinkCode (…)
