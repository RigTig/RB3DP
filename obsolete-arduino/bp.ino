//------------------------------------------------------------------------------
// 3D drawing/printing robot - Supports 28BYJ-48
// rigtig181@gmail.com 2015feb22-2015mar22
// based on drawing/printing robot
//  rigtig181@gmail.com 2013dec13 - 2014jan29
//  which was inspired by dan@marginallyclever.com
//------------------------------------------------------------------------------

// configuration
// for debugging, uncomment the following line
#define VERBOSE      (-1)

// Drawing Machine Unit ID
int drawingMachineUID=3; //change by= UID n;

//MOTORS
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 64. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4.  
// Blue   - 28BYJ48 wire 1
// Pink   - 28BYJ48 wire 2
// Yellow - 28BYJ48 wire 3
// Orange - 28BYJ48 wire 4
// Red    - 28BYJ48 wire 5 (VCC)
// Each motor is connected to a ULN2003 driver
// 4 motors, each with 4 driven pins
int motorPin[4][4]= {{8,9,10,11},{4,5,6,7},{A4,A5,A2,A3},{2,3,12,13}}; // connect wires 1,2,3,4

// The step angle is 45/64 degrees
#define STEPS_PER_REV  (512.0)
// choose to use 4 microsteps per step
// sequence of microsteps (one bit per wire)
int microsteps[4] = {B00001, B00010, B00100, B01000};
#define CLOCKWISE            (1)
#define ANTICLOCKWISE        (-1)


// No-load pull-in frequency: > 500 or 600 Hz (depends on mfr)
// No-load pull-out frequency: > 900 or 1000 Hz (depends on mfr)
// if can reliably microstep at 1000 Hz, then maximum rpm 
// = 1000 microstep/sec x 60 sec/min / 512 step/revolution / 4 microsteps/step = 29.2 rev/min
#define MIN_MICROSTEPPERIOD  (1000)   // microseconds = 1000Hz
#define MAX_MICROSTEPPERIOD  (1000000)   // microseconds =1Hz

static long microstepPeriod; // microseconds per microstep

// If motor shaft is 65mm diameter,
//  then it is 204 mm circumference, and
//  204 / 512 = .399 mm string movement per step
//  ~= 0.05 mm string moved each microstep,
//  At 1000Hz, speed is ~ 50 mm/sec
//   equivalent to 3000 millimetres per minute
//   equivalent to 20 seconds per meter
//   = 20 mSec/mm = 20,000 uSec/mm

// same calibration for each motor
//long um_per_step = 399; // assuming shaft diameter is 65 mm
long um_per_step = 152; // assuming shaft diameter is 24.77 mm

// extruder factor from cm to steps
//  m8 bolt standard pitch is 1.25mm, so 8 turns is 1cm.
//  at 512 steps per turn, the number of steps per cm is 4096.
//  nut is on 50 tooth cog driven by 10 tooth cog, so 
//  factor is 4096*50/17
static float e2steps = 20475.0; // E is filament length

// switch sensitivity
#define SWITCH_HALF     (512)


// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
// 2do: use Bresenham's arc algorithm instead
#define CM_PER_SEGMENT   (0.2)

// Serial communication bitrate
#define BAUD            (115200)
// Maximum length of serial input message.
#define MAX_BUF         (64)

#define File int

//------------------------------------------------------------------------------
// EEPROM MEMORY MAP
//------------------------------------------------------------------------------
#define EEPROM_VERSION   1             // Increment EEPROM_VERSION when adding new variables
#define ADDR_VERSION     0             // address of the version number (one byte)
#define ADDR_UUID        1             // address of the UUID (long - 4 bytes)
#define ADDR_STEPLEN     5             // address of the step length for all motors (long)


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

// Saving config
#include <EEPROM.h>
#include <Arduino.h>  // for type definitions


//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------
// string supports must be on a horizontal plane (i.e. at same height above a horizontal ground)
// calibration and length calculations are based on
// - the three support points are called S1, S2 and S3, in an anticlockwise direction
// - S1 has coordinates of [0,0,0] = machine origin
// - S2 = [x2,0,0], so the s1 to S2 line is the X axis
// - S3 = [x3,y3,0]
static float x2 = 300.0; //cm, or 3000mm
static float x3 = 150.0;
static float y3 = 300.0;

static float zmax = 280.0; // height from lowest horizontal surface to plane of string supports  

// what are the motors called? (anticlockwise on the ground = clockwise looking up)
int m1=0;
char m1d='A';
int m2=1;
char m2d='B';
int m3=2;
char m3d='C';
// last motor is for extruder
int m4=3;
char m4d='E'; 


// which way are the shafts wound, relative to motor movement?
int M1_REEL_IN  = ANTICLOCKWISE;
int M1_REEL_OUT = CLOCKWISE;
int M2_REEL_IN  = ANTICLOCKWISE;
int M2_REEL_OUT = CLOCKWISE;
int M3_REEL_IN  = ANTICLOCKWISE;
int M3_REEL_OUT = CLOCKWISE;
int M4_REEL_IN  = CLOCKWISE;
int M4_REEL_OUT = ANTICLOCKWISE;

// motor positions (saves a call to IK every line() )
static long m1Pos_steps, m2Pos_steps, m3Pos_steps, m4Pos_steps;

// where is object origin relative to machine origin (NB z increases down) [cm]
static float objx = 150.0; // say, the middle
static float objy = 150.0; // say, middle
static float objz = zmax;  // say, at lowest level

// effector position, relative to object origin (NB z increases up) [cm]
static float posx = 0.0;
static float posy = 0.0;
static float posz = 0.0;
// extruder
static float pose = 0.0; 

static char absolute_mode=1;  // absolute (=1) or incremental (=0) programming mode?
static char inputUnit_Name[3]; // mm or inches
static float cmPerUnit; 

// Serial comm reception
static char buffer[MAX_BUF];  // Serial buffer
static int sofar;             // Serial buffer progress


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
static long stringMovement(float diameter) {
  // input is diameter in cm
  // return is micrometers per step
  return diameter * PI * 10000.0 / STEPS_PER_REV; // cm/rev * 10000 um/cm / step/rev
}

//------------------------------------------------------------------------------
// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


//------------------------------------------------------------------------------
static char readSwitches() {
  return 0; // assume no switches triggerred
  // 2do: fit switches to detect string shortened to minimum
}


//------------------------------------------------------------------------------
static long feedrate2microseconds(float v) {
  // input is feedrate in units/min
  // returns microstep delay in microseconds
  float factor = 1500.0 / (v * cmPerUnit); 
  // (100 cm/m * 60 sec/min / 4 microsteps/step) / (unit/min * cm/unit)
  
  long microstepDelay = factor * um_per_step;
  microstepDelay = constrain(microstepDelay, MIN_MICROSTEPPERIOD, MAX_MICROSTEPPERIOD);

#ifdef VERBOSE  
  Serial.print(F("microstep pause time (uS)="));
  Serial.println(microstepDelay);
#endif
  return microstepDelay;
}


//------------------------------------------------------------------------------
static void printFeedRate() {
  Serial.print(1500.0 * um_per_step / (microstepPeriod * cmPerUnit));
  // (sec/min * cm/m / microsteps/step) * um/step / (usec/microstep * cm/unit)= unit/min 
  Serial.print(inputUnit_Name);
  Serial.print(F("/min"));
}


//------------------------------------------------------------------------------
void onestep(int motor,int dirn) {
//  if(motor==0) Serial.print('A');
//  else if (motor==1) Serial.print('B');
//  else Serial.print('C');
//  Serial.print(dirn);
  int i;
  if(dirn==CLOCKWISE) {
    for(i = 3; i >= 0; i--)
    {
      outputMicrostep(motor, i);
    }
  } else { // ANTICLOCKWISE
    for(int i = 0; i < 4; i++)
    {
      outputMicrostep(motor, i);
    }
  }
}

void outputMicrostep(int motor, int out) {
  int mstep = microsteps[out];
  for (int p=0; p<4; p++) {
//    Serial.print(motorPin[motor][p]);
//    Serial.print("=");
//    Serial.println(bitRead(mstep, p));
    digitalWrite(motorPin[motor][p], bitRead(mstep, p));
    delayMicroseconds(microstepPeriod);  // determines speed
  }
}


void motorPinOutput(int motor) {
  //set the motor pins as outputs
#ifdef VERBOSE
  Serial.print("pins for motor ");
  Serial.print(motor);
  Serial.print(" are ");
#endif
  for(int p=0; p<4; p++) {
#ifdef VERBOSE
    Serial.print(motorPin[motor][p]);
    Serial.print(", ");
#endif
    pinMode(motorPin[motor][p], OUTPUT);
  }
#ifdef VERBOSE
  Serial.println();
#endif
}


void release() {
  for (int motor = 2; motor >=0; motor--) {
    //set the motor pins as inputs
    pinMode(motorPin[motor][0], INPUT);
    pinMode(motorPin[motor][1], INPUT);
    pinMode(motorPin[motor][2], INPUT);
    pinMode(motorPin[motor][3], INPUT);
  }
}

//------------------------------------------------------------------------------
// Inverse Kinematics - turns XYZ object coordinates into thread lengths l1,l2 & l3 in steps
static void IK(float x, float y, float z, long &l1, long &l2, long &l3) {
#ifdef VERBOSE
  Serial.print(F("IK [")); 
  Serial.print(x); Serial.print(",");
  Serial.print(y); Serial.print(",");
  Serial.print(z); Serial.println("]");
#endif
  // translate object coordinates into machine coordinates
  float mx = x + objx;
  float my = y + objy;
  float mz = objz - z;
  
  // find string lengths in steps, i.e. cm * 10000 um/cm / um/step
  float mzmz = mz*mz;
  l1 = floor( sqrt(mx*mx + my*my + mzmz) * 10000.0 / um_per_step ); 
  l2 = floor( sqrt((mx-x2)*(mx-x2) + my*my + mzmz) * 10000.0 / um_per_step ); 
  l3 = floor( sqrt((mx-x3)*(mx-x3) + (my-y3)*(my-y3) + mzmz) * 10000.0 / um_per_step ); 
#ifdef VERBOSE
  Serial.print(m1d); Serial.println(l1);
  Serial.print(m2d); Serial.println(l2);
  Serial.print(m3d); Serial.println(l3);
#endif
}


//------------------------------------------------------------------------------
// Forward Kinematics - turns lengths of string (in steps) into XYZ object coordinates
static void FK(long l1, long l2, long l3, float &x, float &y, float &z) {
#ifdef VERBOSE
  Serial.println(F("FK")); 
  Serial.print(m1d); Serial.println(l1);
  Serial.print(m2d); Serial.println(l2);
  Serial.print(m3d); Serial.println(l3);
#endif
  float a = l1 * um_per_step * 0.0001; // step * um/step * 0.0001 cm/um
  float b = l2 * um_per_step * 0.0001;
  float c = l3 * um_per_step * 0.0001; // step * um/step * 0.0001 cm/um

#ifdef VERBOSE
  Serial.print(a); Serial.print(":");
  Serial.print(b); Serial.print(":");
  Serial.println(c);
#endif
  
  x = (a*a - b*b + x2*x2)/(2*x2);
  y = (a*a - c*c + x3*x3 - 2*x*x3 + y3*y3)/(2*y3);
  z = sqrt(a*a - x*x - y*y);
  x -= objx;
  y -= objy;
  z = objz -z;
  
  // in object space
#ifdef VERBOSE
  Serial.print(F("[")); 
  Serial.print(x); Serial.print(",");
  Serial.print(y); Serial.print(",");
  Serial.print(z); Serial.println("]");
#endif
  
}


//------------------------------------------------------------------------------
static void line(float x,float y,float z, float e) {
  long l1,l2,l3,l4;
  IK(x,y,z,l1,l2,l3);
  l4 = e * e2steps;
  // in machine space
  long d1 = l1 - m1Pos_steps;
  long d2 = l2 - m2Pos_steps;
  long d3 = l3 - m3Pos_steps;
  long d4 = l4 - m4Pos_steps;

  long ad1=abs(d1);
  long ad2=abs(d2);
  long ad3=abs(d3);
  long ad4=abs(d4);

  long ad1x2=ad1*2;
  long ad2x2=ad2*2;
  long ad3x2=ad3*2;
  long ad4x2=ad4*2;
  
  int dir1=d1<0?M1_REEL_IN:M1_REEL_OUT;
  int dir2=d2<0?M2_REEL_IN:M2_REEL_OUT;
  int dir3=d3<0?M3_REEL_IN:M3_REEL_OUT;
  int dir4=d4<0?M4_REEL_IN:M4_REEL_OUT;

  long i, err1, err2, err3;
  
  if((ad1>=ad2) && (ad1>=ad3) && (ad1>=ad4)) {
    err1=ad2x2 - ad1;
    err2=ad3x2 - ad1;
    err3=ad4x2 - ad1;
    for(i=0; i<ad1-1; i++) {
      if(err1>0) {
        onestep(m2,dir2);
        err1 -= ad1x2;
      }
      if(err2>0) {
        onestep(m3,dir3);
        err2 -= ad1x2;
      }
      if(err3>0) {
        onestep(m4,dir4);
        err3 -= ad1x2;
      }
      err1 += ad2x2;
      err2 += ad3x2;
      err3 += ad4x2;
      onestep(m1,dir1);
    }  
    if(readSwitches()) return;
  }

  if((ad2>=ad1) && (ad2>=ad3) && (ad2>=ad4)) {
    err1=ad1x2 - ad2;
    err2=ad3x2 - ad2;
    err3=ad4x2 - ad2;
    for(i=0; i<ad2-1; i++) {
      if(err1>0) {
        onestep(m1,dir1);
        err1 -= ad2x2;
      }
      if(err2>0) {
        onestep(m3,dir3);
        err2 -= ad2x2;
      }
      if(err3>0) {
        onestep(m4,dir4);
        err3 -= ad2x2;
      }
      err1 += ad1x2;
      err2 += ad3x2;
      err3 += ad4x2;
      onestep(m2,dir2);
    }  
    if(readSwitches()) return;
  }

  if((ad3>=ad1) && (ad3>=ad2) && (ad3>=ad4)) {
    err1=ad1x2 - ad3;
    err2=ad2x2 - ad3;
    err3=ad4x2 - ad3;
    for(i=0; i<ad3-1; i++) { 
      if(err1>0) {
        onestep(m1,dir1);
        err1 -= ad3x2;
      }
      if(err2>0) {
        onestep(m2,dir2);
        err2 -= ad3x2;
      }
      if(err3>0) {
        onestep(m4,dir4);
        err3 -= ad3x2;
      }
      err1 += ad1x2;
      err2 += ad2x2;
      err3 += ad4x2;
      onestep(m3,dir3);
    }  
    if(readSwitches()) return;
  }

  if((ad4>=ad1) && (ad4>=ad2) && (ad4>=ad3)) {
    err1=ad1x2 - ad4;
    err2=ad2x2 - ad4;
    err3=ad3x2 - ad4;
    for(i=0; i<ad4-1; i++) { 
      if(err1>0) {
        onestep(m1,dir1);
        err1 -= ad4x2;
      }
      if(err2>0) {
        onestep(m2,dir2);
        err2 -= ad4x2;
      }
      if(err3>0) {
        onestep(m3,dir3);
        err3 -= ad4x2;
      }
      err1 += ad1x2;
      err2 += ad2x2;
      err3 += ad3x2;
      onestep(m4,dir4);
    }  
    if(readSwitches()) return;
  }

  m1Pos_steps=l1;
  m2Pos_steps=l2;
  m3Pos_steps=l3;
  m4Pos_steps=l4;
  
  // in object space
  posx=x;
  posy=y;
  posz=z;
  pose=e;
}


//------------------------------------------------------------------------------
// This method assumes the limits have already been checked.
// 2do: limits not checked!!
// This method assumes the start and end radius match.
// 2do: change method to Bresenham's arc algorithm
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
static void arc(float cx,float cy,float x,float y,float z,float dir,float e) {
  // get radius
  float dx = posx - cx;
  float dy = posy - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = floor( len / CM_PER_SEGMENT );
 
  float nx, ny, nz, ne, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    nz = ( z - posz ) * scale + posz;
    ne = pose + (e - pose) * scale;
    // send it to the planner
    line(nx,ny,nz,ne);
  }
  
  line(x,y,z,e);
}


//------------------------------------------------------------------------------
// instantly move the virtual plotter position in object space
// does not validate if the move is valid
static void teleport(float x,float y, float z, float e) {
  // re-map object origin 
  objx += x - posx;
  posx = x;
  objy += y - posy;
  posy = y;
  objz += z - posz;
  posz = z;
  // set extruder in both object space and machine space
  pose = e;
  m4Pos_steps = pose * e2steps;
}


//------------------------------------------------------------------------------
static void help() {
  Serial.println(F("++ rigtig's Big 3D Printer ++"));
  Serial.println(F("HELP;  - display this message"));
  Serial.println(F("CONFIG [Ax.x] [Bx.x] [Cx.x] [Px.x] [Qx.x] [Rx.x] [H[-]1] [I[-]1] [J[-]1] [K[-]1];"));
  Serial.println(F(" - set current physical position as logical origin"));
  Serial.println(F(" - can set distances between supports, string lengths, plus motor directions."));
  Serial.println(F("UID n; - set the identifier used to distinguish printers."));
  Serial.println(F("D00 [An] [Bn] [Cn]; - move motor a number of steps (+/-)."));
  Serial.println(F("D01 [An] [Bn] [Cn]; - adjust shaft diameter (all the same)."));
  Serial.println(F("D02 ; - show shaft diameter."));
  Serial.println(F("As well as the following G-codes (http://en.wikipedia.org/wiki/G-code):"));
  Serial.println(F("G00,G01,G02,G03,G04,G20,G21,G28,G90,G91,G92,M17,M18,M114"));
}


//------------------------------------------------------------------------------
// find limit switches, set home position for pen and go to it.
static void FindHome() {
#ifdef USE_LIMIT_SWITCH
  Serial.println(F("Homing..."));
  
  if(readSwitches()) {
    Serial.println(F("** ERROR **"));
    Serial.println(F("Problem: Swing is already touching switches."));
    Serial.println(F("Solution: Please unwind the strings a bit and try again."));
    return;
  }
  
  microstepPeriod=MIN_MICROSTEPPERIOD; // as fast as possible
  // 2do=reset speed at end of homing
  int safe_out=50;
  
  // reel in the first motor until contact is made.
  //2do: reel out only as far as the length from this motor to each of the other two
  Serial.println(F("Find first..."));
  do {
    onestep(m1,M1_REEL_IN );
    onestep(m2,M2_REEL_OUT);
    onestep(m3,M3_REEL_OUT);
  } while(!readSwitches());
  m1Pos_steps=326; //2do: set to offset from pivot to switch. Est=13cm
  
  // back off so we don't get a false positive on the next motor
  int i;
  for(i=0;i<safe_out;++i) {
    onestep(m1,M1_REEL_OUT);
  }
  m1Pos_steps+=safe_out;
  
  // reel in the second motor until contact is made
  //2do: reel out only as far as the length from this motor to each of the other two
  Serial.println(F("Find second..."));
  do {
    onestep(m1,M1_REEL_OUT);
    onestep(m2,M2_REEL_IN );
    onestep(m3,M3_REEL_OUT);
   m1Pos_steps++;
  } while(!readSwitches());
  m2Pos_steps=326; //2do: set to offset from pivot to switch
  
  // back off so we don't get a false positive on the next motor
  for(i=0;i<safe_out;++i) {
    m2->step(1,M2_REEL_OUT);
  }
  m2Pos_steps+=safe_out;
  
  // reel in the third motor until contact is made
  //2do: reel out only as far as the length from this motor to each of the other two
  Serial.println(F("Find third..."));
  do {
    onestep(m1,M1_REEL_OUT);
    onestep(m2,M2_REEL_OUT);
    onestep(m3,M3_REEL_IN);
   m1Pos_steps++;
   m2Pos_steps++;
  } while(!readSwitches());
  m3Pos_steps=326; //2do: set to offset from pivot to switch
  
  // back off so we don't get a false positive that kills line()
  for(i=0;i<safe_out;++i) {
    m3->step(1,M3_REEL_OUT);
  }
  m3Pos_steps+=safe_out;
  
  Serial.println(F("Centering..."));
  line(0,0,0,0);
#endif // USE_LIMIT_SWITCH
}


//------------------------------------------------------------------------------
static void where() {
  Serial.print(F("Object using ")); Serial.print(inputUnit_Name);
  Serial.print(F(": X")); Serial.print(posx/cmPerUnit);
  Serial.print(F(" Y")); Serial.print(posy/cmPerUnit);
  Serial.print(F(" Z")); Serial.print(posz/cmPerUnit);
  Serial.print(F(" E")); Serial.println(pose/cmPerUnit);
  Serial.print(F(" F")); printFeedRate(); Serial.print(F("=")); 
   Serial.print(microstepPeriod); Serial.println(F(" microseconds per microstep"));
  Serial.print(F("Offsets in cm from machine origin:")); 
   Serial.print(objx); Serial.print(F(","));
   Serial.print(objy); Serial.print(F(","));
   Serial.println(objz); 
}


//------------------------------------------------------------------------------
static void printConfig() {
  Serial.print(F("Using ")); Serial.println(inputUnit_Name);
  Serial.println(F("S1=[0,0,0]"));
  Serial.print(F("S2=[")); Serial.print(x2/cmPerUnit); Serial.println(F(",0,0]")); 
  Serial.print(F("S3=[")); Serial.print(x3/cmPerUnit); Serial.print(F(",")); 
   Serial.print(y3/cmPerUnit); Serial.println(F(",0]")); 
  Serial.print(e2steps*cmPerUnit); Serial.print(F(" extruder steps per "));
   Serial.println(inputUnit_Name); 
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
void EEPROM_writeLong(int ee, long value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  EEPROM.write(ee++, *p++);
}


//------------------------------------------------------------------------------
// from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1234477290/3
float EEPROM_readLong(int ee) {
  long value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  *p++ = EEPROM.read(ee++);
  return value;
}


//------------------------------------------------------------------------------
static void LoadConfig() {
  char version_eeprom=EEPROM.read(ADDR_VERSION);
  if(version_eeprom<1 || version_eeprom>EEPROM_VERSION) {
    // If not the current EEPROM_VERSION or the EEPROM_VERSION is sullied (i.e. unknown data)
    // Update the version number
    EEPROM.write(ADDR_VERSION,EEPROM_VERSION);
    // Update robot uuid
    //drawingMachineUID=2;
    SaveUID();
    // Update shaft diameter variables
    saveStepLen();
  }
  if(version_eeprom==1) {
    // Retrieve superseded configuration details
    // none earlier, yet
    // update the EEPROM version
    EEPROM.write(ADDR_VERSION,EEPROM_VERSION);
  } else if(version_eeprom==EEPROM_VERSION) {
    // Retrieve Stored Configuration
    drawingMachineUID=EEPROM_readLong(ADDR_UUID);
    um_per_step=EEPROM_readLong(ADDR_STEPLEN);
  } else {
    // Code should not get here if it does we should display some meaningful error message
    Serial.println(F("An Error Occurred during LoadConfig"));
  }
}


//------------------------------------------------------------------------------
static void SaveUID() {
  EEPROM_writeLong(ADDR_UUID,drawingMachineUID);
}

//------------------------------------------------------------------------------
static void saveStepLen() {
  EEPROM_writeLong(ADDR_STEPLEN, um_per_step);
}

static void checkNewStepLen(long newStepLen) {
    if(um_per_step != newStepLen) { 
      um_per_step = newStepLen;
      EEPROM_writeLong(ADDR_STEPLEN, um_per_step);
    }
}
 





//------------------------------------------------------------------------------
static int processSubcommand() {
  int found=0;
  
  char *ptr=buffer;
  while(ptr && ptr<buffer+sofar && strlen(ptr)) {
    if(!strncmp(ptr,"G20",3)) {
      cmPerUnit=2.54f;  // inches -> cm
      strcpy(inputUnit_Name,"in");
      Serial.print(F("F")); printFeedRate();
      found=1;
    } else if(!strncmp(ptr,"G21",3)) {
      cmPerUnit=0.1;  // mm -> cm
      strcpy(inputUnit_Name,"mm");
      Serial.print(F("F")); printFeedRate();
      found=1;
    } else if(!strncmp(ptr,"G90",3)) {
      // absolute mode
      absolute_mode=1;
      found=1;
    } else if(!strncmp(ptr,"G91",3)) {
      // relative mode
      absolute_mode=0;
      found=1;
    }
    ptr=strchr(ptr,' ')+1;
  }
  
  return found;
}


//------------------------------------------------------------------------------
static void processCommand() {
  // temporary variables needed since min() and max() cannot have calculated parameters
  float limit_lo, limit_hi;

  // blank lines
  if(buffer[0]==0) return;

// HELP  
  if(!strncmp(buffer,"HELP",4)) {
    help();
    
// UID    
  } else if(!strncmp(buffer,"UID",3)) {
    drawingMachineUID=atoi(strchr(buffer,' ')+1);
    SaveUID();
    
// G28    
  } else if(!strncmp(buffer,"G28",3)) {
    FindHome();

// G92    
  } else if(!strncmp(buffer,"G92",3)) {
    float xx=posx;
    float yy=posy;
    float zz=posz;
    float ee=pose;
  
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': xx=atof(ptr+1)*cmPerUnit;  break;
      case 'Y': yy=atof(ptr+1)*cmPerUnit;  break;
      case 'Z': zz=atof(ptr+1)*cmPerUnit;  break;
      case 'E': ee=atof(ptr+1)*cmPerUnit;  break;
      default: ptr=0; break;
      }
    }
    
    teleport(xx,yy,zz,ee);

// M114    
  } else if(!strncmp(buffer,"M114",4)) {
    where();
    
// M17
  } else if(!strncmp(buffer,"M17",3)) {
    // enable motors
  motorPinOutput(0); 
  motorPinOutput(1); 
  motorPinOutput(2); 
  motorPinOutput(3); 
    
// M18
  } else if(!strncmp(buffer,"M18",3)) {
    // disable motors
    release();
    
// CONFIG
  } else if(!strncmp(buffer,"CONFIG",6)) {
    float l12=x2;
    float l23=sqrt(y3*y3 + (x2-x3)*(x2-x3));
    float l13=sqrt(x3*x3 +y3*y3);
    float h=zmax;
    long l1,l2,l3;
    IK(posx, posy, posz, l1, l2, l3);
#ifdef VERBOSE
    printConfig();
    Serial.print("l12=P="); Serial.print(l12/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l23=Q="); Serial.print(l23/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l13=R="); Serial.print(l13/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l1=A="); Serial.print(l1*0.0001*um_per_step/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l2=B="); Serial.print(l2*0.0001*um_per_step/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l3=C="); Serial.print(l3*0.0001*um_per_step/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("h=H="); Serial.print(h/cmPerUnit); Serial.println(inputUnit_Name);
#endif
    
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      // current lengths of strings in steps (id is same as motor)
      case 'A': l1=atof(ptr+1)*cmPerUnit*10000/um_per_step; l1=max(l1, 1);  break;
      case 'B': l2=atof(ptr+1)*cmPerUnit*10000/um_per_step; l2=max(l2, 1);  break;
      case 'C': l3=atof(ptr+1)*cmPerUnit*10000/um_per_step; l3=max(l3, 1);  break;
      // lengths between supports in cm
      case 'P': l12=atof(ptr+1)*cmPerUnit; l12=max(l12, 1);  break;
      case 'Q': l23=atof(ptr+1)*cmPerUnit; l23=max(l23, 1);  break;
      case 'R': l13=atof(ptr+1)*cmPerUnit; l13=max(l13, 1);  break;
      // height
      case 'H': h=atof(ptr+1)*cmPerUnit; h=max(h, 1);  break;
      case 'I':
        if(atoi(ptr+1)>0) {
          M1_REEL_IN=ANTICLOCKWISE;
          M1_REEL_OUT=CLOCKWISE;
        } else {
          M1_REEL_IN=CLOCKWISE;
          M1_REEL_OUT=ANTICLOCKWISE;
        }
        break;
      case 'J':
        if(atoi(ptr+1)>0) {
          M2_REEL_IN=ANTICLOCKWISE;
          M2_REEL_OUT=CLOCKWISE;
        } else {
          M2_REEL_IN=CLOCKWISE;
          M2_REEL_OUT=ANTICLOCKWISE;
        }
        break;
      case 'K':
        if(atoi(ptr+1)>0) {
          M3_REEL_IN=ANTICLOCKWISE;
          M3_REEL_OUT=CLOCKWISE;
        } else {
          M3_REEL_IN=CLOCKWISE;
          M3_REEL_OUT=ANTICLOCKWISE;
        }
        break;
      }
    }
    
    // x1=y1=y2=0
    x2 = l12;
    x3 = (l12*l12 + l13*l13 - l23*l23)/(2*l12);
    y3 = sqrt(l13*l13 - x3*x3);
    zmax = h;
    m1Pos_steps=l1;
    m2Pos_steps=l2;
    m3Pos_steps=l3;
    
    // update current position of effector
    // start by assuming no offset for object from machine space
    objx = objy = 0; objz = zmax;
    FK(l1, l2, l3, posx, posy, posz); // where is the effector?
    // adjust offsets and set effector at object origin
    objx = posx; objy = posy ; objz = zmax - posz; 
    posx = posy = posz = pose = 0; // set current pos as home, including extruder
     
    Serial.print("l12=P="); Serial.print(l12/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l23=Q="); Serial.print(l23/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l13=R="); Serial.print(l13/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l1=A="); Serial.print(l1*0.0001*um_per_step/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l2=B="); Serial.print(l2*0.0001*um_per_step/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("l3=C="); Serial.print(l3*0.0001*um_per_step/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("h=H="); Serial.print(h/cmPerUnit); Serial.println(inputUnit_Name);
    Serial.print("I"); Serial.print(M1_REEL_OUT);
    Serial.print(" J"); Serial.print(M2_REEL_OUT);
    Serial.print(" K"); Serial.println(M3_REEL_OUT);
    where();
    printConfig();

    
// G00 G01
  } else if(!strncmp(buffer,"G00 ",4) || !strncmp(buffer,"G01 ",4)
         || !strncmp(buffer,"G0 " ,3) || !strncmp(buffer,"G1 " ,3) ) {
    // line
    processSubcommand();
    float xx, yy, zz, ee;
    
    if(absolute_mode==1) {
      xx=posx;
      yy=posy;
      zz=posz;
      ee=pose;
    } else {
      xx=0;
      yy=0;
      zz=0;
      ee=0;
    }
    
 
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': xx= atof(ptr+1)*cmPerUnit;
        break;
      case 'Y': yy=atof(ptr+1)*cmPerUnit;
        break;
      case 'Z': zz=atof(ptr+1)*cmPerUnit;  break;
      case 'E': ee=atof(ptr+1)*cmPerUnit;  break;
      case 'F': microstepPeriod=feedrate2microseconds(atof(ptr+1));  break;
      }
    }
 
    if(absolute_mode==0) {
      xx+=posx;
      yy+=posy;
      zz+=posz;
      ee+=pose;
    }
    line(xx,yy,zz,ee);

// G02 G03
  } else if(!strncmp(buffer,"G02 ",4) || !strncmp(buffer,"G2 " ,3) 
         || !strncmp(buffer,"G03 ",4) || !strncmp(buffer,"G3 " ,3)) {
    // arc
    processSubcommand();
    float xx, yy, zz, ee;
    float dd = (!strncmp(buffer,"G02",3) || !strncmp(buffer,"G2",2)) ? -1 : 1;
    float ii = 0;
    float jj = 0;
    
    if(absolute_mode==1) {
      xx=posx;
      yy=posy;
      zz=posz;
      ee=pose;
    } else {
      xx=0;
      yy=0;
      zz=0;
      ee=0;
    }
    
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'I': ii=atof(ptr+1)*cmPerUnit;  break;
      case 'J': jj=atof(ptr+1)*cmPerUnit;  break;
      case 'X': xx= atof(ptr+1)*cmPerUnit; break;
      case 'Y': yy=atof(ptr+1)*cmPerUnit;  break;
      case 'Z': zz=atof(ptr+1)*cmPerUnit;  break;
      case 'E': ee=atof(ptr+1)*cmPerUnit;  break;
      case 'F': microstepPeriod=feedrate2microseconds(atof(ptr+1));  break;
      }
    }
 
    if(absolute_mode==0) {
      xx+=posx;
      yy+=posy;
      zz+=posz;
      ee+=pose;
    }

    arc(posx+ii,posy+jj,xx,yy,zz,dd,ee);

// G04
  } else if(!strncmp(buffer,"G04 ",4) || !strncmp(buffer,"G4 ",3)) {
    // dwell
    long xx=0;

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'X': 
      case 'U': 
      case 'P': xx=atoi(ptr+1);  break;
      }
    }

    delay(xx);

// D00
  } else if(!strncmp(buffer,"D00 ",4)) {
    // move one motor a number of steps
    char *ptr=strchr(buffer,' ')+1;
    int amount = atoi(ptr+1);
    int i, dir;
    if(*ptr == m1d) {
      dir = amount < 0 ? M1_REEL_IN : M1_REEL_OUT;
      amount=abs(amount);
      for(i=0;i<amount;++i) {  onestep(m1,dir); }
    } else if(*ptr == m2d) {
      dir = amount < 0 ? M2_REEL_IN : M2_REEL_OUT;
      amount = abs(amount);
      for(i=0;i<amount;++i) {  onestep(m2,dir); }
    } else if(*ptr == m3d) {
      dir = amount < 0 ? M3_REEL_IN : M3_REEL_OUT;
      amount = abs(amount);
      for(i=0;i<amount;++i) {  onestep(m3,dir); }
    } else if(*ptr == m4d) {
      dir = amount < 0 ? M4_REEL_IN : M4_REEL_OUT;
      amount = abs(amount);
      for(i=0;i<amount;++i) {  onestep(m4,dir); }
    }

// D01
  }  else if(!strncmp(buffer,"D01 ",4)) {
    // adjust factors for motors 
    //  shaft diameter for drive motors
    //  steps per unit for extruder
    
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      ptr=strchr(ptr,' ')+1;
      switch(*ptr) {
      case 'A': 
      case 'B': 
      case 'C': 
        checkNewStepLen(stringMovement(atof(ptr+1)) * cmPerUnit); 
        break;
      case 'E':
        e2steps = atof(ptr+1)/cmPerUnit; 
        if(e2steps>0) {
          M4_REEL_IN=CLOCKWISE;
          M4_REEL_OUT=ANTICLOCKWISE;
        } else {
          M4_REEL_IN=ANTICLOCKWISE;
          M4_REEL_OUT=CLOCKWISE;
          e2steps=abs(e2steps);
        }
        e2steps=max(e2steps,1);
        break;
      }
    }
    printConfig();

// D02
  } else if(!strncmp(buffer,"D02 ",4)) {
    Serial.print("all=");
    Serial.print(um_per_step * STEPS_PER_REV * 0.0001/ (PI * cmPerUnit) ); 
    // um/step * steps/rev * 0.0001 cm/um / cm/unit = unit
    Serial.println(inputUnit_Name);
  
    // temporary tests 
//    float tx = 0;
//    float ty = 0;
//    float tz = 0;
//    long t1 =  7375;
//    long t2 =  7375;
//    long t3 =  7375;
//    FK(t1,t2,t3,tx,ty,tz);
//    Serial.print(tx); Serial.print(","); 
//    Serial.print(ty); Serial.print(",");
//    Serial.println(tz);
//    IK(tx,ty,tz,t1,t2,t3);
//    Serial.print(t1 * um_per_step * 0.0001); Serial.print(";"); 
//    Serial.print(t2 * um_per_step * 0.0001); Serial.print(";");
//    Serial.println(t3 * um_per_step * 0.0001);
//    FK(t1,t2,t3,tx,ty,tz);
//    Serial.print(tx); Serial.print(","); 
//    Serial.print(ty); Serial.print(",");
//    Serial.println(tz);
//    IK(tx,ty,tz,t1,t2,t3);
//    Serial.print(t1 * um_per_step * 0.0001); Serial.print(";"); 
//    Serial.print(t2 * um_per_step * 0.0001); Serial.print(";");
//    Serial.println(t3 * um_per_step * 0.0001);
//    tx = 0;
//    ty = 0;
//    tz = 0;
//    IK(tx,ty,tz,t1,t2,t3);
//    Serial.print(t1 * um_per_step * 0.0001); Serial.print(";"); 
//    Serial.print(t2 * um_per_step * 0.0001); Serial.print(";");
//    Serial.println(t3 * um_per_step * 0.0001);


// default
  } else {
    if(processSubcommand()==0) {
      Serial.print(F("Invalid command '"));
      Serial.print(buffer);
      Serial.println(F("'"));
    }
  }
}


//------------------------------------------------------------------------------
void setup() {
  LoadConfig();
  
  // initialize the read buffer
  sofar=0;
  // start communications
  Serial.begin(BAUD);
  Serial.println(F("\nHello World! I am a rigtig's Big 3D Printer."));
  Serial.print(F("rB3DP #"));
  Serial.println(drawingMachineUID);
  
  // ready motors
  motorPinOutput(0); 
  motorPinOutput(1); 
  motorPinOutput(2); 
  motorPinOutput(3); 
  
  // initialize the input units as millimetres
  strcpy(inputUnit_Name,"mm");
  cmPerUnit=0.1;
  
  microstepPeriod=MIN_MICROSTEPPERIOD; // as fast as possible
  
  // display the help at startup.
  help();

  // initialize the plotter position.
  teleport(0,0,0,0);
  
  Serial.print(F("> "));
}


//------------------------------------------------------------------------------
void loop() {
  // See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
  // listen for serial commands
  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read();
    if(buffer[sofar-1]==0x0D || buffer[sofar-1]==0x0A) break; // LF is end-of-line (EOL)
    if(buffer[sofar-1]==';') { // ignore anything after semi-colon to EOL
      while (!(buffer[sofar-1] == 0x0D || buffer[sofar-1]==0x0A)) buffer[sofar-1]=Serial.read();
      break;
    }  
    // make input case insensitive, by making all input upper case
    if(isLowerCase(buffer[sofar-1])) {
      buffer[sofar-1]=buffer[sofar-1]-0x20;  //2ck
    }
  }
 
  // if we hit EOL, handle instruction.
  if(sofar>0 && (buffer[sofar-1]==0x0D || buffer[sofar-1]==0x0A)) {
    buffer[sofar-1]=0;
    
    // echo confirmation
    Serial.println(buffer);
 
    // do something with the command
    processCommand();
 
    // reset the buffer
    sofar=0;
 
    // echo completion
    Serial.print(F("> "));
  }
}

