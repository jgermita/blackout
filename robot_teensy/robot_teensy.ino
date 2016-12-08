#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <PulsePosition.h>
#include <Adafruit_PWMServoDriver.h>

// Servo driver stuff:
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  560 // this is the 'maximum' pulse length count (out of 4096)

// PPM decoder stuff:
PulsePositionInput myIn;
boolean connected = false, firstConnect = false;
int axis[] = {0,0,0,0};
int flap = 0;
int fm = 0;
int aux = 0;
int conn_ctr = 0, num = 0;

// 9DOF sensor stuff:
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
boolean accel_valid = false;

// Robot Orientation stuff:
byte v_axis = 2; // selector for vertical axis, 0 = x, 1 = y, 2 = z, 3 = disable inversion check
boolean v_invert = 0;	// Invert the vertical axis. 
boolean inverted = false;



void setup() {
  Serial.begin(115200);	// Serial for debugging...
  delay(50);	// Delay to let devices connect. 
  
  // init devices
  accel_valid = accel.begin();	// Accel
  pwm.begin();					// PWM Servo Driver
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
  
  yield();

  myIn.begin(6);

  Serial2.begin(9600);  // Datalogger
}

void loop() {
  updateSensors();
  updateRx();
  updatePwm();

  int lOut = 0;
  int rOut = 0;
  
  if(!inverted) {
    lOut = axis[0];
    rOut = axis[2];
  } else {
    lOut = -axis[2];
    rOut = -axis[0];
  }
  
  setPwm(0, lOut);
  setPwm(1, rOut);
  setPwm(2, (flap*50)-100);
  setPwm(3, (fm * 100)-100);

  Serial.print(flap);
  Serial.print("\t");
  Serial.print(fm);
  Serial.print("\t");
  Serial.print(aux);
  Serial.println("\t");
  
  //Serial.println(inverted ? 1 : 0);
  Serial2.print(connected);
  Serial2.print("\t");
  Serial2.print(num);
  Serial2.print("\t");
  Serial2.print(axis[0]);
  Serial2.print("\t");
  Serial2.print(axis[1]);
  Serial2.print("\t");
  Serial2.print(axis[2]);
  Serial2.print("\t");
  Serial2.println(axis[3]);
  delay(10);
}


void updatePwm() {
  digitalWrite(17, !connected);
}

void setPwm(int id, int value) {
  pwm.setPWM(id, 0, map(value, -100, 100, SERVOMIN, SERVOMAX));
}

void updateSensors() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  switch(v_axis) {
    case 0: 
      inverted = accel_event.acceleration.x < 0; break;
    case 1:
      inverted = accel_event.acceleration.y < 0; break;
    case 2:
      inverted = accel_event.acceleration.z < 0; break;
  }
  
  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    //Serial.print(("Orientation: "));
    //Serial.print(orientation.roll);
    //Serial.print((" "));
    //Serial.print(orientation.pitch);
    //Serial.print((" "));
    //Serial.print(orientation.heading);
    //Serial.println((""));
  }
  
}

void updateRx() {
  num = myIn.available();

  
  if (num < 15 && num != -1) {	// Read ppm data if valid.
    axis[0] = map(myIn.read(1), 805, 2215, -100, 100)-2;
    axis[1] = map(myIn.read(2), 1100, 1925, -100, 100);
    axis[2] = map(myIn.read(3), 805, 2215, -100, 100)-2;
    axis[3] = map(myIn.read(4), 1100, 1925, -100, 100);
    flap = map(myIn.read(5), 801, 2219, 0, 2);
    fm = map(myIn.read(7), 1034, 1985, 0, 2);
    aux = map(myIn.read(8), 1034, 1985, 0, 1);
    conn_ctr = 0;
  } else {
    conn_ctr++;
  }
  
  

  if(conn_ctr > 3) {	// if non-valid signal for 3 loops, zero inputs.
    connected = false;
    axis[0] = 0;
    axis[1] = 0;
    axis[2] = 0;
    axis[3] = 0;
	
	flap = 0;
	fm = 0;
	aux = 0;
  } else {
    connected = true;
  }
} 

