#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <PulsePosition.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  560 // this is the 'maximum' pulse length count (out of 4096)



PulsePositionInput myIn;

byte v_axis = 2; // selector for vertical axis, 0 = x, 1 = y, 2 = z, -1 = disable inversion check

Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

boolean inverted = false;

boolean connected = false, firstConnect = false;


int axis[] = {0,0,0,0};
int flap = 0;
int fm = 0;
int aux = 0;

int conn_ctr = 0, num = 0;

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}


void setup() {
  Serial.begin(115200);
  delay(50);
  initSensors();
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  yield();
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);

  myIn.begin(6);
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

  
  //Serial.println(inverted ? 1 : 0);
  Serial.print(connected);
  Serial.print("\t");
  Serial.print(num);
  Serial.print("\t");
  Serial.print(axis[0]);
  Serial.print("\t");
  Serial.print(axis[1]);
  Serial.print("\t");
  Serial.print(axis[2]);
  Serial.print("\t");
  Serial.println(axis[3]);
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

  // Every time new data arrives, simply print it
  // to the Arduino Serial Monitor.
  num = myIn.available();

  
  if (num < 15 && num != -1) {
    axis[0] = map(myIn.read(1), 805, 2215, -100, 100)-2;
    axis[1] = map(myIn.read(2), 1100, 1925, -100, 100);
    axis[2] = map(myIn.read(3), 805, 2215, -100, 100)-2;
    axis[3] = map(myIn.read(4), 1100, 1925, -100, 100);
    flap = myIn.read(5);
    fm = myIn.read(7);
    aux = myIn.read(8);
    conn_ctr = 0;
  } else {
    conn_ctr++;
  }
  
  

  if(conn_ctr > 3) {
    connected = false;
    axis[0] = 0;
    axis[1] = 0;
    axis[2] = 0;
    axis[3] = 0;
  } else {
    connected = true;
  }
} 


