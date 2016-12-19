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
int8_t motor[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Motor ID definitions:
#define LEFT_DRIVE 0
#define RIGHT_DRIVE 1
#define WEAPON 2
#define LED_STRIP 3

// PPM decoder stuff:
PulsePositionInput myIn;
boolean connected = false, firstConnect = false;
int8_t axis[] = {0, 0, 0, 0};
int flap = 0;
int fm = 0;
int aux = 0;
int conn_ctr = 0, num = 0;
// Axis ID definitions:
#define THROTTLE 0
#define YAW 1
#define ELEVATOR 2
#define ROLL 3

// 9DOF sensor stuff:
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
boolean accel_valid = false;
int accel_data[] = {0, 0, 0};
int gyro_data[] = {0, 0, 0};

// Robot Orientation stuff:
byte v_axis = 3; // selector for vertical axis, 0 = x, 1 = y, 2 = z, 3 = disable inversion check
#define v_invert 1
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
  int8_t lOut = 0;
  int8_t rOut = 0;

  if (!inverted) {
    lOut = axis[THROTTLE];
    rOut = axis[ELEVATOR];
  } else {
    lOut = -axis[ELEVATOR];
    rOut = -axis[THROTTLE];
  }

  motor[LEFT_DRIVE] = -lOut;
  motor[RIGHT_DRIVE] = -rOut;

  // "SmartSpin" weapon control. 
  // pulls back weapon command speed as a function of commanded turning rate
  int8_t weaponIn = (flap * 50) - 100;
  
  int8_t turning = (motor[LEFT_DRIVE] + motor[RIGHT_DRIVE]);
  motor[WEAPON] = weaponIn - ((turning*4)/5);
  
  motor[LED_STRIP] = (fm * 100) - 100;
  
  updateSensors();
  updateRx();
  updatePwm();
  updateLog();
}


void updatePwm() {
  digitalWrite(17, !connected);

  for(int8_t i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, map(motor[i], -100, 100, SERVOMIN, SERVOMAX));  
  }
}



int inv_ctr = 0;
void updateSensors() {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  accel_data[0] = accel_event.acceleration.x;
  accel_data[1] = accel_event.acceleration.y;
  accel_data[2] = accel_event.acceleration.z;
  int axis = 0;
  switch (v_axis) {
    case 0:
      axis = accel_data[0]; break;
    case 1:
      axis = accel_data[1]; break;
    case 2:
      axis = accel_data[2]; break;
    case 3:
      axis = 0; break;
  }

  axis *= -v_invert;

  if(axis < -4) {
    inv_ctr++;
  } else {
    inv_ctr = 0;
  }

  inverted = inv_ctr > 2;
  
  /* Use the new fusionGetOrientation function to merge accel/mag data */
  //if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  //{
    /* 'orientation' should have valid .roll and .pitch fields */
    //Serial.print(("Orientation: "));
    //Serial.print(orientation.roll);
    //Serial.print((" "));
    //Serial.print(orientation.pitch);
    //Serial.print((" "));
    //Serial.print(orientation.heading);
    //Serial.println((""));
  //}

}

void updateRx() {
  num = myIn.available();


  if (num < 15 && num != -1) {	// Read ppm data if valid.
    axis[0] = map(myIn.read(1), 805, 2215, -100, 100) - 2;
    axis[1] = map(myIn.read(2), 1100, 1925, -100, 100);
    axis[2] = map(myIn.read(3), 805, 2215, -100, 100) - 2;
    axis[3] = map(myIn.read(4), 1100, 1925, -100, 100);
    flap = map(myIn.read(5), 801, 2219, 0, 2);
    fm = map(myIn.read(7), 1034, 1985, 0, 2);
    aux = map(myIn.read(8), 1034, 1985, 0, 1);
    conn_ctr = 0;
  } else {
    conn_ctr++;
  }



  if (conn_ctr > 3) {	// if non-valid signal for 3 loops, zero inputs.
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

int t = 0;
void updateLog() {

  if (!connected) {
    t++;
  } else {
    t = 0;
  }

  if (t < 50) {
    Serial2.print(millis());
    Serial2.print("\t");
    Serial2.print(connected);
    Serial2.print("\t");
    Serial2.print(axis[0]);
    Serial2.print("\t");
    Serial2.print(axis[1]);
    Serial2.print("\t");
    Serial2.print(axis[2]);
    Serial2.print("\t");
    Serial2.print(axis[3]);
    Serial2.print("\t");
    Serial2.print(flap);
    Serial2.print("\t");
    Serial2.print(aux);
    Serial2.print("\t");
    Serial2.print(fm);
    Serial2.print("\t");
    Serial2.print(accel_data[0]);
    Serial2.print("\t");
    Serial2.print(accel_data[1]);
    Serial2.print("\t");
    Serial2.print(accel_data[2]);
    Serial2.print("\t");
    Serial2.print(gyro_data[0]);
    Serial2.print("\t");
    Serial2.print(gyro_data[1]);
    Serial2.print("\t");
    Serial2.print(gyro_data[2]);
    Serial2.print("\t");
    for(int8_t i = 0; i < 16; i++) {
      Serial2.print(motor[i]);
      Serial2.print("\t");
    }
    Serial2.println("");
  }
}

