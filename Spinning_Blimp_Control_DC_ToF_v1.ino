#include <crazyflieComplementary.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <VL53L1X.h>

#define THRUST1 A0
#define THRUST2 A1
// #define THRUST1 A2
// #define THRUST2 A3

VL53L1X sensor;

// min and max high signal of thruster PWMs
int minUs = 0;
int maxUs = 255*2;

// Wi-Fi access details
const char * ssid = "AIRLab-BigLab";
const char * password = "Airlabrocks2022";

// using servo lib to control brushless motors
// Servo thrust1;
// Servo thrust2; 

// c.c. Edward
SensFusion sensorSuite;

AsyncUDP udp;
float joy_data[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
volatile bool joy_ready = false;
volatile unsigned long time_now, time_loop; 

// P.I.D. Values
float roll, pitch, yaw;
float rollrate, pitchrate, yawrate;
float estimatedZ, velocityZ, groundZ;
float abz = 0.0;
float kpz = 0.08; // N/meter
float kiz = 0.002;
float kdz = 0.005;
float kpx = 0.035;
float kdx = 0.2;
float kptz = 0.3;
float kdtz = -0.025;
float kptx = 0.01;
float kdtx = 0.01;
float lx = 0.25;
float m1 = 0.0;
float m2 = 0.0;
// float heading = 0.0;
float massthrust = 0.3;
long dt, last_time, time_nw;

float wall = 0.0;
float kwall = 0.02;
float last_time_flip = 0.0;

float forcex = 0.3;
// float forcey = 0.001*random(-100, 100);
float forcey = 0.3;
float walk_heading  = atan2(forcey,forcex);
float joymag = sqrt(pow(forcex,2) + pow(forcey,2));
float random_reflect = 0;
float wall_min = 1500;
float yaw_min_dist = 0;

int wall_detect;
int wall_truly_short;

bool random_trip = false;



// float timedif;
// float timest = 0;
// float timeend = 0;

void setup() {
  Serial.begin(9600); 
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }

  // Allocate timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  // Standard 50hz PWM for thrusters
  // thrust1.setPeriodHertz(50);
  // thrust2.setPeriodHertz(50);

  // thrust1.attach(THRUST1, minUs, maxUs);
  // thrust2.attach(THRUST2, minUs, maxUs);

  pinMode(THRUST1,OUTPUT);
  pinMode(THRUST2,OUTPUT);

  delay(100);

  // ESC arm
  // escarm();
  
  // Access sensor suite c.c. Edward
  sensorSuite.initSensors();
  sensorSuite.updateKp(5, -1, 0.3); // 20, -1, 0
  groundZ = sensorSuite.returnZ();
  // sensorSuite.recordData();

  // magnetometer calibration
  float transformationMatrix[3][3] = {
    {1.0f, 9.693f, 0.6187f},
    {9.6624f, -0.6822f, 0.3864f},
    {-0.4155f, 0.6628f, -10.7386f}
  };
  float offsets[3] = {11.98f, 7.01f, 21.77f};
  sensorSuite.enterTransform(offsets, transformationMatrix);
  getSensorValues();

  
  time_now = millis();
  // time_loop = millis();
  if(udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());

    // setup callback functions of the udp
    udp.onPacket([](AsyncUDPPacket packet) {
      joy_ready = false;
      time_now = millis();
      unsigned char *buffer = packet.data();
      unpack_joystick(joy_data, buffer);
      joy_ready = true;
      //reply to the server
      // packet.printf("Got %u bytes of data", packet.length());
    });
  }

  // Yaw heading setup
  // heading = sensorSuite.getYaw(); 

  // Time of flight sensors setup
  Wire.begin();
  Wire.setClock(400000);

  sensor.setTimeout(500);
  if (!sensor.init()){
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(15000);
  sensor.startContinuous(15);
  Serial.println("sensor ready");

}

void loop() {
    //gyro, acc, mag, euler, z
  float cfx, cfy, cfz, ctx, cty, ctz;

  // Runs the sensor fusion loop
  sensorSuite.sensfusionLoop(false, 5);

  if (joy_data[7] != 1){

    // Serial.println("Initialization") //debug;
    // thrust1.writeMicroseconds(minUs);
    // thrust2.writeMicroseconds(minUs);
    analogWrite(THRUST1, (int) minUs);
    analogWrite(THRUST2, (int) minUs);
    
    wall = sensor.read();
    // Serial.println(wall);

    // Debug joystick input
    // thrust1.writeMicroseconds((int) (1000 + joy_data[0]));
    // thrust2.writeMicroseconds((int) (1000 + joy_data[0]));

  } else if (joy_ready && millis() - time_now <= 1000){ //&& millis() - time_loop > 50) {kdz
    // Call sensor suite to update 10-DOF values
    getSensorValues();
    getControllerInputs(&cfx, &cfy, &cfz, &ctx, &cty, &ctz, &abz);
    addFeedback(&cfx, &cfy, &cfz, &ctx, &cty, &ctz, abz);
    controlOutputs(cfx, cfy, cfz, ctx, cty, ctz);

    int m1us = 0;
    int m2us = 0;
    
    // if (joy_data[3] != 1 joy_data[7] == 1){  

    //   // Convert motor input to 1000-2000 Us values
    m1us = (minUs + (maxUs - minUs)*m1*2.0);
    m2us = (minUs + (maxUs - minUs)*m2*2.0);

    // } else{
    //   // m1us = abz;
    //   // m2us = abz;
    // }
    // Write motor thrust values to the pins
    // m1us = clamp(m1us, 1100, 1800);
    // m2us = clamp(m2us, 1100, 1800);
    // m1us = clamp(m1us, 1125, 1550);
    // m2us = clamp(m2us, 1125, 1550);
    // thrust1.write((int) m1us);
    // thrust2.write((int) m2us);
    analogWrite(THRUST1, (int) m1us);
    analogWrite(THRUST2, (int) m2us);
    // Serial.print("m1us: ");
    // Serial.println((int) m1us);

    // Else statement for if the Wi-Fi signal is lost
  } else {
    // thrust1.writeMicroseconds((int) minUs);
    // thrust2.writeMicroseconds((int) minUs);
    analogWrite(THRUST1, (int) minUs);
    analogWrite(THRUST2, (int) minUs);
    
  }

}

//Enter arming sequence for ESC
// void escarm(){
//   // ESC arming sequence for BLHeli S
//   thrust1.writeMicroseconds(1000);
//   delay(10);
//   thrust2.writeMicroseconds(1000);
//   delay(10);

//   // Sweep up
//   for(int i=1100; i<1500; i++) {
//     thrust1.writeMicroseconds(i);
//     delay(5);
//     thrust2.writeMicroseconds(i);
//     delay(5);
//   }
//   // Sweep down
//   for(int i=1500; i<1100; i--) {
//     thrust1.writeMicroseconds(i);
//     delay(5);
//     thrust2.writeMicroseconds(i);
//     delay(5);
//   }
//   // Back to minimum value
//   thrust1.writeMicroseconds(1000);
//   delay(10);
//   thrust2.writeMicroseconds(1000);
//   delay(10);

// }


void getSensorValues(){ 
  //all in radians or meters or meters per second
  // roll = sensorSuite.getRoll();
  // pitch = -1*sensorSuite.getPitch();
  yaw = sensorSuite.getYaw();
  Serial.println(yaw);
  // rollrate = sensorSuite.getRollRate();
  // pitchrate = sensorSuite.getPitchRate();
  yawrate = sensorSuite.getYawRate();
  estimatedZ = sensorSuite.returnZ();
  velocityZ = sensorSuite.returnVZ(); 
}

float valtz = 0;
void getControllerInputs(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float *abz){
  if (false) {
    *fx = 0;//joy_data[0];
    *fy = 0;//joy_data[1];
    *fz = 0;//joy_data[2];
    *tx = 0;//joy_data[3];
    *ty = 0;//joy_data[4];
    *tz = 0;//joy_data[5];
    *abz = 0;//joy_data[6];
    if (valtz > 1){
      valtz = -1;
    } else {
      valtz += .01;
    }
  } else{
  *fx = joy_data[0];
  *fy = joy_data[1];
  *fz = joy_data[2];
  *tx = joy_data[3];
  *ty = joy_data[4];
  *tz = joy_data[5];
  *abz = joy_data[6];
  }
}
void addFeedback(float *fx, float *fy, float *fz, float *tx, float *ty, float *tz, float abz){
    // *fz = 0;
    // float err = estimatedZ-groundZ;
    delta_time();
    // float int_err =+ dt * err;
    // *fz = (*fz  - (estimatedZ-groundZ))*kpz - (int_err * kiz) - (velocityZ)*kdz + abz;//*fz = *fz + abz;//
    // *fz = (*fz  - (*tz))*kpz - (velocityZ)*kdz + abz;//*fz = *fz + abz;//
    // Serial.print(estimatedZ);
    // Serial.println(*tz);
    *fz = (*fz  - (estimatedZ-groundZ))*kpz - (velocityZ)*kdz + abz;//*fz = *fz + abz;//
    // *fz = *fz + *tz;
  // Serial.println(int_err*kiz);
}

long delta_time(){
  time_nw = millis();
  dt = (time_nw - last_time);
  last_time = time_nw;
  return dt;
}

float clamp(float in, float min, float max){
  if (in< min){
    return min;
  } else if (in > max){
    return max;
  } else {
    return in;
  }
}

void controlOutputs(float ifx, float ify, float ifz, float itx, float ity, float itz) {
  // Define boundary for flipping fx-direction
  int distance = 1400;
  //yaw = ity;


  // Convert joystick input to theta and magnitude for cyclic input
  // float joytheta = atan2(ify,-ifx) + PI;  

  // Flipping fx-direction based on time since fx was last flipped
  if ((time_nw - last_time_flip)>6000.0){
    wall = sensor.read();
    
    if (random_trip == true){
      walk_heading = walk_heading + (PI*0.0025 * random(-25,25));
      random_trip = false;
    }
    // create a variable that always keeps the min wall distance
    // store yaw value at the min wall distance

    if (wall<distance) {
      wall_detect ++;
      if (wall_min > wall){
        wall_min = wall;
        yaw_min_dist = yaw;
      }

      if (wall_detect > 3){
        joymag = 0;

        if (wall>wall_min){
          wall_truly_short ++;
        }

        if (wall_truly_short > 2){
          // walk_heading = walk_heading + 2.0*(1.57 - (walk_heading - yaw_min_dist)); // 
          walk_heading = -yaw_min_dist;

          // if (walk_heading > PI){
          //   walk_heading = walk_heading - 2*PI;
          // } else{if (walk_heading < -1.0*PI){
          //   walk_heading = walk_heading + 2*PI;
          // }
          // }

          // forcex = -1.0 * forcex;
          // forcey = -1.0 * forcey;
          last_time_flip = time_nw;
          wall_detect = 0;
          wall_truly_short = 0;
          joymag = sqrt(pow(forcex,2) + pow(forcey,2));
          wall_min = 1500;
          random_trip = true;
        }
      }
    }else{if(wall_detect > 0){wall_detect --;}}
  }
  send_udp_feedback(wall,wall_min,estimatedZ,groundZ);
  // float joymag  = sqrt(pow(forcex,2) + pow(forcey,2));

  // Cyclic pitch input
  // float lhs = joymag*(coscos  - sincos);
  // float rhs = joymag*(-sincos - coscos);

  float pitch = joymag * sin(walk_heading) * cos(yaw);
  float roll =  joymag * cos(walk_heading) * sin(yaw);


  // TODO: bring back x-y feedback
  // Mass Thrust it a proportional debug gain
  float mt = massthrust;
  float f1 = 0;
  float f2 = 0;

  // if (wall<distance) {
  //   f1 = ifz + (mt * (pitch + roll)) - 0.0*kwall*(distance-wall); // LHS motor
  //   f2 = ifz - (mt * (pitch + roll)) + kwall*(distance-wall); // RHS motor
  //   ifx = ifx*-1.0;
  //   sleep(2);
  // } else{
  //   f1 = ifz + (mt * (pitch + roll)); // LHS motor
  //   f2 = ifz - (mt * (pitch + roll)); // RHS motor
  // }
  f1 = ifz + (mt * (pitch + roll)); // LHS motor
  f2 = ifz - (mt * (pitch + roll));

  // Clamp to ensure motor doesn't stop spinning at minimum
  // and motor doesn't draw too much current
  m1 = clamp(f1, 0.02, 0.25*2.0);
  m2 = clamp(f2, 0.02, 0.25*2.0);
  // Serial.println(m1);
  // m1 = clamp(f1, 0.03, 0.19);
  // m2 = clamp(f2, 0.03, 0.19);

}

// Having questions? Ask Jiawei!
void unpack_joystick(float *dat, const unsigned char *buffer) {
  int num_floats = 8;
  int num_bytes = 4;
  int i, j;

  for(i = 0; i < num_floats; i++) {
    char temp[4] = {0, 0, 0, 0};
    for(j = 0; j < num_bytes; j++) {
      temp[j] = buffer[num_bytes*i + j];
    }
    dat[i] = *((float*) temp);

  }
;
}

// Optional UDP feeback section
// ----------------------------

// This function converts floats to strings and concatenates the strings for broadcastTo func
void send_udp_feedback(int wall, float wall_min, float walk_heading, float yaw_min_dist){

  String motor1 = String(wall);
  String motor2 = String(wall_min);
  String motor3 = String(walk_heading);
  String motor4 = String(yaw_min_dist);
  String comb = String("");
  String comma = String(", ");
  String semicol = String(";");
  comb = motor1 + comma + motor2 + comma + motor3 + comma + motor4 + semicol;
  // UDP Broadcast (string,port)
  udp.broadcastTo(comb.c_str(),8003);
// udp.broadcastTo("I Work",8001);
}