#include "ArduinoSTL.h"
#include "Enes100.h"
#include <Servo.h>
#include <Math.h>
using namespace std;
//Black-Orange-Red-White
//Macros
#define DISTANCE_TOLERANCE 0.05
#define THETA_TOLERANCE PI/180. //1 degree of tolerance
#define ARUCO_NUMBER 19
#define WIFI_TX 9
#define WIFI_RX 8
#define SERVO_OUT 3
#define LEFT_ENABLE 5
#define RIGHT_ENABLE 6
#define LEFT_MOTOR_1 10
#define LEFT_MOTOR_2 11
#define RIGHT_MOTOR_1 7
#define RIGHT_MOTOR_2 4
#define ULTRASONIC_FRONT_TRIG 14
#define ULTRASONIC_FRONT_ECHO 15
#define DEFAULT_SPEED 150
#define LOWERED_SERVO_VALUE 173
#define RAISED_SERVO_VALUE 110
#define DEFAULT_TURNING_DURATION 70
#define PWM_PIN 16
#define MISSION_SITE_APPROACH_TOLERANCE_M 0.02
#define CENTER_TO_SENSOR .2475

Servo myservo;
void setup() {
  
  while(!Enes100.begin("DATA TEAM (REAL)", DATA, ARUCO_NUMBER, WIFI_TX, WIFI_RX)) {
    Enes100.println("Unable to connect to simulation");
  }
  
  // Initialize Enes100 Library
  // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
 
  myservo.attach(SERVO_OUT);
  pinMode(ULTRASONIC_FRONT_ECHO, INPUT);
  pinMode(ULTRASONIC_FRONT_TRIG, OUTPUT);
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(PWM_PIN, INPUT);
  Serial.begin(9600);
}

double read_cycle() {
  int pwm_value = pulseIn(PWM_PIN, HIGH);
  return pwm_value;
}


std::vector<double> minus(std::vector<double> one, std::vector<double> two) {
  std::vector<double> result;
  for (int i = 0; i < min(one.size(), two.size()); i++) {
    result.push_back(one[i] - two[i]);
  }
  return result;
}

std::vector<double> get_heading(std::vector<double> target) {
  std::vector<double> result = {target[0] - get_x(), target[1] - get_y()};
  return result;
}

void left_forwards() {
  digitalWrite(LEFT_MOTOR_1, HIGH); digitalWrite(LEFT_MOTOR_2, LOW);
}

void left_backwards() {
    digitalWrite(LEFT_MOTOR_1, LOW); digitalWrite(LEFT_MOTOR_2, HIGH);
}

void right_forwards() {
  digitalWrite(RIGHT_MOTOR_1, HIGH); digitalWrite(RIGHT_MOTOR_2, LOW);
}

void right_backwards() {
    digitalWrite(RIGHT_MOTOR_1, LOW); digitalWrite(RIGHT_MOTOR_2, HIGH);
}

void turn_left(int wheel_speed) {
  analogWrite(LEFT_ENABLE, wheel_speed);
  analogWrite(RIGHT_ENABLE, wheel_speed);
  left_backwards();
  right_forwards();
}

void turn_right(int wheel_speed) {
  analogWrite(LEFT_ENABLE, wheel_speed);
  analogWrite(RIGHT_ENABLE, wheel_speed);
  left_forwards();
  right_backwards();
}

void forwards(int wheel_speed) {
  analogWrite(LEFT_ENABLE, wheel_speed);
  analogWrite(RIGHT_ENABLE, wheel_speed);
  left_forwards();
  right_forwards();
}

void backwards(int wheel_speed) {
  analogWrite(LEFT_ENABLE, wheel_speed);
  analogWrite(RIGHT_ENABLE, wheel_speed);
  left_backwards();
  right_backwards();
}

void halt() {
  analogWrite(LEFT_ENABLE, 0);
  analogWrite(RIGHT_ENABLE, 0);
}

double dot(std::vector<double> one, std::vector<double> two) {
  double dot_product = 0;
  for (int i = 0; i < min(one.size(), two.size()); i++) {
    dot_product += one[i] * two[i];
  }
  return dot_product;
}

double norm(std::vector<double> vec, double p) {
  double sum = 0;
  for (int i = 0; i < vec.size(); i++) {
    sum += pow(vec[i], p);
  }
  return pow(sum, 1./p);
}

double angle(std::vector<double> one, std::vector<double> two) {
  return acos(cos(one, two));
}

double cos(std::vector<double> one, std::vector<double> two) {
  return dot(one, two) / (norm(one, 2) * norm(two, 2));
}

double angle(std::vector<double> one, double theta) {
  std::vector<double> vec;
  vec.push_back(cos(theta));
  vec.push_back(sin(theta));
  return angle(one, vec);
}

void orient_to_heading(std::vector<double> heading) {
  double current_angle;
  halt();
  while (!Enes100.updateLocation()) {
   
  }
  while (abs(angle(heading, get_theta())) > THETA_TOLERANCE) {
    current_angle = angle(heading, get_theta());
    if (current_angle > 0) {
      turn_left(DEFAULT_SPEED);
    } else {
      turn_right(DEFAULT_SPEED);
    }
    delay(pow(abs(angle(heading, get_theta())), 0.5) * DEFAULT_TURNING_DURATION);
    halt();
    while (!Enes100.updateLocation()) {
     
    }
  }
}

void update_motors_with_target(std::vector<double> target, int duration_ms) {
  std::vector<double> heading = get_heading(target);
  orient_to_heading(heading);
  forwards(DEFAULT_SPEED);
  delay(duration_ms);
  halt();
}

double read_from_front_ultrasonic() {
  long duration;
  double distance;
  digitalWrite(ULTRASONIC_FRONT_TRIG, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(ULTRASONIC_FRONT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_FRONT_TRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ULTRASONIC_FRONT_ECHO, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  distance *= 0.01;
  return distance;
}


float get_x() {
  return Enes100.location.x;
}

float get_y() {
  return Enes100.location.y;
}

double get_theta() {
  double theta = Enes100.location.theta;
  if (theta >= 0) {
    return theta;
  } else {
    return 2 * PI - abs(theta);
  }
}

void print_location() {
  Enes100.print("OSV is at (x=");
  Enes100.print(get_x());
  Enes100.print(", y=");
  Enes100.print(get_y());
  Enes100.print(", theta=");
  Enes100.print(get_theta());
  Enes100.println(")");
}

void wait_forever() {
  while (1) {
    halt();
    Enes100.updateLocation();
    print_location();
    delay(1000);
  }
}

/*
 * Algorithm for mission
 * 1. Make sure arm is raised
 * 2. Compute location of mission site
 * 3. Navigate to a point 0.2 meters away from the mission site
 * 4. Navigate to arm's distance from site using high-precision approach routine
 * 5. Execute contact routine
 * 6. Execute duty cycle read routine
 * 7. Raise arm
 * 8. Navigate to limbo
 * 9. Lower arm
 * 10. Pass under limbo
 *
 *
 */

 std::vector<double> map_arena() {
  //Navigate to (1, 0.5), (1, 1.0), (1, 1.5)
  //After each arrival, orient to theta 0, take 3 readings from ultrasonic, average them together, and save them
  std::vector<double> arena_map;
  std::vector<std::vector<double>> coords = {{1.0, 0.5}, {1.0, 1.0}, {1.0, 1.5}};
  std::vector<double> plus_x = {1.0, 0.0};
  while (norm(get_heading(coords[0]), 2) > 0.03) {
    print_location();
    while (!Enes100.updateLocation()) {}
    update_motors_with_target(coords[0], norm(get_heading(coords[0]), 2) * 1000);
  }
  orient_to_heading(plus_x);
  arena_map.push_back((read_from_front_ultrasonic() + read_from_front_ultrasonic() + read_from_front_ultrasonic()) / 3.0);
  
  while (norm(get_heading(coords[1]), 2) > 0.03) {
    print_location();
    while (!Enes100.updateLocation()) {}
    update_motors_with_target(coords[1], norm(get_heading(coords[1]), 2) * 1000);
  }
  orient_to_heading(plus_x);
  arena_map.push_back((read_from_front_ultrasonic() + read_from_front_ultrasonic() + read_from_front_ultrasonic()) / 3.0);
  
  while (norm(get_heading(coords[2]), 2) > 0.03) {
    print_location();
    while (!Enes100.updateLocation()) {}
    update_motors_with_target(coords[2], norm(get_heading(coords[2]), 2) * 1000);
  }
  orient_to_heading(plus_x);
  arena_map.push_back((read_from_front_ultrasonic() + read_from_front_ultrasonic() + read_from_front_ultrasonic()) / 3.0);

  return arena_map;
 }

void raise_arm() {
  myservo.write(RAISED_SERVO_VALUE);
  delay(1000);
}

void lower_arm() {
  myservo.write(LOWERED_SERVO_VALUE);
  delay(1000);
}

/*
 * Obstacle avoidance algorithm:
 *    1. Read from ultrasonic sensor
 *    2. If reading is less than 0.3 meters, and get_y() < 1, orient to PI/2, drive forward for 1500 ms, then 
 *    3. 
 *    
 * 
 */

 std::vector<double> compute_mission_site_coords() {
  std::vector<double> mission_site_coords = {0.55};
  while (!Enes100.updateLocation()) {}
  if (get_y() < 1.0) {
    mission_site_coords.push_back(1.2);
  } else {
    mission_site_coords.push_back(0.8);
  }
  return mission_site_coords;
 }

int wait_on_contact() {
  while (read_cycle() < 1000) {
    lower_arm();
  }
  return round(read_cycle() / 1000) * 10;
}

int compute_rumble_index(std::vector<double> arena_map) {
  double maximum = -1;
  int index = 0;
  for (int i = 0; i < arena_map.size(); i++) {
    if (arena_map[i] > maximum) {
      maximum = arena_map[i];
      index = i;
    }
  }
  return index;
}

void transmit_duty_cycle(int cycle_percent_value) {
  Enes100.mission(CYCLE, cycle_percent_value);
}

void transmit_magnetism(boolean is_magnetic) {
  Enes100.mission(MAGNETISM, is_magnetic ? MAGNETIC : NOT_MAGNETIC);
}

void make_contact_and_transmit() {
  double lower_threshold = 0.05;
  double upper_threshold = 0.15;
  raise_arm();
  turn_right(150);
  delay(350);
  halt();
  raise_arm();
  while (read_from_front_ultrasonic() < lower_threshold || read_from_front_ultrasonic() > upper_threshold) {
    turn_left(150);
    delay(75);
  }
  halt();
  lower_arm();
  forwards(150);
  int cycle_percentage = wait_on_contact();
  halt();
  transmit_duty_cycle(cycle_percentage);
  transmit_magnetism(get_site_magnetism());
  raise_arm();
  backwards(150);
  delay(500);
  turn_right(150);
  delay(500);
  halt();
}

boolean get_site_magnetism(void);

void loop() {
  raise_arm();
  std::vector<double> mission_site_coords = compute_mission_site_coords();
  while (!Enes100.updateLocation()) {}
  while (norm(get_heading(mission_site_coords), 2) > MISSION_SITE_APPROACH_TOLERANCE_M) {
    while (!Enes100.updateLocation()) {}
    update_motors_with_target(mission_site_coords, norm(get_heading(mission_site_coords), 2) * 1000);
  }
  make_contact_and_transmit();
  std::vector<double> arena_map = map_arena();
  int rumble_index = compute_rumble_index(arena_map);
  std::vector<double> target_coords = {1.0, 0.5 * (1.0 + rumble_index)};
  while (!Enes100.updateLocation()) {}
  while (norm(get_heading(target_coords), 2) > DISTANCE_TOLERANCE) {
    while (!Enes100.updateLocation()) {}
      update_motors_with_target(target_coords, 250);
  }
  target_coords[0] += 2.0;
  while (!Enes100.updateLocation()) {}
  while (norm(get_heading(target_coords), 2) > DISTANCE_TOLERANCE) {
    if (norm(get_heading(target_coords), 2) < 0.5) {
      lower_arm();
    }
    while (!Enes100.updateLocation()) {}
    update_motors_with_target(target_coords, 500);
  }
  delay(600000);
}
