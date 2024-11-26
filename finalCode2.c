#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position


const int trig = 3;  //doi tu 6 thanh 3
const int echo = 4;//5    
int tien1 = 10;       
int tien2 = 11;        
int lui1 = 12;        
int lui2 = 13;     
int enA = 6; //doi tu 3 thanh 6
int enB = 5;   
int dongcoservo = 9;   


double e, P, I, D, PID;
double pe = 0;
double Kp = 1;
//int Ki = 1/2;
//int Kd = 1/2;
int v_moi;
int v_hientai = 50;
int gioihan = 50;//khoảng cách nhận biết vật 
int i;
int x = 0;
unsigned long thoigian;
int khoangcach;          
int khoangcachtrai, khoangcachphai;
int maxspeed=30;

double dokhoangcach();
void dithang(int duongdi);
void disangtrai();
void disangphai();
void dilui();
void resetdongco();
void quaycbsangphai();
void quaycbsangtrai();
void tiepcan(double PID);
double tinhpid();
#define MAX_DISTANCE 100    // Maximum distance from the obstacle
#define MAX_SPEED 100       // Maximum car speed
#define GAMMA 0.6           // Discount factor

// Reward function
double reward(int distance, int speed) {
    if (distance < 25) return -100; // Collision penalty
    if (distance == 25) return 100; // Terminal reward

    // Linear mapping of distance to optimal speed
    int optimal_speed = fmax(20, 80 - (80 - 20) * (100 - distance) / 75);
    
    // // Penalize deviation from optimal speed
    // double speed_penalty = fabs(speed - optimal_speed) * 2.0; // Higher penalty for large deviations

    // return 50.0 - speed_penalty; // Reward is higher when speed is closer to the optimal value
    double speed_penalty = fabs(speed - optimal_speed) * 2.0; // Higher penalty for large deviations

    // Penalize high speeds for small distances explicitly
    double distance_penalty = (distance < 50) ? (speed * (50 - distance) / 50.0) : 0;

    return 50.0 - speed_penalty - distance_penalty;
}

// Transition function
void next_state(int distance, int speed, const char* action, int* new_distance, int* new_speed) {
    int new_speed_temp = speed;
    if (strcmp(action, "slow_down") == 0) new_speed_temp = fmax(0, speed - 1);
    else if (strcmp(action, "speed_up") == 0) new_speed_temp = fmin(MAX_SPEED, speed + 1);

    *new_distance = fmax(0, distance - new_speed_temp); // New distance depends on speed
    *new_speed = new_speed_temp; // Update speed
    //Serial.print("current distance in old state: "); Serial.print(distance); Serial.print("\n");
    //Serial.print("current speed in old state: "); Serial.print(speed); Serial.print("\n");
    //Serial.print("action choose: "); Serial.print(action); Serial.print("\n");
    //Serial.print("new distance with action have choosen: "); Serial.print(*new_distance); Serial.print("\n");
    //Serial.print("new speed with action have choosen: "); Serial.print(*new_speed); Serial.print("\n");
    
}

// Compute value for a single state
double compute_value(int distance, int speed, const char* actions[], int action_count) {
    double max_value = -1e9; // Initialize with a large negative value
    for (int i = 0; i < action_count; ++i) {
        int next_distance, next_speed;
        next_state(distance, speed, actions[i], &next_distance, &next_speed);
        double value = reward(next_distance, next_speed) + GAMMA * reward(next_distance, next_speed); // Approximating next state's value
        max_value = fmax(max_value, value);
    }
    
    return max_value;
}

int computeAction(int distance) {
    const char* actions[] = {"slow_down", "maintain_speed", "speed_up"};

    double best_value = -1e9;
    int best_speed = 0;

    for (int speed = 0; speed <= MAX_SPEED; ++speed) {
        double value = compute_value(distance, speed, actions, 3);

        // Introduce a bias for reducing speed at lower distances
        double bias = (distance < 50) ? -(speed * (50 - distance) / 50.0) : 0;
        value += bias;

        if (value > best_value) {
            best_value = value;
            best_speed = speed;
        }
    }

    return best_speed;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(9); 
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);   

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(tien1, OUTPUT);
  pinMode(tien2, OUTPUT);
  pinMode(lui1, OUTPUT);
  pinMode(lui2, OUTPUT);
  digitalWrite(tien1, LOW);
  digitalWrite(tien2, LOW);
  digitalWrite(lui1, LOW);
  digitalWrite(lui1, LOW);
  myservo.write(90);
  delay(500);



}

void loop()
{
  v_hientai = 50; // xung
  khoangcach = 0;
  khoangcach = dokhoangcach();
  //Serial.print("THis is khoang cach: ");
  Serial.print(khoangcach);
  Serial.print("\n");
  //Serial.print("\n");
  /*
  if (khoangcach <= 30){
      //int action = computeAction(khoangcach);
      Serial.print("THis is curr action: ");
      Serial.print(action);
      Serial.print("\n");
    }
    */
  if (khoangcach > gioihan || khoangcach == 0)
  {
      dithang();
  }
  
  else if (khoangcach < gioihan && khoangcach> 25)
  {
    
      //PID = tinhpid();
      //tiepcan(PID);
      int newV = computeAction(khoangcach);
      // newV = newV + 0.5*newV;
      // tunning for newV:
      if (newV > 0 && newV < 20){
        newV += 30;
      }
      Serial.print("newV: ");
      Serial.print(newV);
      Serial.print("\n");
      //Serial.print(" with distance: ");Serial.print(khoangcach);Serial.print("\n");
      tiepcan(newV);
  }
  else
  {
    resetdongco();
    quaycbsangtrai();
    delay(100);
    dokhoangcach();
    //Serial.print("THis is khoang cach: ");
    Serial.print(khoangcach);
    //Serial.print("\n");
    khoangcachtrai = khoangcach;
    quaycbsangphai();
    delay(100);
    dokhoangcach();
    //Serial.print("THis is khoang cach: ");
  //Serial.print(khoangcach);
  //Serial.print("\n");
    khoangcachphai = khoangcach;
    if (khoangcachphai < 30 && khoangcachtrai < 30) {
      dilui();
    }
    else
    {
      if (khoangcachphai >= khoangcachtrai)
      {        
        disangphai();
        delay(500);
      }
      if (khoangcachphai < khoangcachtrai)
      {
        disangtrai();
        delay(500);
      }
    }
  }
delay(500);
}
void dithang()
{

  digitalWrite(tien1, HIGH);
  digitalWrite(tien2, HIGH);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
  analogWrite(enA, 52);
  analogWrite(enB, 50);
/**
  for (int k = 0; k <256; k++)
  {
    analogWrite(enA, k);
    analogWrite(enB, k);
    delay(100);
  }
**/
}

void disangphai()
{
  resetdongco();
  digitalWrite(lui1, HIGH);
  delay(1000);//thời gian lùi
  digitalWrite(lui1, LOW);
  analogWrite(enA, 50);
  analogWrite(enB, 50);

}
void disangtrai()
{
  resetdongco();
  digitalWrite(lui2, HIGH);
  delay(1000);//thời gian 
  digitalWrite(lui2, LOW);
  analogWrite(enA, 50);
  analogWrite(enB, 50);
}

void dilui()
{
  resetdongco();
  //for (i = 0; i < 20; i++)
  digitalWrite(lui1, HIGH);
  digitalWrite(lui2, HIGH);;
  analogWrite(enA, 50);
  analogWrite(enB, 50);
  delay(1000);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

void resetdongco()
{
  digitalWrite(tien1, LOW);
  digitalWrite(tien2, LOW);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
}

double dokhoangcach()
{

  digitalWrite(trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);  
  delayMicroseconds(10); 
  digitalWrite(trig, LOW); 


  // Đo độ rộng xung HIGH ở chân echo.
  thoigian = pulseIn(echo, HIGH);

  khoangcach = thoigian / 2 / 29.412;
  return khoangcach;
  //Serial.print("Khoang cach hien tai: "); Serial.print(khoangcach);
}


void quaycbsangtrai()
{
  myservo.write(180);              // tell servo to go to position in variable 'pos'
  delay(1000);
  dokhoangcach();
  myservo.write(90);              // tell servo to go to position in variable 'pos'
}
void quaycbsangphai()
{
  myservo.write(0);              // tell servo to go to position in variable 'pos'
  delay(1000);
  dokhoangcach();
  myservo.write(90);              // tell servo to go to position in variable 'pos'
}
void resetservo()
{
  myservo.write(90);
}
double tinhpid()
{
    e = khoangcach;
    P = e;
    I = I + e;
    D = e - pe;
    pe = e;
    
    PID = (Kp*P) ;
    return PID;
    
}
void tiepcan(double PID)
{
  digitalWrite(tien1, HIGH);
  digitalWrite(tien2, HIGH);
  digitalWrite(lui1, LOW);
  digitalWrite(lui2, LOW);
  v_moi = PID;
  v_hientai = v_moi;
  analogWrite(enA, v_hientai);
  analogWrite(enB, v_hientai);
}
