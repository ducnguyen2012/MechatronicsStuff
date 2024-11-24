#include <Servo.h>
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position


// duc code start
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_DISTANCE 12   // Maximum distance from the obstacle
#define MAX_SPEED 12        // Maximum car speed
#define GAMMA 0.4          // Discount factor
#define THRESHOLD 1e-4     // Convergence threshold

// Reward function
double reward(int distance, int speed) {
    if (distance < 3) return -100; // Collision penalty
    if (distance == 3) return 100;
    if (speed > distance) return -10;    // Speeding penalty
    return 10 - speed * 0.5;             // Safe speed reward
}

// Transition function
void next_state(int distance, int speed, const char* action, int* new_distance, int* new_speed) {
    int new_speed_temp = speed;
    if (strcmp(action, "slow_down") == 0) new_speed_temp = fmax(0, speed - 1);
    else if (strcmp(action, "speed_up") == 0) new_speed_temp = fmin(MAX_SPEED, speed + 1);

    *new_distance = fmax(0, distance - new_speed_temp);
    *new_speed = new_speed_temp;
}

// Value iteration
void value_iteration(const char* actions[], int action_count, double V[MAX_DISTANCE + 1][MAX_SPEED + 1]) {
    while (1) {
        double delta = 0.0; // Track max change in value
        double new_V[MAX_DISTANCE + 1][MAX_SPEED + 1] = {0}; // New value table

        for (int distance = 0; distance <= MAX_DISTANCE; ++distance) {
            for (int speed = 0; speed <= MAX_SPEED; ++speed) {
                if (distance == 0) continue; // Skip terminal state

                double max_value = -1e9; // Initialize with a large negative value
                for (int i = 0; i < action_count; ++i) {
                    int next_distance, next_speed;
                    next_state(distance, speed, actions[i], &next_distance, &next_speed);
                    double value = reward(distance, speed) + GAMMA * V[next_distance][next_speed];
                    max_value = fmax(max_value, value);
                }

                new_V[distance][speed] = max_value;
                delta = fmax(delta, fabs(new_V[distance][speed] - V[distance][speed]));
            }
        }

        memcpy(V, new_V, sizeof(new_V));
        if (delta < THRESHOLD) break; // Stop if converged
    }
}

int computeAction(int distance) {
    const char* actions[] = {"slow_down", "maintain_speed", "speed_up"};
    double V[MAX_DISTANCE + 1][MAX_SPEED + 1] = {0}; // Value table
    Serial.print("compute action!");
    value_iteration(actions, 3, V);

    // Print results
    int res = -9999;
    int finalSpeed = -9999;
    //int finalDistance = -9999;
    // int distance;
    // printf("Enter your current speed: ");
    // scanf("%d", &distance);
    for (int speed = 0; speed <= MAX_SPEED; ++speed) {
        if (res < V[distance][speed]) {
            res = V[distance][speed];
            //finalDistance = distance;
            finalSpeed = speed;
        }
    }
    //printf("final: %d and speed: %d\n", finalDistance, finalSpeed);

    return finalSpeed;
}





// duc code end

const int trig = 6;  //doi tu 6 thanh 3
const int echo = 4;//5    
int tien1 = 10;       
int tien2 = 11;        
int lui1 = 12;        
int lui2 = 13;     
int enA = 3; //doi tu 3 thanh 6
int enB = 5;   
int dongcoservo = 9;   


double e, P, I, D, PID;
double pe = 0;
double Kp = 1.0;
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
  Serial.print("THis is khoang cach: ");
  Serial.print(khoangcach);
  Serial.print("\n");
  if (khoangcach <= 10){
      Serial.print("khoang cach in 40");
      Serial.print("\n");
      int action = computeAction(khoangcach);
      Serial.print("THis is curr action: ");
      Serial.print(action);
      Serial.print("\n");
    }
  if (khoangcach > gioihan || khoangcach == 0)
  {
      dithang();
  }
  
  else if (khoangcach < gioihan && khoangcach>6)
  {
    
      //PID = tinhpid();
      //tiepcan(PID);
  }
  else
  {
    resetdongco();
    quaycbsangtrai();
    dokhoangcach();
    khoangcachtrai = khoangcach;
    quaycbsangphai();
    dokhoangcach();
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
  analogWrite(enA, 100);
  analogWrite(enB, 100);
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
  analogWrite(enA, 100);
  analogWrite(enB, 100);

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
  v_moi =  v_hientai - PID;
  v_hientai = v_moi;
  analogWrite(enA, v_hientai);
  analogWrite(enB, v_hientai);
}
