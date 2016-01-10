#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#define TRUE 1

#define ARMING 18
#define CALIBRATING 23
#define ARMED 24

#define PINGER 20
#define TRIGGER 21

#define CALIBRATION_READING_COUNT 100
#define CALIBRATION_READING_DELAY 3

#define FIRE_THRESHOLD 2

#define BURST_TIME 5000

#define ARMING_MESSAGE "{ \"event\": \"arming\", \"time\":\"%s\" }\n"
#define CALIBRATING_MESSAGE "{ \"event\": \"calibrating\", \"time\":\"%s\" \"observations\":\"%d\"}\n"
#define CALIBRATION_COMPLETE_MESSAGE "{ \"event\": \"calibration complete\", \"time\":\"%s\", \"mean\":\"%f\", \"stdev\":\"%f\" }\n"
#define ARMED_MESSAGE "{ \"event\": \"armed\", \"time\":\"%s\" }\n"
#define FIRING_MESSAGE "{ \"event\": \"firing for %dms\", \"time\":\"%s\", \"threshold\":\"%f\", \"delta\":\"%f\", observed: %f, expected: %f }\n"
#define EXITING_MESSAGE "{ \"event\": \"process terminated\", \"time\":\"%s\" }\n"

struct array_stats {
   int n;
   float mean;
   float stdev;
};

struct fire_message {
  bool should_fire;
  float delta;
  float threshold;
};

struct array_stats calibrated_parameters;

char *get_current_timestamp(){
  time_t now =time(NULL);
  char *now_string =  asctime( localtime(&now) );
  return strtok(now_string,"\n");
}

void teardown() {

  digitalWrite(TRIGGER, LOW);
  pinMode(TRIGGER, INPUT);
  digitalWrite(ARMING, LOW);
  pinMode(ARMING, INPUT);
  digitalWrite(CALIBRATING, LOW);
  pinMode(CALIBRATING, INPUT);
  digitalWrite(ARMED, LOW);
  pinMode(ARMED, INPUT);
}

void sighandler(int signum)
{
   printf(EXITING_MESSAGE, get_current_timestamp());
   teardown();
   exit(1);
}

void setup() {
  wiringPiSetupGpio();
  pinMode(TRIGGER, OUTPUT);
  digitalWrite(TRIGGER, LOW);
  pinMode(ARMING, OUTPUT);
  digitalWrite(ARMING, LOW);
  pinMode(CALIBRATING, OUTPUT);
  digitalWrite(CALIBRATING, LOW);
  pinMode(ARMED, OUTPUT);
  digitalWrite(ARMED, LOW);
  signal(SIGINT,sighandler);
  signal(SIGKILL,sighandler);
  delay(30);
}

int getCM() {
  pinMode(PINGER, OUTPUT);
  digitalWrite(PINGER, HIGH);
  delayMicroseconds(5);
  digitalWrite(PINGER, LOW);
  long timeout = micros() + 50000;
  pinMode(PINGER, INPUT);

  while(digitalRead(PINGER) == LOW);
  long startTime = micros();
  while(digitalRead(PINGER) == HIGH) if(micros()>timeout) return 0;
  long travelTime = micros() - startTime;

  int distance = travelTime / 20;
  return distance;
}

struct array_stats calculate_statistics(int n, int data[n]){
  float mean=0.0, sum_deviation=0.0, stdev=0.0;
  for(int i=0; i<n;++i)
  {
    mean+=data[i];
  }
  mean=mean/n;
  for(int i=0; i<n;++i){
    float difference = data[i]-mean;
    sum_deviation+=pow(difference, 2);
  }
  stdev = sqrt(sum_deviation / n);
  struct array_stats result = {.mean = mean, .stdev=stdev, .n=n};

  return result;
}


struct array_stats collect_sensor_sample(int n, int read_delay){
  int readings[n];
  int distance;
  for(int i=0; i<n; i++)
  {
    distance = getCM();
    if(distance == 0) {
      i=0;
      continue;
    }

    readings[i] = distance;
    delay(read_delay);
  }
  struct array_stats results = calculate_statistics(n,readings);
  return results;
}

void calibrate(){
  printf(CALIBRATING_MESSAGE, get_current_timestamp(), CALIBRATION_READING_COUNT);
  digitalWrite(CALIBRATING, HIGH);
  calibrated_parameters = collect_sensor_sample(CALIBRATION_READING_COUNT, CALIBRATION_READING_DELAY);
  digitalWrite(CALIBRATING, LOW);

  printf(
    CALIBRATION_COMPLETE_MESSAGE,
    get_current_timestamp(),
    calibrated_parameters.mean,
    calibrated_parameters.stdev
  );
}


struct fire_message get_fire_determinant(struct array_stats observation){

  int delta = observation.mean - calibrated_parameters.mean;
  int abs_delta = abs((int)delta);
  float threshold = FIRE_THRESHOLD * calibrated_parameters.stdev;

  bool should_fire = abs_delta > threshold;

  struct fire_message result = {
    .should_fire = should_fire,
    .threshold = threshold,
    .delta = abs_delta
  };
  return result;
}

void show_safe_timer(){
  printf(ARMING_MESSAGE, get_current_timestamp());
  for(int i = 3; i>0;i--){
    for(int j = 0; j<i; j++){
      digitalWrite(ARMING,HIGH);
      delay(i*200);
      digitalWrite(ARMING,LOW);
      delay(200);
    }
    delay(500);
    digitalWrite(ARMING,LOW);
  }
}

int main(void) {
  struct array_stats observationA;
  struct array_stats observationB;

  struct fire_message determinationA;
  struct fire_message determinationB;

  setup();
  show_safe_timer();
  calibrate();
  digitalWrite(ARMED,HIGH);
  printf(ARMED_MESSAGE, get_current_timestamp());

  while(1==1){
    delay(15);
    observationA = collect_sensor_sample(5, 3);
    determinationA = get_fire_determinant(observationA);

    if(determinationA.should_fire){
      observationB = collect_sensor_sample(15, 1);
      determinationB = get_fire_determinant(observationB);
    }

    if(determinationA.should_fire && determinationB.should_fire){
      digitalWrite(TRIGGER, HIGH);
      printf(
        FIRING_MESSAGE,
        BURST_TIME,
        get_current_timestamp(),
        determinationB.threshold,
        determinationB.delta,
        observationB.mean,
        calibrated_parameters.mean

      );
      delay(BURST_TIME);
    }
    digitalWrite(TRIGGER, LOW);
  }
  return 0;

}
