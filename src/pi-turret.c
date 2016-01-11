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

#define VERBOSE false

#define PINGER 20
#define TRIGGER 21

#define CALIBRATION_READING_COUNT 100
#define CALIBRATION_READING_DELAY 10

#define FIRE_THRESHOLD 2
#define OUTLIER_THRESHOLD 100


#define BURST_TIME 5000

#define ARMING_MESSAGE "{ \"event\": \"arming\", \"time\":\"%s\" }\n"
#define CALIBRATING_MESSAGE "{ \"event\": \"calibrating\", \"time\":\"%s\" \"observations\":\"%d\"}\n"
#define CALIBRATION_COMPLETE_MESSAGE "{ \"event\": \"calibration complete\", \"time\":\"%s\", \"mean\":\"%f\", \"stdev\":\"%f\",  \"outliers\":\"%d\"}\n"
#define ARMED_MESSAGE "{ \"event\": \"armed\", \"time\":\"%s\" }\n"
#define FIRE_DETERMINATION_MESSAGE  "{ \"event\": \"fire determination made\", \"should fire\":\"%s\", \"observed mean distance\":\"%f\",  \"stdev\":\"%f\", \"outliers\":\"%d\" , \"expected\": \"%f\", \"threshold\":\"%f\", \"delta\":\"%f\"}\n"
#define FIRING_MESSAGE "{ \"event\": \"firing for %dms\", \"spacer\":\"*****************************************************************************************************\" \"time\":\"%s\"}\n"
#define EXITING_MESSAGE "{ \"event\": \"process terminated\", \"time\":\"%s\" }\n"

#define READING_MESSAGE "{ \"event\": \"read distance\", \"time\":\"%s\", \"value\":\"%d\" }\n"

struct array_stats {
   int n;
   float mean;
   float stdev;
   int outliers;
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
  pinMode(PINGER, OUTPUT);
  digitalWrite(PINGER, LOW);
  pinMode(PINGER, INPUT);
}

void sighandler(int signum)
{
   printf(EXITING_MESSAGE, get_current_timestamp());
   teardown();
   exit(1);
}

long last_reading_time = -1;
long last_difference = -1;
bool pingLive = false;
int pingsReceived = 0;

void handle_pinger_change() {
  int now = micros();
  if(!pingLive)return;

  pingsReceived++;
  last_difference = now - last_reading_time;
  last_reading_time = now ;
  if(VERBOSE) printf(" diff: %ld \n", last_difference);

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

  pinMode(PINGER, OUTPUT);
  // digitalWrite(PINGER, HIGH);
  // digitalWrite(PINGER, LOW);
  wiringPiISR (PINGER, INT_EDGE_BOTH,  handle_pinger_change);
  // piHiPri(10);
  delay(30);
}

int get_reading() {
  long timeout = millis() + 15;

  int distance = 0;

  pingsReceived = 0;
  if(VERBOSE) printf("begin ping %d\n", 0);
  //ping
  pinMode(PINGER, OUTPUT);
  digitalWrite(PINGER, HIGH);
  delayMicroseconds(5);
  digitalWrite(PINGER, LOW);
  pinMode(PINGER, INPUT);
  if(VERBOSE) printf("end ping %d\n", 0);
  long now = micros();

  delayMicroseconds(100);
  last_reading_time =  now;
  pingLive = true;

  //pong

  while(millis()<timeout){
    if(pingsReceived < 2){
        if(VERBOSE)printf("wait\n");
        delayMicroseconds(500);
        continue;
    } else pingLive = false;
    if(VERBOSE) printf("do calculation on: %ld\n", last_difference);
    distance = last_difference;
    if(VERBOSE)
      printf(READING_MESSAGE, get_current_timestamp(), distance);
    return distance;
  }
  pingLive = false;
  return 0;
}

struct array_stats calculate_statistics(int n, int data[n]) {
  struct array_stats result = {.mean = data[0], .stdev=0, .n=1, .outliers=0 };
  if (n<2) {
    return result;
  }
  float mean = 0.0,
    p_stdev = 0.0,
    delta = 0.0,
    new_delta = 0.0;


  short k = 0;
  short outliers = 0;

  for(int i = 0; i<n; i++){
      k=i+1;
      delta = data[i] - mean;
      mean += (delta / k);
      new_delta = data[i] - mean;
      p_stdev += (delta * new_delta);

      if(abs(delta) > p_stdev * OUTLIER_THRESHOLD)outliers++;
  }

  result.mean = mean;
  result.stdev=p_stdev / n;
  result.n=n;
  result.outliers=outliers;

  return result;
}

struct array_stats calculate_statistics_old(int n, int data[n]){
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
    distance = get_reading();
    if(distance == 0) {
      i=0;
      delay(read_delay);
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
    calibrated_parameters.stdev,
    calibrated_parameters.outliers
  );
}


struct fire_message get_fire_determinant(struct array_stats observation){

  int delta = observation.mean - calibrated_parameters.mean;
  int abs_delta = abs((int)delta);
  float threshold = FIRE_THRESHOLD * calibrated_parameters.stdev;

  bool should_fire =
    abs_delta > threshold &&
    observation.outliers < 2 ;
    // &&
    // observation.stdev < calibrated_parameters.mean;

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
      observationB = collect_sensor_sample(30, 3);
      determinationB = get_fire_determinant(observationB);
      printf(
        FIRE_DETERMINATION_MESSAGE,
        determinationB.should_fire ? "YES": "NO",
        observationB.mean,
        observationB.stdev,
        observationB.outliers,
        calibrated_parameters.mean,
        determinationB.threshold,
        determinationB.delta
      );
    }

    if(determinationA.should_fire && determinationB.should_fire ){
      digitalWrite(TRIGGER, HIGH);
      printf(
        FIRING_MESSAGE,
        BURST_TIME,
        get_current_timestamp()
      );
      delay(BURST_TIME);
    }
    digitalWrite(TRIGGER, LOW);
  }
  return 0;

}
