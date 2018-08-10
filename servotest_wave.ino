#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include <noise.h>

#define numServos 16
#define servoPin 7
#define numCases 7
#define SERVOMIN  200
#define SERVOMAX  500 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo servoo;

uint16_t servoo_val;
uint16_t servoo_current;
uint16_t vals[numServos];
uint16_t newVals[numServos];
float phase_offset = 0.5; // distance offset between values in waves
int currentFunc = 1;
int originalFunc;
uint8_t centerx, centery;
uint16_t timeoffsetx, timeoffsety;
unsigned long funcChangeInterval = 6000;
unsigned long funcChangeTime;
unsigned long interpolateInterval = 2000;
unsigned long interpolateTime;
bool interpolateInit = false;

int numPeaks;

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
  Serial.println("16 Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60); 
  servoo.attach(servoPin);
  servoo_current = servoo.read();
  servoo_val = servoo_current;
  timeoffsetx = random(65535);
  timeoffsety = random(65535);
  funcChangeTime = millis();
  interpolateTime = 0;
  numPeaks = 1;
}

void loop() {
  for(uint8_t servonum = 0; servonum < numServos; servonum++){
    vals[servonum] = switchFunc(servonum, currentFunc);
  }

//  if(millis() > funcChangeTime + funcChangeInterval){
//    if(currentFunc < numCases) currentFunc += 1;
//    else currentFunc = 1;
//    interpolation(currentFunc);
//    funcChangeTime = millis();
//    //interpolateTime = millis();
//  }

  controlServos();

}

/*** Update position values to servos. ***/
void controlServos(){
  // Control for servo attached to digital pins
  servoo_val = map(vals[servoPin], 200, 500, 1, 180);
  servoo_current = servoo.read();
  if(servoo_current < servoo_val){
    for (int i = servoo_current; i <= servoo_val; i+=1){
      servoo.write(i);
      delay(1);
    }
  } else if (servoo_current > servoo_val){
    for (int i = servoo_current; i >= servoo_val; i-=1) {
      servoo.write(i);
      delay(1);
    }
  } else if (servoo_current == servoo_val) {
    servoo.write(servoo_current);
  }
  // Control for servos attached to motor shield
  for(uint8_t servonum = 0; servonum<numServos; servonum++){
    if(servonum != servoPin) 
      pwm.setPWM(servonum, 0, vals[servonum]);
  } 
}

/*** Huge switch statement with each case being one unique time-based animation***/
uint16_t switchFunc(uint8_t servonum, int currentFunc){
  uint16_t pos;
  switch(currentFunc){
    // Snaky sine wave
    case 1: 
      if((servonum / 4) % 2 == 0){
        pos = linear_sine(servonum*phase_offset, numPeaks, interpolateTime);
      } else {
        int row_number = servonum / 4;
        int row_index = servonum % 4;
        pos = linear_sine((3 - row_index + row_number * 4)*phase_offset, numPeaks, interpolateTime);
      }
      break;

    // Horizontal sine wave
    case 2:
      pos = linear_sine( servonum / 4, 1, interpolateTime);
      break;

    // Vertical sine wave
    case 3:
      pos = linear_sine( servonum % 4, 1, interpolateTime);
      break;

    //Combining vertical and horizontal sine waves
    case 4:
      pos = twod_sine(servonum);
      break;
      
    // 2d Noise
    case 5:
      pos = twod_noise(servonum);
      break;

    // Moving peak
    case 6:
      centerx = constrain(inoise8(millis() + timeoffsetx), 0, 200);
      centery = constrain(inoise8(millis() + timeoffsety), 0, 200);
      centerx = map(centerx, 0, 200, 0, 3);
      centery = map(centery, 0, 200, 0, 3);
      pos = moving_peak(servonum, centerx, centery);
      break;
      
    case 7:
      pos = bouncing(servonum, 'x'); // 'x' or 'y'
      break;

    // Reset and calibrate
    default:
      pos = 200;
  }
  return pos;
}

//uint16_t smoothing(uint8_t index, uint8_t newVal){
//  uint8_t current = vals[index];
//  
//}

/*** Give servos small amount of time to get ready for next position, in order to avoid big jumps between position values of different animations ***/
// Here, currentFunc stands for the function number of next animation
void interpolation(int currentFunc){
  delay(500);
  unsigned long initi = millis();
  bool updated[numServos];
  // get new position values
  interpolateTime = 0;
  for(uint8_t servonum = 0; servonum < numServos; servonum++){
    newVals[servonum] = switchFunc(servonum, currentFunc);
    updated[servonum] = false;
  }
  // increment or decrement vals slowly
  while(!updated[0] || !updated[1] || !updated[2] || !updated[3] || !updated[4] || !updated[5] || !updated[6] || !updated[7] || !updated[8] || !updated[9] || !updated[10] || !updated[11] || !updated[12] || !updated[13] || !updated[14] || !updated[15]){ //loop until all position values are updated
    for(uint8_t servonum = 0; servonum < numServos; servonum++){
      if(vals[servonum] < newVals[servonum]){
        vals[servonum]++;
      } else if (vals[servonum] > newVals[servonum]){
        vals[servonum]--;
      } else if (vals[servonum] == newVals[servonum]){
        updated[servonum] = true;
      }
    }
    controlServos();
  }
  interpolateTime = millis() - initi;
}


/** Mathematical transformation (animation) functions **/

// linear sine wave function
// half wavelength
uint16_t linear_sine(float phase, int peaks, unsigned long timeoffset){
  double sine = sin((millis()-timeoffset)*PI/1000.0*0.2 + phase*2.0*float(peaks)); //0.2 controls rate of sine wave travel
  double mapped_sine = (sine - (-1)) * (SERVOMAX - SERVOMIN) / (1 - ( - 1)) + SERVOMIN;
  uint16_t int_sine = (unsigned int)mapped_sine;
  return int_sine;
}

uint16_t twod_sine(int servonum) {
  int row_number = servonum / 4;
  int row_index = servonum % 4;
  double sinex = sin(millis() * PI / 1000.0 * 0.2 + row_index) / 2;
  double siney = sin(millis() * PI / 1000.0 * 0.2 + row_number) / 2;
  double sine = sinex + siney;
  double mapped_sine = (sine - (-1)) * (SERVOMAX - SERVOMIN) / (1 - ( - 1)) + SERVOMIN;
  uint16_t int_sine = (unsigned int)mapped_sine;
  return int_sine;
}

//decrease update rate or decrease noise range
uint16_t twod_noise(uint8_t index) {
  uint16_t row_number = index / 4;
  uint16_t row_index = index % 4;
  uint8_t noise_val = inoise8(row_index, row_number, millis()/10);
  uint16_t int_noise = map((unsigned int)noise_val, 0, 255, SERVOMIN+200, SERVOMAX);
  return int_noise;
}

// assuming that SERVOMIN will pull the string up the most
// add lerping between future and present values
uint16_t moving_peak(uint8_t index, uint8_t centerx, uint8_t centery){
  uint16_t row_number = index / 4;
  uint16_t row_index = index % 4;
  uint8_t dist = max(abs(row_index-centerx), abs(row_number-centery));
  uint16_t pos;
  switch(dist){
    case 0:
      // u are the center
      pos = SERVOMIN;
      break;
    case 1:
      // first layer
      pos = 300;
      break;
    case 2:
      // second layer
      pos = 400;
      break;
    case 3:
      // third layer
      pos = SERVOMAX;
      break;
    default:
      pos = SERVOMAX;
  }
  return pos;
}

uint16_t bouncing (uint8_t index, char dir){
  uint16_t row_number = index / 4;
  uint16_t row_index = index % 4;
  uint16_t pos;
  if(dir == 'x'){
    if(row_index == 1 || row_index == 2){
      pos = sin(millis()*PI/1000.0*0.2);
    } else {
      pos = -sin(millis()*PI/1000.0*0.2);
    }
  } else if (dir == 'y') {
    if(row_number == 1 || row_number == 2){
      pos = sin(millis()*PI/1000.0*0.2);
    } else {
      pos = -sin(millis()*PI/1000.0*0.2);
    }
  } else {
    return 0;
  }
  pos = map(pos, -1, 1, 200, 500);
  return pos;
}

//uint16_t circular_wave(uint8_t index){
//  
//}

