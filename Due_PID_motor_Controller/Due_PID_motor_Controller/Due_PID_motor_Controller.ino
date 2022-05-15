//created by ATLIN ANDERSON [ABA Industries] with code modified from curio-res
// see video https://www.youtube.com/watch?v=3ozgxPi_tl0 [How to control multiple DC motors with encoders]
//BOARD: AI THINKER ESP32-CAM
//ESP32 CAM board used

// A class to compute the control signal
class SimplePID {
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
    // Constructor
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

    // A function to set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // A function to compute the control signal
    void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
      // error
      int e = target - value;

      // derivative
      float dedt = (e - eprev) / (deltaT);

      // integral
      eintegral = eintegral + e * deltaT;

      // control signal
      float u = kp * e + kd * dedt + ki * eintegral;

      // motor power
      pwr = (int) fabs(u);
      if ( pwr > umax ) {
        pwr = umax;
      }

      // motor direction
      dir = 1;
      if (u < 0) {
        dir = -1;
      }

      // store previous error
      eprev = e;
    }

};

// How many motors
#define NMOTORS 4

//motor assignments
//FRONT
//M1  M2
//
//M3  M4
//BACK

//{M1, M2, M3, M4};

// Pins
const int enca[] = {41, 43, 45, 47};
const int encb[] = {40, 42, 44, 46 };
const int pwm[] = {2, 3, 4, 5};
const int in1[] = {24, 28, 31, 27};
const int in2[] = {26, 30, 29, 25};

// Globals
long prevT = 0;
int posPrev[] = {0, 0, 0, 0};

// Use the "volatile" directive for variables
// used in an interrupt
volatile int posi[] = {0, 0, 0, 0};

float v1Filt [] = {0, 0, 0, 0};
float v1Prev [] = {0, 0, 0, 0};

//stores when to output to serial
unsigned long previousMillis = 0;

const int dc_motor_max_rpm = 300;

float vt [] = {0, 0, 0, 0};


//read data from ESP32cam and decode to instruct motor direction and speed
void decode_serial_command(String command)
{
  int motor_speed_directon = 0;

  int motor_counter = 0;

  //check if valid command
  if (command != "-1" || command != "")
  {
    //first char is ieqal to D (for ID purposes)
    if (command[0] == 'D')
    {
      //Serial.println(command[0]);

      for (int i = 1; i < command.length(); i++)
      {
        //reset to zero
        motor_speed_directon = 0;

        //check for negative symbol
        if (command[i] == '-')
        {
          i++;
          motor_speed_directon = -(String(command[i])).toInt();
        }
        else
        {
          motor_speed_directon = (String(command[i])).toInt();
        }

        //scale to motor RPM
        motor_speed_directon = constrain(motor_speed_directon, -9, 9);
        motor_speed_directon = map(motor_speed_directon, -9, 9, -dc_motor_max_rpm, dc_motor_max_rpm);

        if (((sizeof(vt) / sizeof(float))) > motor_counter)
        {
          vt[motor_counter] = float(motor_speed_directon);

          //          Serial.println(vt[motor_counter]);
          //
          //          delay(1000);

          motor_counter++;
        }
      }
    }
  }
}




// PID class instances
SimplePID pid[NMOTORS];

void setup()
{
  Serial.begin(9600);
  Serial3.begin(9600);

  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);

    pid[k].setParams(3, 0.025, 0, 255);
  }
  enable_interrupts();
}

void loop() {
  // TIMING VARIABLE
  unsigned long currentMillis = millis();

  //serial read variable
  String command = "";

  // Read the position/////////////////////////////////////////////////////////////
  int pos[NMOTORS];

  if (Serial3.available() >= 0)
  {
    //noInterrupts(); // disable interrupts temporarily while readin
    disable_interrupts();

    //read the incoming byte:
    command = Serial3.readString();

    for (int k = 0; k < NMOTORS; k++)
    {
      pos[k] = posi[k];
    }
    enable_interrupts();
  }
  else
  {
    //noInterrupts(); // disable interrupts temporarily while readin
    disable_interrupts();
    for (int k = 0; k < NMOTORS; k++)
    {
      pos[k] = posi[k];
    }
    enable_interrupts();
  }

  if (command != "" || command != "-1")
  {
    decode_serial_command(command);
  }

  //interrupts(); // turn interrupts back on


  // time difference/////////////////////////////////////////////////////////////////
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  int velocity[NMOTORS];
  for (int k = 0; k < NMOTORS; k++)
  {
    velocity[k] = (pos[k] - posPrev[k]) / deltaT;
    posPrev[k] = pos[k];
  }
  prevT = currT;

  //convert  velocity to RPM with a low pass 25hz filter/////////////////////////////
  float v1[NMOTORS];
  for (int k = 0; k < NMOTORS; k++)
  {
    // Convert count/s to RPM
    v1[k] = velocity[k] / (16 * 23.1) * 60.0;

    // Low-pass filter (25 Hz cutoff)
    v1Filt[k] = 0.854 * v1Filt[k] + 0.0728 * v1[k] + 0.0728 * v1Prev[k];
    v1Prev[k] = v1[k];
  }

  // loop through the motors
  for (int k = 0; k < NMOTORS; k++)
  {
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(v1Filt[k], vt[k], deltaT, pwr, dir);
    // signal the motor
    setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
  }

  //  Serial.println(command);
  //  delay(3);
  //  if (currentMillis - previousMillis >= 1000) {
  //    // save the last time you blinked the LED
  //    previousMillis = currentMillis;
  //    for (int k = 0; k < NMOTORS; k++) {
  //      Serial.print(vt[k]);
  //      Serial.print(" ");
  //      Serial.print(v1Filt[k]);
  //      Serial.print(" ");
  //    }
  //    Serial.println();
  //  }

}

void enable_interrupts()
{
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
}

void disable_interrupts()
{
  detachInterrupt(digitalPinToInterrupt(enca[0]));
  detachInterrupt(digitalPinToInterrupt(enca[1]));
  detachInterrupt(digitalPinToInterrupt(enca[2]));
  detachInterrupt(digitalPinToInterrupt(enca[3]));

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) {
    posi[j]++;
  }
  else {
    posi[j]--;
  }
}
