const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
uint16_t sensors_max[SensorCount], sensors_min[SensorCount];
float norm_ss[SensorCount];
int position = 0;
int pre_pos;
#define button 4

// Define the analog pin connections for your sensors
const uint8_t sensorPins[SensorCount] = {12, 14, 27, 26, 25, 33};

float error = 0;
float error_previous = 0;
float error_integral = 0;
float error_derivative = 0;
float setpoint = 0;
float leftspeed, rightspeed;
float linear_v = 120;
float pidCal;
float dt = 0.01;

float kp = 0.1;//0.1;
float ki = 0.0000000;
float kd = 0.0028;


#define BIN1 18   // GPIO7 (Use an appropriate GPIO if 7 is unavailable)
#define BIN2 19   // GPIO8 (Use an appropriate GPIO if 8 is unavailable)
#define PWMB 21   // GPIO9 (Update to a valid ESP32 PWM-capable GPIO)
#define AIN1 22   // GPIO4
#define AIN2 23   // GPIO5
#define PWMA 15   // GPIO3 (Update to a valid ESP32 PWM-capable GPIO)
#define STBY 13   // GPIO6 (Use an appropriate GPIO if 6 is unavailable)


float previousRvalue;
float previousR1value;
float previousR2value;
float previousLvalue;
float previousL1value;
float previousL2value;

// PWM Configuration
const int freq = 5000;           // PWM frequency
const int resolution = 8;        // PWM resolution (8 bits = 0-255 range)

// PWM channels
const int pwmChannelA = 0;       // PWM channel for Motor A
const int pwmChannelB = 1;  

void setup()
{
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Attach PWM channels to motor PWM pins
  ledcAttach(PWMA, freq, resolution);  // Attach Motor A PWM
  ledcAttach(PWMB, freq, resolution);  // Attach Motor B PWM

  // Initialize motors to stop
  digitalWrite(STBY, HIGH);  // Disable standby mode
  // Set the button pin as input
  pinMode(button, INPUT);
  
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize max and min values for sensors
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    sensors_min[i] = 4096; // Set initial min to the highest possible analog value (1023 for 10-bit resolution)
    sensors_max[i] = 0;    // Set initial max to the lowest possible analog value
  }
}

void loop()
{
  int button_state = digitalRead(button);
  sensor_read();
  // Update min and max values while the button is pressed
  while (button_state == HIGH)
  {
    Serial.println("Reading sensor values...");

    // Read sensor values using analogRead() for each sensor pin
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      sensorValues[i] = 4096 - analogRead(sensorPins[i]);

      // Update the max and min sensor values
      if (sensorValues[i] > sensors_max[i])
      {
        sensors_max[i] = sensorValues[i];
      }
      if (sensorValues[i] < sensors_min[i])
      {
        sensors_min[i] = sensorValues[i];
      }
    }

    // Calculate normalized values and print them
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      // Normalize sensor values between 0 and 1
      if (sensors_max[i] != sensors_min[i]) {
        norm_ss[i] = 1.0 - ((float)(sensorValues[i] - sensors_min[i]) / (sensors_max[i] - sensors_min[i]));
      } else {
        norm_ss[i] = 1.0; // If max equals min, normalization won't change the value
      }      
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": Norm = ");
      Serial.print(norm_ss[i], 3); // Print with 3 decimal places
      Serial.print('\t');
    }
    Serial.println();

    // Update the button state to detect release
    button_state = digitalRead(button);
  }
  for (uint8_t i = 0; i < SensorCount; i++)
    {
      // // Normalize sensor values between 0 and 1
      // norm_ss[i] = 1.0 - ((float)(sensorValues[i] - sensors_min[i]) / (sensors_max[i] - sensors_min[i]));     
      // Serial.print("Sensor ");
      // Serial.print(i + 1);
      // Serial.print(": Norm = ");
      // Serial.print(norm_ss[i], 3); // Print with 3 decimal places
      // Serial.print('\t');
      norm_ss[i] = 1.0 - ((float)(sensorValues[i] - sensors_min[i]) / (sensors_max[i] - sensors_min[i]));
      if (norm_ss[i] < 0.0) {
        norm_ss[i] = 0.0; // If max equals min, normalization won't change the value
      }
      
      Serial.print("S");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(norm_ss[i], 3); // Print with 3 decimal places
      Serial.print('\t');
  }
  //Serial.println();
  position = ((-30*norm_ss[0]) + (-20*norm_ss[1]) + (-10*norm_ss[2]) + (10*norm_ss[3]) + (20*norm_ss[4]) + (30*norm_ss[5]));
  
  pre_pos = position; // position = map(position, -54, 57, -50, 50);
  if(position < - 50 )
  {
    position = -50;
  }
  else if(position > 50 )
  {
    position = 50;
  }
  Serial.print("Position : ");
  Serial.println(position);
  pid();
}


void sensor_read(){
  for (uint8_t i = 0; i < SensorCount; i++)
  {
      sensorValues[i] = 4096 - analogRead(sensorPins[i]);
  }
}

void pid()
{
  error = position - setpoint;
	error_integral += error*dt;
	error_derivative = (error - error_previous)/dt;
	error_previous = error ;
	pidCal = kp*error + ki*error_integral + kd*error_derivative;

  if (pidCal > 80) pidCal = 80;
	if (pidCal < -80) pidCal = -80;
	rightspeed = linear_v - pidCal;
	leftspeed = linear_v + pidCal;
	if (rightspeed > 200) rightspeed = 200;
	if (rightspeed < -220) rightspeed = -200;
	if (leftspeed > 200) leftspeed = 200;
	if (leftspeed < -200) leftspeed = -200;

  if(norm_ss[1]>0.5||norm_ss[2]>0.5||norm_ss[3]>0.5||norm_ss[4]>0.5){
  if(leftspeed >= 0)
  {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMB, leftspeed); 
  }
  else if(leftspeed < 0)
  {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMB, -1*leftspeed);
  }  
  if(rightspeed >= 0)
  {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, rightspeed); 
  }
  else if(rightspeed < 0)
  {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, -1*rightspeed); 
  } 
  }
  else{
    if(previousLvalue > 0.45 ){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      ledcWrite(PWMA, 0);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      ledcWrite(PWMB, 50);  
    }
    if(previousRvalue > 0.45 )
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      ledcWrite(PWMA, 50);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      ledcWrite(PWMB, 0);      
    }
  }
	 previousLvalue = norm_ss[4];
	 previousL1value = norm_ss[5];
	 previousRvalue = norm_ss[0];
	 previousR1value = norm_ss[1];
}
