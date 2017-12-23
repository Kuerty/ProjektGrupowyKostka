
#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

HMC5883L compass;

MPU6050 mpu;

SoftwareSerial bluetooth(10, 11); // RX, TX



float headingDegrees, N_headingDegrees;
Vector normAccel;

//debouncing
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
bool read1,read2,read3,read4,read5,read6;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
int button_state=HIGH;

//roll yaw variables
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float roll = 0;
float yaw = 0;

double N_rawX, N_rawY, N_rawZ, rawX, rawY, rawZ;    //global decrale magnetometer values after noise fix
void magnetometer_noise_modeling(Vector magneto_raw);

int debounce(int port_number);
int calculate_top_wall(Vector accel);
float calculate_heading(double magneto_X, double magneto_Y, double magneto_Z, int top_wall);
int calculate_north_wall(int up_wall, float kompas);


void setup() {
  
  //piny do ktorych nalerzy podlaczyc przyciski
  pinMode(9,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  
  Serial.begin(115200);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  bluetooth.begin(115200);

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  //mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  //mpu.setThreshold(1.5);

  compass.setRange(HMC5883L_RANGE_0_88GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  //compass.setOffset(0, 0);
}


void loop() {

  //odczyt akcelerometru
  normAccel = mpu.readNormalizeAccel();
  
  //odczyt danych z magnetometru
  Vector norm = compass.readNormalize();
  Vector raw = compass.readRaw();
  rawX = raw.XAxis;
  rawY = raw.YAxis;
  rawZ = raw.ZAxis;

  //wylicz ktora sciana jest na gorze
  int up_wall = calculate_top_wall(normAccel); 
  
  //macierz usuwania szumow magnetometru
  magnetometer_noise_modeling(raw);
  
  //wylicz pomiar z kompasu przed usunieciem szumow
  float heading = calculate_heading(rawX, rawY,rawZ, up_wall);
  //wylicz pomiar z kompasu po usunieciem szumow
  float head_N = calculate_heading(N_rawX, N_rawY, N_rawZ, up_wall);
  // Convert to degrees
  headingDegrees = heading * 180/M_PI; 
  N_headingDegrees = head_N * 180/M_PI; 

  //wylicz sciane skierowna do polnocy
  int north_wall = calculate_north_wall(up_wall, headingDegrees);

  read1 = debounce(9);
  read2 = debounce(8);
  read3 = debounce(7);
  read4 = debounce(12);
  read5 = debounce(13);
  read6 = debounce(0);


    
    //odkomentowac do odczytu danych z zyroskopu
    /*
    timer = millis();
    Vector normal = mpu.readNormalizeGyro();
    //if(button_state ==0){
    roll += normal.XAxis * timeStep;
    yaw += normal.ZAxis * timeStep;
    //}
    //else{
    //  roll =0;
    //  yaw=0;
    //}
    */

    
    // odkomentowac aby dane szly po USB
    
    Serial.print(up_wall);              //obliczone ktora sciana jest na gorze
    Serial.print(";");
    //Serial.print(north_wall);   //obliczone ktora sciana skierowana do polnocy
    //Serial.print(pitch);            
    //Serial.print(roll);
    //Serial.print(yaw));
    //Serial.print(norm.XAxis);      //akcelerometr os X - znormalizowany
    //Serial.print(norm.YAxis);      //akcelerometr os Y - znormalizowany
    //Serial.print(norm.ZAxis);      //akcelerometr os Z - znormalizowany
    //Serial.prinet(headingDegrees); //kompas przed usunieciem szumow
    //Serial.prinet(N_headingDegrees); //kompas po usunieciu szumow
    Serial.print(N_rawX);          //magnetometr os X - surowe po usunieciu szumow
    Serial.print(";");
    Serial.print(N_rawY);          //magnetometr os Y - surowe po usunieciu szumow
    Serial.print(";");
    Serial.print(N_rawZ);          //magnetometr os Z - surowe po usunieciu szumow
    Serial.print(";");
    //Serial.print(raw.XAxis);          //magnetometr os X - surowe przed usunieciem szumow
    //Serial.print(raw.YAxis);          //magnetometr os Y - surowe przed usunieciem szumow
    //Serial.print(raw.ZAxis);          //magnetometr os Z - surowe przed usunieciem szumow
    Serial.print(digitalRead(9));         //stan przycisku na pinie 9
    Serial.print(";");
    Serial.print(digitalRead(8));       //stan przycisku na pinie 8 itd...
    Serial.print(";");
    Serial.print(digitalRead(7));
    Serial.print(";");
    Serial.print(digitalRead(6));
    Serial.print(";");
    Serial.print(digitalRead(5));
    Serial.print(";");
    Serial.print(digitalRead(4));
    Serial.println();
    
    //odkomentowac aby dane szly po bluetooth 
    //UWAGA! Parowanie! Urządzenie:Kostka   PIN:2016
    
    bluetooth.print(up_wall);              //obliczone ktora sciana jest na gorze
    bluetooth.print(";");
    //bluetooth.print(north_wall);   //obliczone ktora sciana skierowana do polnocy
    //bluetooth.print(pitch);            
    //bluetooth.print(roll);
    //bluetooth.print(yaw));
    //bluetooth.print(norm.XAxis);      //akcelerometr os X - znormalizowany
    //bluetooth.print(norm.YAxis);      //akcelerometr os Y - znormalizowany
    //bluetooth.print(norm.ZAxis);      //akcelerometr os Z - znormalizowany
    //bluetooth.print(headingDegrees); //kompas przed usunieciem szumow
    //bluetooth.print(N_headingDegrees); //kompas po usunieciu szumow
    bluetooth.print(N_rawX);          //magnetometr os X - surowe po usunieciu szumow
    bluetooth.print(";");
    bluetooth.print(N_rawY);          //magnetometr os Y - surowe po usunieciu szumow
    bluetooth.print(";");
    bluetooth.print(N_rawZ);          //magnetometr os Z - surowe po usunieciu szumow
    bluetooth.print(";");
    //bluetooth.print(raw.XAxis);          //magnetometr os X - surowe przed usunieciem szumow
    //bluetooth.print(raw.YAxis);          //magnetometr os Y - surowe przed usunieciem szumow
    //bluetooth.print(raw.ZAxis);          //magnetometr os Z - surowe przed usunieciem szumow
    bluetooth.print(digitalRead(9));         //stan przycisku na pinie 9
    bluetooth.print(";");
    bluetooth.print(digitalRead(8));     //stan przycisku na pinie 8 itd...
    bluetooth.print(";");
    bluetooth.print(digitalRead(7));
    bluetooth.print(";");
    bluetooth.print(digitalRead(6));
    bluetooth.print(";");
    bluetooth.print(digitalRead(5));
    bluetooth.print(";");
    bluetooth.println(digitalRead(4));
    bluetooth.println(); 
    
}

//ponizej deklaracja funkcji

void magnetometer_noise_modeling(Vector magneto_raw){
    
  const double d =2048;
  const double A1 [3] = { 1.7312, 0.0321, 0.0145 };
  const double A2 [3] = { 0.0321, 1.6029, -0.494 };
  const double A3 [3] = { 0.0145, -0.0494, 1.9021 };
  const double b [3] = { 0.0258, -0.0821, -0.0042};
  double h_b [3];
  
  magneto_raw.XAxis = magneto_raw.XAxis/d;
  magneto_raw.YAxis = magneto_raw.YAxis/d;
  magneto_raw.ZAxis = magneto_raw.ZAxis/d;


  h_b [0] = magneto_raw.XAxis - b[0];
  h_b [1] = magneto_raw.YAxis - b[1];
  h_b [2] = magneto_raw.ZAxis - b[2];

  
  N_rawX = A1[0]*h_b[0];
  N_rawX += A1[1]*h_b[1];
  N_rawX += A1[2]*h_b[2];

  N_rawY = A2[0]*h_b[0];
  N_rawY += A2[1]*h_b[1];
  N_rawY += A2[2]*h_b[2];
  
  N_rawZ = A3[0]*h_b[0];
  N_rawZ += A3[1]*h_b[1];
  N_rawZ += A3[2]*h_b[2];
};

int calculate_top_wall(Vector accel){
    float max_acc;
    int up_wall;
    max_acc = accel.XAxis;
    up_wall = 1;
    if(abs(max_acc) < abs(accel.YAxis)){
      max_acc = accel.YAxis;
      up_wall =3;
    }
    if(abs(max_acc) < abs(accel.ZAxis)){
      max_acc = accel.ZAxis;
      up_wall=5;
    }
    if (max_acc < 0)
      up_wall += 1;
    return up_wall;
};


float calculate_heading(double magneto_X, double magneto_Y, double magneto_Z, int top_wall){
      double compass_offset;
      float heading;
      switch (top_wall){
      case 1:
        compass_offset = 1.8;
        heading = atan2(magneto_X, magneto_Z);    
        break;
      case 2:
        compass_offset = 0.2;                 
        heading = atan2(magneto_Z, magneto_X);             
        break;
      case 3:
        compass_offset =-1.6;          
        heading = atan2(magneto_Z, magneto_Y);    
        break;
      case 4:
        compass_offset =0.12;      
        heading = atan2(magneto_Y, magneto_Z);
        break;
      case 5:      
        compass_offset = 0;
        heading = atan2(magneto_Y, magneto_X);
        break;
      case 6:
        compass_offset=0.80;             
        heading = atan2(magneto_X, magneto_Y);
        break;
      }
      float declinationAngle = (5.0 + (22.0 / 60.0)) / (180 / M_PI);    //stala deklinacji dla gdanska
      heading += declinationAngle;                                      // Correct for heading < 0deg and heading > 360deg

      if (heading < 0)
      {
        heading += 2 * PI;
      }
      if (heading > 2 * PI)
      {
        heading -= 2 * PI;
      }
      
      return heading;
};

int calculate_north_wall(int up_wall, float kompas){
  int which_one_north;
  // when 5 up etc..
  if (up_wall ==5 && kompas > 115 && kompas < 190)
    which_one_north = 3;
  if (up_wall ==5 && (kompas > 317 || kompas < 45))
    which_one_north = 4;
  if (up_wall ==5 && kompas > 190  && kompas < 317)
    which_one_north = 2;
  if (up_wall ==5 && kompas > 45  && kompas < 115)
    which_one_north = 1;

  //when 3 up
  if (up_wall ==3 && kompas > 190 && kompas < 303 )
    which_one_north = 2;
  if (up_wall ==3 && kompas > 62 && kompas < 120)
    which_one_north = 1;
  if (up_wall ==3 && (kompas > 303  || kompas < 62))
    which_one_north = 2;
  if (up_wall ==3 && kompas > 120  && kompas < 190)
    which_one_north = 2;

  //when 4 up  
  if (up_wall ==4 && kompas > 180 && kompas < 315)
    which_one_north = 2;
  if (up_wall ==4 && kompas > 57 && kompas < 104)
    which_one_north = 1;
  if (up_wall ==4 && kompas > 104  && kompas < 180)
    which_one_north = 5;
  if (up_wall ==4 && (kompas > 315  || kompas < 57))
    which_one_north = 6;

  
  //when 2 up  
  if (up_wall ==2 && kompas > 175 && kompas < 255)
    which_one_north = 3;
  if (up_wall ==2 && kompas > 255 && kompas < 315)
    which_one_north = 5;
  if (up_wall ==2 && kompas > 40  && kompas < 175)
    which_one_north = 6;
  if (up_wall ==2 && (kompas > 315 || kompas < 40))
    which_one_north = 4;
    
  //when 1 up  
  if (up_wall ==1 && kompas > 170 && kompas < 230)
    which_one_north = 3;
  if (up_wall ==1 && kompas > 35 && kompas < 170)
    which_one_north = 5;
  if (up_wall ==1 && kompas > 230  && kompas < 300)
    which_one_north = 6;
  if (up_wall ==1 && (kompas > 300  || kompas < 35))
    which_one_north = 4;

  //when 6 up  
  if (up_wall ==6 && kompas > 280 && kompas < 340)
    which_one_north = 1;
  if (up_wall ==6 && kompas > 50 && kompas < 150)
    which_one_north = 2;
  if (up_wall ==6 && kompas > 150  && kompas < 280)
    which_one_north = 3;
  if (up_wall ==6 && (kompas > 340  || kompas < 50))
    which_one_north = 4;

  return which_one_north;
};

 int debounce(int port_number){
 
    //debouncing
    int reading = digitalRead(port_number);
    if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
  
      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;
  
        // only toggle the LED if the new button state is HIGH
        button_state = reading;
        }
    }
    lastButtonState = reading;

    return button_state;
 };
 

