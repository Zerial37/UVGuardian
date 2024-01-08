#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <PeakDetection.h>
#include <SoftwareSerial.h>


/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
PeakDetection peakDetection;

SoftwareSerial BTserial(2, 3);

int Steps;
float step_size = 0.00075;
unsigned long last_time;
int old_peak;
const int size = 20;
float mean[size];
float threshold = 9.81;
int counter;

int latestUV = analogRead(7)/10;
unsigned short exposure_times[5];

char User;


void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch(accel.getDataRate())
  {
    case ADXL343_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch(accel.getRange())
  {
    case ADXL343_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL343_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL343_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL343_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

void setup(void)
{
  Serial.begin(9600);
  BTserial.begin(9600);
  peakDetection.begin(25, 5, 0.0);

  while (!Serial);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_16_G);
  // accel.setRange(ADXL343_RANGE_8_G);
  // accel.setRange(ADXL343_RANGE_4_G);
  // accel.setRange(ADXL343_RANGE_2_G);

  /* Display some basic information on this sensor */
  accel.printSensorDetails();
  displayDataRate();
  displayRange();
  Serial.println("");
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  float mag = sqrt(event.acceleration.x*event.acceleration.x + event.acceleration.y*event.acceleration.y + event.acceleration.z*event.acceleration.z);
  unsigned long current_time = millis();

  if (counter < size) {
    mean[counter] = mag;
  }
  else {
    for (int i = 0; i < size; i++) 
      threshold += mean[i];
    threshold /= size + 1;
    counter = 0;
  }

  counter ++;
  mag = mag - threshold;

  if (mag >= 1 || mag <= -1) {
  peakDetection.add(mag);                     // adds a new data point
  int new_peak = peakDetection.getPeak();          // 0, 1 or -1
  if (new_peak == -1 && old_peak != -1) {
    if (current_time - last_time >= 200 && current_time - last_time <= 2000){
      Steps++;
      old_peak = new_peak;
      last_time = current_time;
    }
    else{
      last_time = current_time;
    }
  }
  else {
    old_peak = 0;
  }
  }

  float Distance_km = Steps * step_size;

  int uv_sensor = analogRead(7)/10;
  float battery = analogRead(3);
  float batvalue = map(battery, 0, 1023, 0, 440);

  if (uv_sensor != latestUV) {      // check if uv index has changed
    latestUV = uv_sensor;
    BTserial.print("UV index: ");
    BTserial.print(uv_sensor);
    if (latestUV <= 2) {
      BTserial.println(". UV index has changed to low! It is safe to spend time outside");
    }
    else if (uv_sensor > 2 && uv_sensor <= 5) {
      
      BTserial.println(". UV index has changed to moderate! Use sunscreen and stay in the shade near midday!");
    }
    else if (uv_sensor > 5 && uv_sensor <= 7) {
     
      BTserial.println(". UV index has changed to high! Use sunscreen and reduce time in the sun!");
    }
    else if (uv_sensor > 7 && uv_sensor <= 10) {
      BTserial.println(". UV index has changed to very high! Use sunscreen and avoid the sun between 10am and 2pm!");
    }
    else if (uv_sensor > 10) {
      BTserial.println(". UV index has changed to extremely high! Use sunscreen and avoid the sun!");
    }
  }

  if (uv_sensor <= 2) {
    exposure_times[0] += 81;
  }
  else if (uv_sensor > 2 && uv_sensor <= 5) {
    exposure_times[1] += 81;
  }
  else if (uv_sensor > 5 && uv_sensor <= 7) {
    exposure_times[2] += 81;
  }
  else if (uv_sensor > 7 && uv_sensor <= 10) {
    exposure_times[3] += 81;
  }
  else if (uv_sensor > 10) {
    exposure_times[4] += 81;
  }

  User = BTserial.read();
  
  if (User == 'Y'){
      BTserial.println("Exposure times(min): ");
      BTserial.print("Low(0-2): ");
      BTserial.println((exposure_times[0] * 0.001) / 60);
      BTserial.print("Moderate(2-5): ");
      BTserial.println((exposure_times[1] * 0.001) / 60);
      BTserial.print("High(5-7): ");
      BTserial.println((exposure_times[2] * 0.001) / 60);
      BTserial.print("Very high(7-10): ");
      BTserial.println((exposure_times[3] * 0.001) / 60);
      BTserial.print("Extremely high(>10): ");
      BTserial.println((exposure_times[4] * 0.001) / 60);
      BTserial.print("UV index: ");
      BTserial.println(uv_sensor);
      BTserial.print("Heartrate: ");
      BTserial.println(0);
      BTserial.print("Current steps: ");
      BTserial.println(Steps);
      BTserial.print("Distance travelled (km): ");
      BTserial.println(Distance_km);
      BTserial.print("Battery value (cV): ");
      BTserial.println(batvalue);
      
  }
  
  delay(75);
}