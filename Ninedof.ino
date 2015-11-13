/************
 9DOF & AHRS
*************/

#define NINEDOF_INTERVAL 1000

float p = 3.1415926;
sensors_vec_t orientation;
uint32_t dofTimer = 0;
uint32_t lastMillis9dof = 0;

// Create LSM9DS0 board instance.
Adafruit_LSM9DS0 lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

void nineDOFSetup() {
  // Initialise the LSM9DS0 board.
  if (!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }

  // Setup the sensor gain and integration time.
  // You don't need to change anything here, but have the option to select different
  // range and gain values.
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);    // 1.) Set the accelerometer range
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);     // 2.) Set the magnetometer sensitivity
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);  // 3.) Setup the gyroscope
}

void nineDOFLoop() {
    dofTimer = millis();
    // Use the simple AHRS function to get the current orientation.
  if (dofTimer - lastMillis9dof > NINEDOF_INTERVAL && ahrs.getOrientation(&orientation)) {
    displayHeading();
    lastMillis9dof = millis();
  }
}

void displayHeading() {
  int  heading = orientation.heading + 180;
  display.setCursor(0,57);
  if (heading < 0) {
    display.print("??");
    display.print(heading);                
  } else if (heading < 23) {
    display.print("W ");
  } else if (heading < 68) {
    display.print("ZW");      
  } else if (heading < 113) { 
    display.print("Z ");
  } else if (heading < 158) {
    display.print("ZO");      
  } else if (heading < 203) {
    display.print("O ");
  } else if (heading < 248) {
    display.print("NO");      
  } else if (heading < 293) {
    display.print("N ");
  } else if (heading < 338) {
    display.print("NW");      
  } else if (heading < 361) {
    display.print("W ");
  } else {
    display.print("??");
    display.print(heading);          
  }
}

void getHeading(char headingc[]) {
    itoa(orientation.heading + 180, headingc, 10);
}

