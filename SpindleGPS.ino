Location currentLocation = Location();

void gpsSetup() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPSSerial.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}

void gpsLoop() {
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (GPS.fix && GPS.HDOP < 5 && GPS.HDOP != 0) {
    currentLocation.set(GPS);
  }  else {
    currentLocation.isValid = false;
  }
}

void displayGPSDebugInfo() {
  if (currentLocation.isValid) {
    Serial.print(F("Location: "));
    Serial.print(currentLocation.latitude, 6);
    Serial.print(F(", "));
    Serial.print(currentLocation.longitude, 6);
    Serial.print(F(", "));
    Serial.println(currentLocation.altitude, 2);
    Serial.print(F("HDOP: "));
    Serial.println(GPS.HDOP);
  } else {
    Serial.println(F("No location"));
  }
}

void getLatLongAlt(char *latitude, char *longitude, char *altitude) {
  if (currentLocation.isValid) {

    char latC1[3];
    itoa(currentLocation.latitude, latC1, 10);
    uint32_t latint = (currentLocation.latitude - int(currentLocation.latitude)) * 10000000;
    char latC2[8];
    itoa(latint, latC2, 10);
    sprintf(latitude, "%s.%s", latC1, latC2);

    char longC1[3];
    itoa(currentLocation.longitude, longC1, 10);
    uint32_t longint = (currentLocation.longitude - int(currentLocation.longitude)) * 10000000;
    char longC2[8];
    itoa(longint, longC2, 10);
    sprintf(longitude, "%s.%s", longC1, longC2);

    char altC1[5];
    itoa(currentLocation.altitude, altC1, 10);
    uint32_t altint = abs((currentLocation.altitude - int(currentLocation.altitude)) * 10000000);
    char altC2[2];
    itoa(altint, altC2, 10);
    sprintf(altitude, "%s.%s", altC1, altC2);
  }
}

Location getCurrentLocation() {
  return currentLocation;
}

#ifdef __AVR__ // ToDo: Reminder to fix this properly.
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}
#endif //#ifdef__AVR__

