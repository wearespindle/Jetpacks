void sendLocation () {
  // Only sent if there is a valid location
  if (!currentLocation.isValid) return;

  char url[160];
  uint16_t statuscode;
  int16_t length;

  uint16_t vbat;
  char vbatc[4];
  char headingc[4];

  char latitude[12];
  char longitude[12];
  char altitude[12];

  fona.getBattPercent(&vbat);
  itoa(vbat, vbatc, 10);

  getHeading(headingc);
  getLatLongAlt(latitude, longitude, altitude);

  sprintf (url, "http://data.sparkfun.com/input/%s?private_key=%s&latitude=%s&longitude=%s&altitude=%s&battery=%s&heading=%s",
           SPARKFUN_PUBLIC_KEY, SPARKFUN_PRIVATE_KEY, latitude, longitude, altitude, vbatc, headingc);

  Serial.print(F("Sending: ")); Serial.println(url);

  uint8_t rssi = fona.getRSSI();

  if (rssi > 5) {
    // Make an attempt to turn GPRS mode on.  Sometimes the FONA gets freaked out and GPRS doesn't turn off.
    // When this happens you can't turn it on aagin, but you don't need to because it's on.  So don't sweat
    // the error case here -- GPRS could already be on -- just keep on keeping on and let HTTP_GET_start()
    // error if there's a problem with GPRS.
    display.setCursor(0, 8);
    display.print("GPRS       ");
    if (!fona.enableGPRS(true)) {
      Serial.println(F("Failed to turn GPRS on!"));
    }

    if (fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
      display.setCursor(0, 8);
      while (length > 0) {
        while (fona.available()) {
          char c = fona.read();
          display.print(c);
          Serial.print(c);
          length--;
          if (! length) break;
        }
      }
      fona.HTTP_GET_end();
    } else {
      Serial.println(F("Failed to send GPRS data!"));
    }

    if (!fona.enableGPRS(false)) {
      Serial.println(F("Failed to turn GPRS off!"));
    }
  } else {
    Serial.println(F("Can't transmit, network signal strength is crap!"));
  }
}
