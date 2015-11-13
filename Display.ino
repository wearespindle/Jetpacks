void updateDisplay () {
    timer = millis(); // reset the timer
    // +/- 125 msec
    fona.getBattPercent(&battpercent);
    display.setCursor(18,57);
    display.print(battpercent);
    display.print("%");
    
    uint8_t r = map(fona.getRSSI(), 0, 31, 1, 5);
    display.setCursor(48,57);
    display.print(r);

    if (largebatt) {
      display.setTextSize(4);
      display.setCursor(8,18);
      display.print(battpercent);
      display.print("%");
      display.setTextSize(1);
    }

  char time[5];
  // ToDo: timezone implementation
  sprintf(time, "%2d:%02d", GPS.hour+1, GPS.minute);
  display.setCursor(65,57);
  display.print(time);

  if (GPS.fix) {
    display.drawChar(91,16,'G',WHITE,BLACK,1);
  } else {
    display.drawChar(91,16,'g',WHITE,BLACK,1);
  }
}

void clearMiddle() {
  display.fillRect(0,8,89,49,BLACK);
}
