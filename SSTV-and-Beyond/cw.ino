const char* getMorse(char c) {
  switch (toupper(c)) {
    case 'A': return ".-";
    case 'B': return "-...";
    case 'C': return "-.-.";
    case 'D': return "-..";
    case 'E': return ".";
    case 'F': return "..-.";
    case 'G': return "--.";
    case 'H': return "....";
    case 'I': return "..";
    case 'J': return ".---";
    case 'K': return "-.-";
    case 'L': return ".-..";
    case 'M': return "--";
    case 'N': return "-.";
    case 'O': return "---";
    case 'P': return ".--.";
    case 'Q': return "--.-";
    case 'R': return ".-.";
    case 'S': return "...";
    case 'T': return "-";
    case 'U': return "..-";
    case 'V': return "...-";
    case 'W': return ".--";
    case 'X': return "-..-";
    case 'Y': return "-.--";
    case 'Z': return "--..";
    case '0': return "-----";
    case '1': return ".----";
    case '2': return "..---";
    case '3': return "...--";
    case '4': return "....-";
    case '5': return ".....";
    case '6': return "-....";
    case '7': return "--...";
    case '8': return "---..";
    case '9': return "----.";
    case '-': return "-....-";
    default: return "";
  }
}

void sendMorse(const char* message, int wpm) {
  int dotDuration = 1200 / wpm; // Dot-Dauer in ms (bei 60 WPM = 20 ms)

  ledcWriteTone(1000);
  waitBitPeriod();

  sa818Serial.println(SA818_SSTV_CONF);
  delay(200);
  digitalWrite(SA818_PTT, LOW);
  
  for (int i = 0; message[i] != '\0'; i++) {
    char c = message[i];
    if (c == ' ') {
      delay(dotDuration * 7); // Wortabstand
    } else {
      const char* code = getMorse(c);
      for (int j = 0; code[j] != '\0'; j++) {
        if (code[j] == '.') {
          ledcSendTone(dotDuration);
        } else if (code[j] == '-') {
          ledcSendTone(dotDuration * 3);
        }
        // Zwischen den Symbolen eines Buchstabens: 1 Dot-Pause (wenn nicht letztes Symbol)
        if (code[j+1] != '\0') {
          delay(dotDuration);
        }
      }
      // Buchstabenpause: insgesamt 3 Dot-Pausen, da bereits 1 Dot-Pause erfolgt
      delay(dotDuration * 2);
    }
  }

  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  // Stop TX
  digitalWrite(SA818_PTT, HIGH);
}