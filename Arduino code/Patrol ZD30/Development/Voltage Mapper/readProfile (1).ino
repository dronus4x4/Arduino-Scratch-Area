// ===== THINGS TO DO ==========
// 1. Check that parameters are valid signed integers

void readProfile() {
  File profileInfo;
  int count = 0;
  int trap = 0;
  int percent = 0;
  String tempValue = "";
  //Append a string to construct the file name, then convert it to a charArray. It's messy but that's how arduino wants it.
  String fileNameSTRING = "profile";
  fileNameSTRING += profileSelect;
  fileNameSTRING += ".txt";
  char fileNameCHAR[fileNameSTRING.length() + 1]; //Create a charArray variable as long as fileName plus one
  fileNameSTRING.toCharArray(fileNameCHAR, fileNameSTRING.length() + 1); //Convert string to charArray and stuff it on the char variable
  Serial.print("Opening File:  " + fileNameSTRING);
  if (SD.exists(fileNameCHAR)) {  //Check if the file exists if so open it
    profileInfo = SD.open(fileNameCHAR);
    if (!profileInfo) { //Check if the file is opened properly, if not stop everything
      Serial.println(F("   ...ERROR!!!!"));
      return;
    } else {
      Serial.println(F("   ...SUCCESS"));
      char fileChar; // need to set the file contents to char type
      if (debug == true) {
        Serial.println(F("========== File Content ==========="));
        Serial.println(F(""));
      } else {
        Serial.println(F("Loading Parameters"));
      }
      while (profileInfo.available()) {
        fileChar = profileInfo.read(); // Read the next character in file
        if (fileChar == '=') { //do nothing with the characters until we hit '='
          fileChar = profileInfo.read(); // Read the next character in file
          count++;
          if (debug == true) {
            Serial.print(F("Loadpoint "));
            Serial.print(count);
            Serial.print(F(" = "));
          }
          if (fileChar == ' ') { // Do nothing with the characters until we hit a space
            while (fileChar != '\n') { // Keep reading until we hit a carrage return (TXT files do not have end of line characters, \0, so we are forced to have an end of file marker, essentially an ending line that isn't a parameter)
              fileChar = profileInfo.read(); // Read the next character in file
              tempValue.concat(fileChar); // Append the character to a temp variable
            } ;
            char charBuf[tempValue.length() + 1]; // Create Char variable: Again weird things in Arduino, you can't convert a String to Int, so you must convert to Char first then into String a CharArray then to an Int
            tempValue.toCharArray(charBuf, tempValue.length() + 1); // Convert string to array of chars
            settings[count] = atoi(charBuf); // Convert charArray to integer and load it into settings int Array
          } else {
            settings[count] = 0; // If nothing is found between the = and a carrage return then set a default value of 0
          }
          if (debug == true) {
            Serial.println(settings[count]);
          } else {
            trap++;
            if (trap <= 25) {
              Serial.print(F("."));
            } else {
              Serial.print(F("."));
              percent = percent + 10;
              Serial.print(percent);
              Serial.println(F("%"));
              trap = 0;
            }
          }
          tempValue = ""; // Clear the temp variable for the next parameter
        }
      }
      if (debug == true) {
        Serial.println(F(""));
        Serial.println(F(""));
        Serial.println(F("==================================="));
        Serial.println(F("File Closed"));
        Serial.println(F(""));
      } else {
        percent = percent + 10;
        Serial.print(percent);
        Serial.println(F("%"));
        Serial.println(F("Parameters Loaded"));
      }
      profileInfo.close();  //Close the file
      paramLoaded = true; //Lock it down so it will not be read again
    }
  } else {
    Serial.println(F("   ...FILE NOT FOUND!!!!"));
    return;
  }
}
