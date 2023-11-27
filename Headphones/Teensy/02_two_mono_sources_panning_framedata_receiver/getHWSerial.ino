void getHWSerial() {
  // 11bytes
  // 255, FINAL_RESULT, d1, d2, d3 ,0, 0, 0, 0,0,  elapsed_time
  // 255, FRAME_DATA, frameMSB[0], frameMiddleB[0], frameLSB[0], frameMSB[1], frameMiddleB[1],frameLSB[1],frameMSB[2], frameMiddleB[2],frameLSB[2]

  if (HWSERIAL.available() > 10) {

    //    Serial.println("HWSERIAL");
    // check my bow state
    // my_bow_state = digitalRead(mercurySwitchPin);

   // Serial.println("Got Final Result");
    // digitalWrite(13, HIGH);
    byte first_byte = HWSERIAL.read();
    byte second_byte = HWSERIAL.read();
  //  Serial.print("first_byte ="); Serial.println(first_byte);
  //  Serial.print("second_byte ="); Serial.println(second_byte);
    if (first_byte == 255 && second_byte == FINAL_RESULT) {
      Serial.println();
      Serial.print("DistanceInfo: ");
      // read the raging rusult
      for (int i = 0; i < 3; i++) {
        distance_result[i] = HWSERIAL.read();
        //if (distance_result[i] == 0)MY_ADDRESS = (i + 1);
        Serial.print(distance_result[i]);
        Serial.print(", ");
      }

      Serial.println();
      // read 5 dummy bytes
      for (int i = 0; i < 5; i++) {
        byte discard;
        discard = HWSERIAL.read();
      }

      // read elapsed_time
      elapsed_time = HWSERIAL.read();

      Serial.print("eleapsed time: ");
      Serial.println(elapsed_time);
      /*
        for (int i = 0; i < 80; i++) {
        _info[i] = HWSERIAL.read();
        //if (distance_result[i] == 0)MY_ADDRESS = (i + 1);
        Serial.print(_info[i]);
        Serial.print(", ");
        }

        Serial.println();
      */
      // checksum_received = HWSERIAL.read(); // s_data[103]

      // Serial.print("checksum ");
      // Serial.println(checksum_received);

      //      displayData05();

      isGetPosition = true;

    } else if (first_byte == 255 && second_byte == FRAME_DATA) {
      Serial.print("FrameInfo: ");
      for (int i = 0; i < 3; i++) {
        frameMSB[i] = HWSERIAL.read();
        frameMiddleB[i] = HWSERIAL.read();
        frameLSB[i] = HWSERIAL.read();
        Serial.print(frameMSB[i]); Serial.print(", "); Serial.print(frameMSB[i]); Serial.print(", "); Serial.print(frameMSB[i]); Serial.print(", ");
      }
      Serial.println();
    }
    else {
      HWSERIAL.clear();
      while (HWSERIAL.available())HWSERIAL.read();
    }
    // HWSERIAL.clear();
  }
}
