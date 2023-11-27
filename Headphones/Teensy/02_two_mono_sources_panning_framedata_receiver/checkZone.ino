void checkZone(){
  getPosition();
  
  switch (state_zone) {
    case STATE_ZONE_CHECK:
      // Serial.println("STATE_ZONE_CHECK");
      if (pointInPolygon(x, y , 0)) {
        numZone = 1;
        state_zone = STATE_ZONE1;
        break;
      } else if (pointInPolygon(x, y, 1)) {
        numZone = 2;
        state_zone = STATE_ZONE2;
        break;
      } else {
        Serial.println("No Zone");
        numZone = 0;
        fadeOutZone1();
        fadeOutZone2();
        state_zone = STATE_ZONE_CHECK;
      }
      break;

    case STATE_ZONE1:
      Serial.println("STATE_ZONE1");
      fadeInZone1();
      fadeOutZone2();
      state_zone = STATE_ZONE_CHECK;
      break;

    case STATE_ZONE2:
      Serial.println("STATE_ZONE2");
      fadeInZone2();
      fadeOutZone1();
      state_zone = STATE_ZONE_CHECK;
      break;

    default:
      break;
  }
}

void getPosition() {
  //https://en.wikipedia.org/wiki/Trilateration

  float r1 = distance_result[0] / 10.0; // distance between this tag and anchor1
  float r2 = distance_result[1] / 10.0; // distance between this tag and anchor2
  float r3 = distance_result[2] / 10.0; // distance between this tag and anchor3

  if ((r1 < max_distance[0] && r1 > min_distance[0]) && (r2 < max_distance[1] && r2 > min_distance[1]) && (r3 < max_distance[2] && r3 > min_distance[2])) {
    x = ((r1 * r1) - (r2 * r2) + (d * d)) / (2 * d);
    y =  ((r1 * r1) - (r3 * r3) + (p3_i * p3_i) + (p3_j * p3_j)) / (2 * p3_j) - (p3_i * x / p3_j);
    z = sqrt(r1 * r1 - x * x - y * y );

//    displayData05();
  } else {
    // display error for debug
//    displayData06();
  }
}
