void updateMixerGain() {
  if (millis() > lattime_fade_update + fade_update_interval) {
    ////////////////////////// update mixer_gain
    for (int i = 0; i < 4; i++) {
      if (mixer_gain[i] > destination_gain[i]) mixer_gain[i] = mixer_gain[i] - 0.01;
      if (mixer_gain[i] < destination_gain[i]) mixer_gain[i] = mixer_gain[i] + 0.02;
      if (mixer_gain[i] > 1) mixer_gain[i] = 1;
      if (mixer_gain[i] < 0.01) {
        mixer_gain[i] = 0;
      }else{
      }
    }
    
    ///////////////////////// set mixer_gain
//    // mixerMono
//    for (int i = 0; i < 4; i++) {
//      mixerMono.gain(i, mixer_gain[i]); // L
//    }

    amp1.gain(mixer_gain[0]);
    amp2.gain(mixer_gain[1]);
    amp3.gain(mixer_gain[2]);
    amp4.gain(mixer_gain[3]);
    
    lattime_fade_update = millis();
  }


}
