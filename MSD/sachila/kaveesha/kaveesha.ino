int b;
void setup() {
  b = 1;
}

void loop() {
  if (b == 1) {
    // arm =1;
    b = 2;
  }
  if (b == 2) {
    toright();
    b = 3;
  }
  if (b == 3) {
    toleft();
    b = 4;
  }
  if (b == 4) {
    linefollowing();
    objectdetect();
    b = 5;
  }
  if (b == 5) {
    //detect color
    colors[1];
    b = 6;
  }
  if (b == 6) {
    if (color[1] == blynk[2]) {
      //pick
      b = 7;
    } else {
      b = 18;
    }
  }
  if (b == 7) {
    //turn 180 right func.
    b = 8;
  }
  if (b == 8) {
    if (colors[2] == blynk[1]) {
      b = 9;
    } else {
      b = 12;
    }
  }
  if (b == 9) {
    //pick3
    b = 10;
  }
  if (b == 10) {
    //turn 180 line follow
    b = 11;
  }
  if (b == 11) {
    //stop
    b = 100;
  }
  if (b == 12) {
    //turn 180 line follow
    b = 13;
  }
  if (b == 13) {
    //pick3
    b = 14;
  }
  if (b == 14) {
    turn180();
    b = 15;
  }
  if (b == 15) {
    toleft();
    b = 16;
  }
  if (b == 16) {
    toright();
    b = 17;
  }
  if (b == 17) {
    //stop
    b = 100;
  }
  if (b == 18) {
    //turn 180 line follow
    b = 19;
  }
  if (b == 19) {
    if (colors[2] == blynk[2]) {
      b = 20;
    } else {
      b = 28;
    }
  }
  if (b == 20) {
    //pick arm 2
    b = 21;
  }
  if (b == 21) {
    if (colors[1] == blynk[1]) {
      b = 25;
    } else {
      b = 22;
    }
  }
  if (b == 22) {
    //turn 180 line follow
    b = 23;
  }
  if (b == 23) {
    //kola paata
    b = 24;
  }
  if (b == 24) {
    //stop
    b = 100;
  }
  if (b == 25) {
    toleft();
    b = 26;
  }
  if (b == 26) {
    //nil paata
    b = 27;
  }
  if (b == 27) {
    //stop
    b = 100;
  }
  if (b == 28) {
    //turn 180 line follow
    b = 29;
  }
  if (b == 29) {
    //pick arm 3
    b = 30;
  }
  if (b == 30) {
    turn180();
    b = 31;
  }
  if (b == 31) {
    toleft();
    b = 32;
  }
  if (b == 32) {
    if (colors[1] == blynk[1]) {
      b = 33;
    } else {
      b = 39;
    }
    if (b == 33) {
      toleft();
      b = 34;
    }
    if (b == 34) {
      linefollowing();
      objectdetect();
      b = 35;
    }
    if (b == 35) {
      //pick arm 3
      b = 36;
    }
    if (b == 36) {
      turn180();
      b = 37;
    }
    if (b == 37) {
      linefollowing();
      b = 38;
    }
    if (b == 38) {
      //stop
      b = 100;
    }
    if (b == 39) {
      if (IRL == 1) {
        b = 40;
      } else {
        b = 39;
      }
    }
    if (b == 40) {
      linefollowing();
      b = 41;
    }
    if (b == 41) {
      toleft();
      b = 42;
    }
    if (b == 42) {
      linefollowing();
      objectdetect();
      b = 43;
    }
    if (b == 43) {
      //nil paata
      b = 44;
    }
    if (b == 44) {
      //stop
      b = 100;
    }
    if (b == 100) {
      //bla bla bla
    }
  }
