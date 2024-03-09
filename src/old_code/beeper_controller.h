#include "beeper.h"

#define BEEPER_PIN 23

#define PLAY_STARTUP true

class BalanceBeeper {
  private:
    Beeper beeper;

    bool switchStateLatch = false;
    long lastLowVoltageMillis = 0;
  public:
    BalanceBeeper() :
      beeper(BEEPER_PIN){
    }
    void setup(){ 
      beeper.setup();
      if(PLAY_STARTUP){
        beeper.queueThreeShort();
      }
    }

    void loop(){
      beeper.loop();
      if (battery.warning) {
        beeper.queueSad();
        battery.warning = false;
      }
    }

};