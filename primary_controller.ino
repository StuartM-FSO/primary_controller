#include "system_state.h"
#include "time_helpers.h"

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }
  Serial.println("Starting...");

}

void loop() {

}
