#include "asr_pro.h"

void setup() {
  asrInit();
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
sendToAsr("r");
  delay(1000);
sendToAsr("o");  
 delay(1000);
}
