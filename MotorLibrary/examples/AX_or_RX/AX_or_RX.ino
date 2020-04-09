#include <MotorLibrary.h>
//this example for AX or RX motor


AX_RX_XL motor(2); //set ID
void setup() {
  // put your setup code here, to run once:
  motor.goto_wheel_mode();
  motor.wheel(1, 1023);

  motor.goto_joint_mode();
  motor.joint(200, 1023, 100);
}

void loop() {
  // put your main code here, to run repeatedly:

}
