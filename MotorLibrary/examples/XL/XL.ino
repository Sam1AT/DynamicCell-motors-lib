#include <MotorLibrary.h>
//this example for XL motor


AX_RX_XL motor(2); //set ID
void setup() {
  // put your setup code here, to run once:
  motor.goto_wheel_mode_XL();
  motor.wheel_XL(1, 1023);

  motor.goto_joint_mode_XL();
  motor.joint_XL(200, 1023, 100);
}

void loop() {
  // put your main code here, to run repeatedly:

}
