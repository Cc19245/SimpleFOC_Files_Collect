 #include <SimpleFOC.h>

#define LED_PIN 10
#define pi 3.1415926
#define init_smooth 800 // larger, slower the initialization is. in case of disturbance.
#define volt_limit 5.0000

MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

void Blink(int n)
{
  for (int i = 0; i < n; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    _delay(200);
    digitalWrite(LED_PIN, LOW);
    _delay(100);
  }
}
float target_angle = 0;
float received_angle = 0;
void init_angle()// let the motor keep static when init.
{
  target_angle = sensor.getAngle();
  float delta = volt_limit / init_smooth;
  for (int i = 0; i <= init_smooth; i++)
  {
    motor.voltage_limit = delta * i;
    motor.loopFOC();
    motor.move(target_angle);
    serialReceiveUserCommand();
  }
  motor.voltage_limit = volt_limit;
}

void setup() {
  TCCR0B = (TCCR0B & 0xF8) | 0x01;
  TCCR1B = (TCCR1B & 0xF8) | 0x01;
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 8;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 3;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = ControlType::angle;
  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0.001;
  motor.LPF_velocity.Tf = 0.02;
  motor.P_angle.P = 20;
  motor.velocity_limit = 4;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();
  
  /*Serial.println("Motor ready.");
    Serial.print("ANGLE RANGE:  ");
    Serial.print(gap / pi * 180, 2);
    Serial.print("-");
    Serial.print((Max_Angle - Min_Angle - 2 * gap) / pi * 180, 2);
    Serial.println(" DEG");
    Serial.println("Set the target angle using serial terminal:");*/
  Blink(3);
  //_delay(1000);
  init_angle();
}

void loop() {
  motor.loopFOC();
  motor.move(target_angle);
  serialReceiveUserCommand();
}
void serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  while (Serial.available()) {
    // get the new byte:
    digitalWrite(LED_PIN, HIGH);
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {
      
      // change the motor target
      target_angle = received_chars.toFloat()/ 180 * pi;
      Serial.print("Target angle: ");
      Serial.println(target_angle);
      
      // reset the command buffer 
      received_chars = "";
      digitalWrite(LED_PIN, LOW);
    }
  }
}
