/*PWM占空比对电机角度控制程序
   允许不同频率PWM控制电机转动角度
   PWM频率过高可能不能运转，实测122Hz可以，500Hz不行。
*/

#include <SimpleFOC.h>
#include <math.h>
#define LED_PIN 10
#define PWM_PIN 2
#define pi 3.1415926

//以下参数需要配置
#define min_ang 0  //旋转角度上下限
#define max_ang 360
#define PWM_frq 122  //PWM输入频率
#define VCC 8  //VCC输入电压
#define V_limit 7   //最高相电压限制，电机发热严重时调低，但是力矩也会降低

#define time_duration_max 122*260/PWM_frq

bool fir = true;
unsigned long time_h = 0, time_duration = 0, time_l = 0, time_duration_temp = 0;
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);


void Read_PWM()
{
  if (digitalRead(2) == HIGH)
  {
    time_h = millis();
  }
  else if (digitalRead(2) == LOW)
  {
    time_l = millis();
    if (fir)
    {
      time_duration_temp = time_l - time_h;
      fir = false;
    }
    if (abs(time_duration_temp - time_duration) < 3)
    {
      time_duration = time_l - time_h;
      time_duration_temp = time_duration;
    }
  }
}

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

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), Read_PWM, CHANGE);
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = VCC;
  motor.voltage_limit = V_limit;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 3;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller =  angle;
  motor.PID_velocity.P = 0.08;
  motor.PID_velocity.I = 4;
  motor.PID_velocity.D = 0.0002;//如果出现高速抖动，调低P，D调0
  motor.LPF_velocity.Tf = 0.02;
  motor.P_angle.P = 20;
  motor.velocity_limit = 50;
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();
  Blink(3);
  //_delay(1000);
}

void loop() {
  target_angle = float(map(time_duration, 0, time_duration_max, min_ang, max_ang)) / 180 * pi;
  motor.loopFOC();
  motor.move(target_angle);
}
