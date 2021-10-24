#include "BLDCMotor.h"

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
BLDCMotor::BLDCMotor(int pp, float _R)
: FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // torque control type is voltage by default
  torque_controller = TorqueControlType::voltage;
}


/**
	Link the driver which controls the motor
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
void BLDCMotor::init() {
  if(monitor_port) monitor_port->println(F("MOT: Init"));

  // 没有电流采样，并且复制了电机相电阻，那么就用电流限制值*电机相电阻作为电压限制值。
  // if no current sensing and the user has set the phase resistance of the motor, use current limit to calculate the voltage limit
  if( !current_sense && _isset(phase_resistance)) {
    float new_voltage_limit = current_limit * (phase_resistance); // v_lim = current_lim / (3/2 phase resistance) - worst case
    // use it if it is less then voltage_limit set by the user
    voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
  }
  // sanity check for the voltage limit configuration   再次确认限制电压
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // constrain voltage for sensor alignment  传感器校准时的电压约束
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // update the controller limits
  if(current_sense){  //是否采用了电流环的电流采集， 默认是没有，所以是空指针。如果非空指针，则有电流采集
    // current control loop controls voltage
    PID_current_q.limit = voltage_limit;    //电流环PID输出限幅值 为 电压限制值
    PID_current_d.limit = voltage_limit;
    // velocity control loop controls current
    PID_velocity.limit = current_limit;   // 速度环PID输出限幅值 为 电流限制值
  }else{
    PID_velocity.limit = voltage_limit;   //没有电流环，那么 速度环的PID输出限幅值 就是 电压限制值
  }
  P_angle.limit = velocity_limit;   //角度环的PID输出限幅值 为 速度限制值

  _delay(500);
  // enable motor
  if(monitor_port) monitor_port->println(F("MOT: Enable driver."));
  enable();   //打开PWM输出
  _delay(500); 
}


// disable motor driver
void BLDCMotor::disable()
{
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void BLDCMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // motor status update
  enabled = 1;
}

/**
  FOC functions
*/
// FOC initialization function
int  BLDCMotor::initFOC( float zero_electric_offset, Direction _sensor_direction) {
  int exit_flag = 1;   //退出标志，是1的话说明校准成功，是0的话说明出错。
  // align motor if necessary
  // alignment necessary for encoders!
  if(_isset(zero_electric_offset)){  //如果提供了 电角度零点偏置值，那么就不需要进行电机的偏移校准
    // abosolute zero offset provided - no need to align
    zero_electric_angle = zero_electric_offset;
    // set the sensor direction - default CW
    sensor_direction = _sensor_direction;
  }

  // sensor and motor alignment - can be skipped
  // by setting motor.sensor_direction and motor.zero_electric_angle
  _delay(500);
  if(sensor)   //如果使用编码器
    exit_flag *= alignSensor();   //校准编码器
  else if(monitor_port)   //没使用编码器的话，就输出无传感器的提示
    monitor_port->println(F("MOT: No sensor."));

  // aligning the current sensor - can be skipped
  // checks if driver phases are the same as current sense phases
  // and checks the direction of measuremnt. 
  // 校准电流传感器，检查驱动器的三相是否与电流传感器的三相相同，并且检查测量的方向
  _delay(500);
  if(exit_flag){ 
    if(current_sense) 
      exit_flag *= alignCurrentSense();   //使用了电流传感器，那么就校准电流传感器
    else if(monitor_port)    //没使用电流传感器打印提示
      monitor_port->println(F("MOT: No current sense."));
  }
  
  if(exit_flag){
    if(monitor_port) monitor_port->println(F("MOT: Ready."));
  }else{
    if(monitor_port) monitor_port->println(F("MOT: Init FOC failed."));
    disable();
  }
 
  return exit_flag;
}

// Calibarthe the motor and current sense phases
int BLDCMotor::alignCurrentSense() {
  int exit_flag = 1; // success 

  if(monitor_port) monitor_port->println(F("MOT: Align current sense."));

  // align current sense and the driver
  exit_flag = current_sense->driverAlign(driver, voltage_sensor_align);
  if(!exit_flag){ 
    // error in current sense - phase either not measured or bad connection
    if(monitor_port) monitor_port->println(F("MOT: Align error!"));
    exit_flag = 0;
  }else{
    // output the alignment status flag
    if(monitor_port) monitor_port->print(F("MOT: Success: "));
    if(monitor_port) monitor_port->println(exit_flag);
  }

  return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle  编码器对准电气0度
int BLDCMotor::alignSensor() {
  int exit_flag = 1; //success
  if(monitor_port) monitor_port->println(F("MOT: Align sensor."));
  
  // if unknown natural direction  如果自然方向未知
  if(!_isset(sensor_direction)){
    // check if sensor needs zero search
    if(sensor->needsSearch())  // 磁编码器不需要寻找零点，因此这里不执行
      exit_flag = absoluteZeroSearch();
    // stop init if not found index
    if(!exit_flag) return exit_flag;

    // find natural direction
    // move one electrical revolution forward  向前转动一圈电气角度
    for (int i = 0; i <=500; i++ ) {
      float angle = _3PI_2 + _2PI * i / 500.0;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
      _delay(2);
    }
    // take and angle in the middle
    float mid_angle = sensor->getAngle();
    // move one electrical revolution backwards  //电机再反转一圈电气角度
    for (int i = 500; i >=0; i-- ) {
      float angle = _3PI_2 + _2PI * i / 500.0 ;
      setPhaseVoltage(voltage_sensor_align, 0,  angle);
      _delay(2);
    }
    float end_angle = sensor->getAngle();
    setPhaseVoltage(0, 0, 0);  //关闭电机驱动
    _delay(200);
    // determine the direction the sensor moved 
    if (mid_angle == end_angle) {  //说明电机没动，打印提示信息
      if(monitor_port) monitor_port->println(F("MOT: Failed to notice movement"));
      return 0; // failed calibration
    } else if (mid_angle < end_angle) {  //根据两次正反转的角度对比，得到编码器的方向
      if(monitor_port) monitor_port->println(F("MOT: sensor_direction==CCW"));
      sensor_direction = Direction::CCW;
    } else{
      if(monitor_port) monitor_port->println(F("MOT: sensor_direction==CW"));
      sensor_direction = Direction::CW;
    }
    // check pole pair number 检验电机的极对数
    if(monitor_port) monitor_port->print(F("MOT: PP check: "));
    float moved =  fabs(mid_angle - end_angle);  //转过的机械角度
    if( fabs(moved*pole_pairs - _2PI) > 0.5 ) { // 0.5 is arbitrary number it can be lower or higher!
      if(monitor_port) monitor_port->print(F("fail - estimated pp:"));
      if(monitor_port) 
       //上面是驱动电机转动一圈电气角度2Pi，编码器测得的是机械角度moved，他们之间关系是 2Pi = moved * pole_pairs
        monitor_port->println(_2PI/moved,4);  //4的意思应该是打印输出4位数字或者四位小数？
    }else if(monitor_port) monitor_port->println(F("OK!"));

  }
  else if(monitor_port)  //初始化编码器就复制编码器的计数方向，那么这里就可以跳过编码器方向和电机校准。
    monitor_port->println(F("MOT: Skip dir calib."));  

  // zero electric angle not known   电气零度未知
  if(!_isset(zero_electric_angle)){
    // align the electrical phases of the motor and sensor
    // set angle -90(270 = 3PI/2) degrees 
    setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  //设置电机到-90电气角度
    _delay(700);
    // 用此时编码器读取的机械角度*极对数，计算出来的电气角度作为电气零度
    zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    _delay(20);
    if(monitor_port){
      monitor_port->print(F("MOT: Zero elec. angle: "));
      monitor_port->println(zero_electric_angle);
    }
    // stop everything
    setPhaseVoltage(0, 0, 0);  
    _delay(200);
  }else if(monitor_port) monitor_port->println(F("MOT: Skip offset calib."));
  return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int BLDCMotor::absoluteZeroSearch() {
  
  if(monitor_port) monitor_port->println(F("MOT: Index search..."));
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5*_2PI);
    // call important for some sensors not to loose count
    // not needed for the search
    sensor->getAngle();
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found
  if(monitor_port){
    if(sensor->needsSearch()) monitor_port->println(F("MOT: Error: Not found!"));
    else monitor_port->println(F("MOT: Success!"));
  }
  return !sensor->needsSearch();
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void BLDCMotor::loopFOC() {
  // if disabled do nothing
  if(!enabled) return; 
  // if open-loop do nothing
  if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;

  // shaft angle   机械角度，里面减去了偏置
  shaft_angle = shaftAngle();  
  // electrical angle - need shaftAngle to be called first
  electrical_angle = electricalAngle();  //里面加上偏置再*极对数求电气角度，又减去了电气零点角度

  switch (torque_controller) {
    case TorqueControlType::voltage:
      // no need to do anything really
      break;
    case TorqueControlType::dc_current:
      if(!current_sense) return;
      // read overall current magnitude
      current.q = current_sense->getDCCurrent(electrical_angle);
      // filter the value values
      current.q = LPF_current_q(current.q);
      // calculate the phase voltage
      voltage.q = PID_current_q(current_sp - current.q); 
      voltage.d = 0;
      break;
    case TorqueControlType::foc_current:   //力矩控制方式为FOC电流模式
      if(!current_sense) return;
      // read dq currents   计算dq轴电流
      current = current_sense->getFOCCurrents(electrical_angle);
      // filter values
      current.q = LPF_current_q(current.q);
      current.d = LPF_current_d(current.d);
      // calculate the phase voltages  电流环
      voltage.q = PID_current_q(current_sp - current.q); 
      voltage.d = PID_current_d(-current.d);
      break;
    default:
      // no torque control selected
      if(monitor_port) monitor_port->println(F("MOT: no torque control selected!"));
      break;
  }
  
  // set the phase voltage - FOC heart function :)
  // 注意这里传入的电角度是 转子 当前的电角度，实际想要的电压的电角度在这个基础上再加上一个值
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);   //输入值包括电角度，这也是有感FOC算法
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void BLDCMotor::move(float new_target) {
  // if disabled do nothing
  if(!enabled) return; 
  // downsampling (optional)  //下采样？
  if(motion_cnt++ < motion_downsample) return;
  motion_cnt = 0;
  // set internal target variable
  if(_isset(new_target)) target = new_target;
  // get angular velocity
  shaft_velocity = shaftVelocity();   //机械速度，已经经过滤波

  switch (controller) {
    case MotionControlType::torque:
      if(torque_controller == TorqueControlType::voltage) // if voltage torque control
        if(!_isset(phase_resistance))  voltage.q = target;  //应用的就是这种模式，把q轴电压直接设置为目标电压
        else voltage.q =  target*phase_resistance; 
      else 
        current_sp = target; // if current/foc_current torque control
      break;
    case MotionControlType::angle:
      // angle set point
      shaft_angle_sp = target;  // 机械角度设置的位置
      // calculate velocity set point  位置环纯P控制，得到目标速度
      shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
      // calculate the torque command  速度环，得到目标电流
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
      // if torque controlled through voltage  如果是电压控制方式，那么目标电压直接等于目标电流
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  
          voltage.q = current_sp;  
        else  
          voltage.q = current_sp*phase_resistance;
        voltage.d = 0;   //Vd直接设置成0
      }
      break;
    case MotionControlType::velocity:
      // velocity set point
      shaft_velocity_sp = target;
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if current/foc_current torque control
      // if torque controlled through voltage control 
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q = current_sp*phase_resistance;
        voltage.d = 0;
      }
      break;
    case MotionControlType::velocity_openloop:
      // velocity control in open loop
      shaft_velocity_sp = target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
    case MotionControlType::angle_openloop:
      // angle control in open loop
      shaft_angle_sp = target;
      voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
  }
}
 

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usaage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      static int trap_120_map[6][3] = {
        {_HIGH_IMPEDANCE,1,-1},{-1,1,_HIGH_IMPEDANCE},{-1,_HIGH_IMPEDANCE,1},{_HIGH_IMPEDANCE,-1,1},{1,-1,_HIGH_IMPEDANCE},{1,_HIGH_IMPEDANCE,-1} // each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
      };
      // static int trap_120_state = 0;
      sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either 
      // modulation_centered == true > driver.volage_limit/2 
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0 
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(_HIGH_IMPEDANCE, _ACTIVE, _ACTIVE); // disable phase if possible
      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(_ACTIVE, _HIGH_IMPEDANCE, _ACTIVE);// disable phase if possible
      }else{ 
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(_ACTIVE,_ACTIVE, _HIGH_IMPEDANCE);// disable phase if possible
      }

    break;

    case FOCModulationType::Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      static int trap_150_map[12][3] = {
        {_HIGH_IMPEDANCE,1,-1},{-1,1,-1},{-1,1,_HIGH_IMPEDANCE},{-1,1,1},{-1,_HIGH_IMPEDANCE,1},{-1,-1,1},{_HIGH_IMPEDANCE,-1,1},{1,-1,1},{1,-1,_HIGH_IMPEDANCE},{1,-1,-1},{1,_HIGH_IMPEDANCE,-1},{1,1,-1} // each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
      };
      // static int trap_150_state = 0;
      sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either 
      // modulation_centered == true > driver.volage_limit/2 
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0 
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(_HIGH_IMPEDANCE, _ACTIVE, _ACTIVE); // disable phase if possible
      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(_ACTIVE, _HIGH_IMPEDANCE, _ACTIVE);// disable phase if possible
      }else{ 
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(_ACTIVE, _ACTIVE, _HIGH_IMPEDANCE);// disable phase if possible
      }

    break;

    case FOCModulationType::SinePWM :
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation

      // angle normalization in between 0 and 2pi
      // only necessary if using _sin and _cos - approximation functions
      angle_el = _normalizeAngle(angle_el);
      _ca = _cos(angle_el);
      _sa = _sin(angle_el);
      // Inverse park transform
      Ualpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      Ubeta =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

      // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
      center = driver->voltage_limit/2;
      // Clarke transform
      Ua = Ualpha + center;
      Ub = -0.5 * Ualpha  + _SQRT3_2 * Ubeta + center;
      Uc = -0.5 * Ualpha - _SQRT3_2 * Ubeta + center;

      if (!modulation_centered) {
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }

      break;

    case FOCModulationType::SpaceVectorPWM :
      // Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
      // https://www.youtube.com/watch?v=QMSWUMEAejg

      // the algorithm goes
      // 1) Ualpha, Ubeta
      // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
      // 3) angle_el = atan2(Ubeta, Ualpha)
      // 
      // equivalent to 2)  because the magnitude does not change is:
      // Uout = sqrt(Ud^2 + Uq^2)
      // equivalent to 3) is
      // angle_el = angle_el + atan2(Uq,Ud)

      float Uout;
      // a bit of optitmisation
      if(Ud){ // only if Ud and Uq set 
        // _sqrt is an approx of sqrt (3-4% error)
        Uout = _sqrt(Ud*Ud + Uq*Uq) / driver->voltage_limit;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));  //这里是去掉了反park变换，直接用Ud和Uq来计算合成目标电压所处的电角度和扇区
      }else{// only Uq available - no need for atan2 and sqrt
        Uout = Uq / driver->voltage_limit;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + _PI_2);  // 直接在当前转子的电角度的基础上增加pi/2,也就是目标电压超前pi/2
      }
      // find the sector we are in currently
      sector = floor(angle_el / _PI_3) + 1;  //floor(x):小于等于x的最大整数
      // calculate the duty cycles
      float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;  //这里的根号3和推导不一样，差一个倍数
      float T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
      // two versions possible
      float T0 = 0; // pulled to 0 - better for low power supply voltage
      if (modulation_centered) {
        T0 = 1 - T1 - T2; //modulation_centered around driver->voltage_limit/2
      }

      // calculate the duty cycles(times)
      float Ta,Tb,Tc;  //注意这里的Tabc是时间（高电平持续时间），而不是通常意义下的定时器的比较计数值。
      // 这里和用MOS管的SVPWM不一样，不用考虑每次只开关一个开关管的问题，所以这里的Tabc直接就是求占空比数值
      switch(sector){  
        case 1:
          Ta = T1 + T2 + T0/2;   
          Tb = T2 + T0/2;
          Tc = T0/2;
          break;
        case 2:
          Ta = T1 +  T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T0/2;
          break;
        case 3:
          Ta = T0/2;
          Tb = T1 + T2 + T0/2;
          Tc = T2 + T0/2;
          break;
        case 4:
          Ta = T0/2;
          Tb = T1+ T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 5:
          Ta = T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T2 + T0/2;
          break;
        case 6:
          Ta = T1 + T2 + T0/2;
          Tb = T0/2;
          Tc = T1 + T0/2;
          break;
        default:
         // possible error state
          Ta = 0;
          Tb = 0;
          Tc = 0;
      }

      // calculate the phase voltages and center
      Ua = Ta*driver->voltage_limit;
      Ub = Tb*driver->voltage_limit;
      Uc = Tc*driver->voltage_limit;
      break;

  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}



// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float BLDCMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts); 
  // for display purposes
  shaft_velocity = target_velocity;
  
  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance; 

  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float BLDCMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5) Ts = 1e-3; 

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  
  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance; 
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;
  
  return Uq;
}
