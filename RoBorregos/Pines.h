// - LED RGB
  #define ledRojo 7
  #define ledVerde 5
  #define ledAzul 6
// - 4 LEDs Color Sensor
  #define S0 53
  #define S1 52
  #define S2 51
  #define S3 50
  #define SensorOut 4
  // Calibration Values
    const int redMin = 26; // Red minimum value
    const int redMax = 206; // Red maximum value
    const int greenMin = 28; // Green minimum value
    const int greenMax = 276; // Green maximum value
    const int blueMin = 22; // Blue minimum value
    const int blueMax = 213; // Blue maximum value
  // Threshold Variable
    const int thresholdValue = 25; // Error acceptance
// - Dist Sensors Trigger------------------------------------------- 
  #define FrontDistSensorTrigger 12
  #define LeftDistSensorTrigger 11
  #define RightDistSensorTrigger 13
// - Dist Sensors Echo-------------------------------------------
  #define FrontDistSensorEcho 23
  #define LeftDistSensorEcho 25
  #define RightDistSensorEcho 27
// - Motors
  #define RightA 32
  #define RightB 34
  #define RightPow 2 
  #define LeftA 35
  #define LeftB 33
  #define LeftPow 3
// - Encoders
  const int pulses_per_turn = 20;
  #define RightEncoder 19
  #define LeftEncoder 18
// - InfraRed
  const int NInfraRedSensor = 5;
  int InfraRedSensors[] = {22, 24, 26, 28, 30};
// - Servo
  #define servoRight 9
  #define servoLeft 10
// - PID
  const float kp = 1;
  const float ki = 0.0001;
  const float kd = 0.4;