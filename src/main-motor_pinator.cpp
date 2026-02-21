#include <Arduino.h>
#include <CrcLib.h>

// const int pin = CRC_PWM_5;

// Servo s1;
// Servo s2;

void setup()
{
    CrcLib::Initialize();
    CrcLib::InitializePwmOutput(CRC_PWM_3, false);
    CrcLib::InitializePwmOutput(CRC_PWM_10, false);
    // s1.attach(CRC_PWM_5);
    // s2.attach(CRC_PWM_6);
}

void loop()
{
    CrcLib::Update();
    // if (!CrcLib::IsCommValid())
    // {
        CrcLib::SetPwmOutput(CRC_PWM_3, 127);
        CrcLib::SetPwmOutput(CRC_PWM_10, 127);
    //     return;
    // }

    // auto val = CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X)/5;

    // s1.writeMicroseconds(2000);
    // s2.writeMicroseconds(1000);
    
}