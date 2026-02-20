#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <Smoothed.h>
#include "AHRSProtocol.h"
#include <QuickPID.h>
#include <angles.hpp> // angles

// using namespace angles;

// /**
//  * =============
//  * TOOLS & UTILS
//  * =============
//  */

// using pin_t = uint8_t;

// /**
//  * assumes source domain is oposite of target domain
//  */
// float convert_domain(float angle, angles::domain target_domain)
// {
//     if (target_domain == angles::domain::continuous)
//     {
//         return angle >= 0 ? angle : 360 + angle;
//     }
//     else
//     {
//         return angle <= 180 ? angle : -360 + angle;
//     }
// }

// class ReadPWM
// {
//     uint32_t _last, _timeout;
//     uint8_t _pin, _mode;

// public:
//     ReadPWM(uint8_t pin, uint8_t mode = HIGH, uint32_t timeout = 1050)
//         : _last(0), _timeout(timeout), _pin(pin), _mode(mode)
//     {
//         pinMode(pin, INPUT);
//     }

//     bool read(uint32_t &into)
//     {
//         into = pulseIn(this->_pin, this->_mode, this->_timeout);
//         if (into == 0)
//         {
//             into = this->_last;
//             return false;
//         }
//         else
//         {
//             this->_last = into;
//             return true;
//         }
//     }
// };

// class PwmToAngleConverter
// {
//     float _offset;
//     bool _reverse;
//     uint32_t _min_pulse, _max_pulse;

// public:
//     static const angles::domain DOMAIN = angles::domain::continuous;
//     static const angles::unit UNIT = angles::unit::radians;

//     PwmToAngleConverter(bool reverse = false, float offset = 0, uint32_t min_pulse = 1, uint32_t max_pulse = 1024)
//         : _offset(offset), _reverse(reverse), _min_pulse(min_pulse), _max_pulse(max_pulse) {}

//     float convert(uint32_t pwm)
//     {
//         auto raw_angle = (pwm - this->_min_pulse) * 2 * PI / (this->_max_pulse - this->_min_pulse);
//         raw_angle = this->_reverse ? (2 * PI) - raw_angle : raw_angle;
//         return raw_angle + this->_offset;
//     }

//     void set_offset(float offset)
//     {
//         this->_offset = offset;
//     }
// };

// float travel_deg(float from, float to)
// {
//     auto zeroed = to - from;
//     if (zeroed > 180)
//     {
//         return zeroed - 360;
//     }
//     else if (zeroed < -180)
//     {
//         return zeroed + 360;
//     }
//     else
//     {
//         return zeroed;
//     }
// }

// int8_t clean_joystick_input(int8_t input)
// {
//     if (abs(input) < 10)
//         return 0;
//     auto constrained = constrain(input, -127, 127); // Fix max value to be symmetric
//     return constrained;
// }

// class NavX
// {
//     static const int ITERATION_DELAY_MS = 10;
//     static const int NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT = 0x32;
//     static const int NUM_BYTES_TO_READ = 8;

//     static const int register_address = NAVX_REG_YAW_L;

//     byte _data[512];

// public:
//     struct Heading
//     {
//         float yaw;
//         float pitch;
//         float roll;
//         float heading;
//     };

//     struct FieldCentricInput
//     {
//         int8_t forward;
//         int8_t strafe;
//         double rotation;
//     };

//     NavX() : _data{0} {}

//     Heading read()
//     {
//         /* Transmit I2C data request */
//         Wire.beginTransmission(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
//         Wire.write(NavX::register_address);                                // Sends starting register address
//         Wire.write(NavX::NUM_BYTES_TO_READ);                               // Send number of bytes to read
//         Wire.endTransmission();                                            // Stop transmitting

//         /* Receive the echoed value back */
//         Wire.beginTransmission(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);                    // Begin transmitting to navX-Sensor
//         Wire.requestFrom(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NavX::NUM_BYTES_TO_READ); // Send number of bytes to read
//         for (size_t i = 0; Wire.available(); i++)
//         { // Read data (slave may send less than requested)
//             this->_data[i++] = Wire.read();
//         }
//         Wire.endTransmission(); // Stop transmitting

//         /* Decode received data to floating-point orientation values */
//         float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[0]) / 2.55;     // The cast is needed on arduino
//         float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[2]) / 2.55;       // The cast is needed on arduino
//         float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[4]) / 2.55;      // The cast is needed on arduino
//         float heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&this->_data[6]) / 2.55; // The cast is needed on arduino

//         return (Heading){.yaw = yaw * 360,
//                          .pitch = pitch * 360,
//                          .roll = roll * 360,
//                          .heading = heading * 360};
//     }

//     static FieldCentricInput convertToRobotCentric(double forward, double strafe, double rotation, double gyroAngle)
//     {
//         // Convert gyro angle to radians
//         double angleRad = (gyroAngle * PI) / 180.0;

//         // Perform field-centric to robot-centric conversion
//         auto field_forward = constrain(forward * cos(angleRad) + strafe * sin(angleRad), -127, 128);
//         auto field_strafe = constrain(-forward * sin(angleRad) + strafe * cos(angleRad), -127, 128);

//         return (FieldCentricInput){.forward = field_forward,
//                                    .strafe = field_strafe,
//                                    .rotation = rotation};
//     }
// };

// struct Joystick
// {
//     float x, y;
// };

// struct JoystickPair
// {
//     Joystick left, right;
// };

// /**
//  * =============
//  * CONFIGURATION
//  * =============
//  */

// const pin_t WHEEL_FL_M_p = CRC_PWM_11;
// const pin_t WHEEL_FR_M_p = CRC_PWM_2;
// const pin_t WHEEL_BL_M_p = CRC_PWM_12;
// const pin_t WHEEL_BR_M_p = CRC_PWM_4;

// const pin_t LIFT_L_M_p = CRC_PWM_10;
// const pin_t LIFT_R_M_p = CRC_PWM_3;
// const pin_t LIFT_E_p = CRC_DIG_3; // lift height

// const pin_t MANIP_PITCH_E_p = CRC_DIG_4;
// const pin_t MANIP_PITCH_M_p = CRC_PWM_8;

// const pin_t MANIP_ROLL_E_p = CRC_DIG_2;
// const pin_t MANIP_ROLL_M_p = CRC_PWM_1; // NOTE: limit to 20% is a good default speed

// const pin_t MANIP_BELT_A_p = CRC_PWM_9;
// const pin_t MANIP_BELT_B_p = CRC_PWM_5;

// const pin_t BEAM_p = CRC_DIG_1;

// const int PRINT_TIMER_DELAY = 1000 / 20; // 20Hz

// const double FIELD_CENTRIC_P = 7;
// const double FIELD_CENTRIC_I = 0;
// const double FIELD_CENTRIC_D = 0.0011;

// const double FIELD_CENTRIC_OUTPUT_LIM = 60;
// const double FIELD_CENTRIC_SAMPLE_FREQ_HZ = 50;

// /**
//  * ========================
//  * WORKERS (OBJECTS & VARS)
//  * ========================
//  */

// angle<domain::continuous, unit::degrees> field_centric_offset{0};

// Servo manip_belt_a, manip_belt_b;

// NavX navx;

// CrcLib::Timer print_timer, battery_low_timeout;

// ReadPWM lift_PWM(LIFT_E_p), pitch_PWM(MANIP_PITCH_E_p), roll_PWM(MANIP_ROLL_E_p);
// PwmToAngleConverter lift_converter, pitch_converter, roll_converter;
// angles::AngleMovingAvg lift_averager(0.2), pitch_averager(0.2), roll_averager(0.2);

// float input, output, setpoint = 0;
// QuickPID pid(&input, &output, &setpoint,
//              FIELD_CENTRIC_P, FIELD_CENTRIC_I, FIELD_CENTRIC_D, QuickPID::Action::reverse);

// /**
//  * ============
//  * SETUP & LOOP
//  * ============
//  */

// void setup()
// {
//     Serial.begin(115200);
//     Wire.begin();

//     CrcLib::Initialize();

//     CrcLib::InitializePwmOutput(WHEEL_FL_M_p, false);
//     CrcLib::InitializePwmOutput(WHEEL_FR_M_p, true); // Is normally true
//     CrcLib::InitializePwmOutput(WHEEL_BL_M_p, false);
//     CrcLib::InitializePwmOutput(WHEEL_BR_M_p, true); // Is normally true

//     CrcLib::InitializePwmOutput(LIFT_L_M_p, true);
//     CrcLib::InitializePwmOutput(LIFT_R_M_p, true);

//     CrcLib::InitializePwmOutput(MANIP_PITCH_M_p, false);
//     CrcLib::InitializePwmOutput(MANIP_ROLL_M_p, false);

//     pinMode(BEAM_p, INPUT);

//     // manip_belt_a.attach(MANIP_BELT_A_p);
//     // manip_belt_b.attach(MANIP_BELT_B_p);

//     print_timer.Start(PRINT_TIMER_DELAY);

//     pid.SetMode(QuickPID::Control::automatic);
//     pid.SetSampleTimeUs(1000 / FIELD_CENTRIC_SAMPLE_FREQ_HZ);
//     pid.SetOutputLimits(-FIELD_CENTRIC_OUTPUT_LIM, FIELD_CENTRIC_OUTPUT_LIM);
// }

// void soft_kill()
// {
//     CrcLib::SetPwmOutput(WHEEL_BL_M_p, 0);
//     CrcLib::SetPwmOutput(WHEEL_BR_M_p, 0);
//     CrcLib::SetPwmOutput(WHEEL_FL_M_p, 0);
//     CrcLib::SetPwmOutput(WHEEL_FR_M_p, 0);
//     CrcLib::SetPwmOutput(LIFT_L_M_p, 0);
//     CrcLib::SetPwmOutput(LIFT_R_M_p, 0);
//     CrcLib::SetPwmOutput(MANIP_PITCH_M_p, 0);
//     CrcLib::SetPwmOutput(MANIP_ROLL_M_p, 0);
//     // manip_belt_a.write(0);
//     // manip_belt_b.write(0);
//     pid.SetOutputSum(0);
// }

// void loop()
// {
//     CrcLib::Update();

//     /**
//      * ------
//      * GUARDS
//      * ------
//      */

//     static float battery_voltage_limit = 11.0;
//     if (false && CrcLib::GetBatteryVoltage() < battery_voltage_limit)
//     {
//         // TODO: figure out something more graceful.
//         battery_voltage_limit = 15.0f;
//         Serial.println("Battery LOW: " + String(CrcLib::GetBatteryVoltage()));
//         soft_kill();
//         return;
//     }

//     if (true && !CrcLib::IsCommValid())
//     {
//         // block everything if controller is not connected
//         soft_kill();
//         Serial.println("no com");
//         return;
//     }

//     /**
//      * ----------------
//      * CONTROLLER INPUT
//      * ----------------
//      */

//     auto joysticks_raw = (JoystickPair){
//         .left = {
//             .x = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X),
//             .y = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_Y),
//         },
//         .right = {
//             .x = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_X),
//             .y = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_Y),
//         }};

//     int8_t trig_L = CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_L);
//     int8_t trig_R = CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_R);

//     /**
//      * ------------------
//      * SENSOR AQUISITIONS
//      * ------------------
//      */

//     uint32_t manip_pitch_signal;
//     pitch_PWM.read(manip_pitch_signal);
//     uint32_t manip_roll_signal;
//     roll_PWM.read(manip_roll_signal);
//     uint32_t lift_height_signal;
//     lift_PWM.read(lift_height_signal);

//     bool beam_obstructed = CrcLib::GetDigitalInput(BEAM_p);

//     NavX::Heading h = navx.read();

//     /**
//      * ---------------
//      * DATA PROCESSING
//      * --------------
//      */

//     auto joysticks_clean = (JoystickPair){
//         .left = {
//             .x = (float)clean_joystick_input(joysticks_raw.left.x),
//             .y = (float)clean_joystick_input(joysticks_raw.left.y),
//         },
//         .right = {
//             .x = (float)clean_joystick_input(joysticks_raw.right.x),
//             .y = (float)clean_joystick_input(joysticks_raw.right.y),
//         }};

//     // lift_averager.add(
//     //     convert_domain(
//     //         lift_converter.convert(lift_height_signal),
//     //         angles::domain::mirror) /
//     //     180);
//     // pitch_averager.add(
//     //     convert_domain(
//     //         pitch_converter.convert(manip_pitch_signal),
//     //         angles::domain::mirror) /
//     //     180);
//     // roll_averager.add(
//     //     convert_domain(
//     //         roll_converter.convert(manip_roll_signal),
//     //         angles::domain::mirror) /
//     //     180);

//     float robot_rotation = h.yaw;
//     // TODO: fix navx instead, will make a better resolution
//     if (robot_rotation < 180)
//     {
//         robot_rotation = map(robot_rotation, 0.0, 100.0, 0.0, 180.0);
//     }
//     else
//     {
//         robot_rotation = map(robot_rotation, 260.0, 360.0, 180.0, 360.0);
//     }

//     auto current_rotation = angle<domain::continuous, unit::degrees>::from(robot_rotation + field_centric_offset.value);

//     /**
//      * ---------------
//      * PID SHENANIGANS
//      * ---------------
//      */

//     static angle<domain::continuous, unit::degrees> target_rotation{NAN};

//     if (isnan(target_rotation.value))
//     {
//         // NOTE: intialize target to the current rotation to avoid the robot trying to move right after turning on
//         target_rotation = current_rotation;
//     }

//     if (abs(joysticks_clean.right.x) > 5 || abs(joysticks_clean.right.y) > 5)
//     {
//         // TODO: ugly fucking line, clean up eventually. but for now, it works.
//         target_rotation = angle<domain::continuous, unit::degrees>::from(180 - (atan2(joysticks_clean.right.x, joysticks_clean.right.y) * 180 / PI));
//     }

//     input = current_rotation.travel(target_rotation).value;

//     /**
//      * -----------------------
//      * MOTOR OUTPUTS, CONTROLS
//      * -----------------------
//      */

//     if (CrcLib::ReadDigitalChannel(BUTTON::SELECT))
//     {
//         /* RESET FIELD CENTRIC REFERENCE*/
//         field_centric_offset = angle<domain::continuous, unit::degrees>::from(field_centric_offset.value - current_rotation.value);
//     }

//     if (true && CrcLib::ReadDigitalChannel(BUTTON::COLORS_DOWN))
//     {
//         /* PRECISION MODE */
//         const float REDUCTION = 0.3;
//         joysticks_clean.left.x *= REDUCTION;
//         joysticks_clean.left.y *= REDUCTION;
//         joysticks_clean.right.x *= REDUCTION;
//         joysticks_clean.right.y *= REDUCTION;
//     }

//     if (true && pid.Compute())
//     {
//         /* FIELD CENTRIC HOLONOMIC */
//         // Convert joystick inputs to field-centric
//         NavX::FieldCentricInput robotCentric = NavX::convertToRobotCentric(
//             joysticks_clean.left.y, // Forward
//             joysticks_clean.left.x, // Strafe
//             output,                 // Rotation
//             -current_rotation.convert<angles::domain::mirror>().value);

//         // Apply converted values to motors
//         CrcLib::MoveHolonomic(
//             robotCentric.forward,
//             robotCentric.rotation,
//             robotCentric.strafe,
//             WHEEL_FL_M_p, WHEEL_BL_M_p, WHEEL_FR_M_p, WHEEL_BR_M_p);
//     }

//     if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_UP))
//     {
//         /* LIFT */
//         CrcLib::SetPwmOutput(LIFT_L_M_p, trig_L);
//         CrcLib::SetPwmOutput(LIFT_R_M_p, trig_R);
//     }

//     if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_LEFT))
//     {
//         /* MANIPULATOR PITCH/ROLL */
//         CrcLib::SetPwmOutput(MANIP_PITCH_M_p, trig_L);
//         CrcLib::SetPwmOutput(MANIP_ROLL_M_p, trig_R);
//     }

//     if (true && CrcLib::ReadDigitalChannel(BUTTON::COLORS_RIGHT))
//     {
//         /* BELTS */
//         auto speed_L = map(trig_L, -128, 127, 1000, 2000);
//         auto speed_R = map(trig_R, -128, 127, 1000, 2000);
//         // manip_belt_a.writeMicroseconds(speed_L);
//         // manip_belt_b.writeMicroseconds(speed_R);
//     }

//     if (CrcLib::ReadDigitalChannel(BUTTON::START))
//     {
//         Serial.println("softkilling");
//         soft_kill();
//     }

//     /**
//      * ----------------
//      * SERIAL REPORTING
//      * ----------------
//      */
//     if (print_timer.IsFinished())
//     {
//         print_timer.Start(PRINT_TIMER_DELAY);

//         // Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));

//         /* beam state */
//         // Serial.println("beam: " + String(beam_obstructed));

//         /* reading encoders */
//         Serial.print("l: " + String(lift_converter.convert(lift_height_signal)) +
//                      "\tp: " + String(pitch_converter.convert(manip_pitch_signal)) +
//                      "\tr: " + String(roll_converter.convert(manip_roll_signal)));

//         Serial.println("\t\tl: " + String(lift_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value) +
//                        "\tp: " + String(pitch_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value) +
//                        "\tr: " + String(roll_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value));

//         /* controller trigger states */
//         Serial.println("triggers:\tL:" + String(trig_L) + "\tR: " + String(trig_R));

//         /* cmp expected rotation with curent rotation*/
//         // Serial.println("expected: " + String(target_rotation) + "\tactual: " + String(current_rotation));

//         /* field centric PID info */
//         // Serial.println("input: " + String(input) + "\toutput: " + String(output));
//     }
// }

// POSITION MOTEUR = ROUGE
// ^^ Quand il sort la pièce = FLASH ROUGE RAPIDE
// Quand il recupere picee VERTICALE = FLASH VERT RAPIDE
// Quand il recupere picee HORIZONTALE = FLASH VERT moitier moitier RAPIDE
// Semi-Manuel / tour = MAUVE => SORTIR PIECE = MAUVE LENTEMENT
// DEFAULT = FLASH FLADE 50/50 ORANGE BLEU CEGEP MOYEN VITE
// Vitesse lente = ^^ mais encore plus lentement

#include <Adafruit_NeoPixel.h>
#define LED_PIN CRC_PWM_11
#define NUM_LEDS 12

Adafruit_NeoPixel strip1(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void init_led_strip(Adafruit_NeoPixel &strip)
{
    strip.begin();
    strip.setBrightness(100); // baisse si alim instable
    strip.clear();
    strip.show();
}

typedef enum LEDMode
{
    POSITION_MOTEUR,
    SORT_LA_PIECE_DU_MANIP,
    RECUPERE_PIECE_VERTICALE,
    RECUPERE_PIECE_HORIZONTALE,
    SEMI_MANUEL_AVEC_PIECE,
    SEMI_MANUEL_PIECE_SORT,
    VITESSE_LENTE,
    VITESSE_DEFAULT,
    MAX
};
LEDMode current_led1_mode = VITESSE_DEFAULT;
// LEDMode current_led2_mode = VITESSE_DEFAULT; // Si pont H fonctionne pas

uint8_t update_LED_Mode()
{
    return 0;
}

const int LED_FLASH_DURATION = 100;
void FLASH_HALF(Adafruit_NeoPixel &strip, Color color1, Color color2, uint8_t speed = 1)
{
    static uint32_t chrono = millis();
    static bool state = false;

    if (millis() - chrono >= LED_FLASH_DURATION * speed)
    {
        chrono = millis();
        state = !state;
    }
    if (state)
    {
        for (int i = 0; i < strip.numPixels() / 2; i++)
        {
            strip.setPixelColor(i, color1.r, color1.g, color1.b);
        }
        for (int i = strip.numPixels() / 2; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, color2.r, color2.g, color2.b);
        }
    }
    else
    {
        for (int i = 0; i < strip.numPixels() / 2; i++)
        {
            strip.setPixelColor(i, color2.r, color2.g, color2.b);
        }
        for (int i = strip.numPixels() / 2; i < strip.numPixels(); i++)
        {
            strip.setPixelColor(i, color1.r, color1.g, color1.b);
        }
    }
}

const Color orange = {255, 40, 0};
const Color blue = {0, 0, 255};

void led_Show(uint8_t mode, Adafruit_NeoPixel &strip)
{
    static bool state = false;
    switch (mode)
    {
    case (POSITION_MOTEUR): // LED ROUGE

        strip.fill(strip.Color(255, 0, 0));
        break;

    case (SORT_LA_PIECE_DU_MANIP): // LED FLASH ROUGE

        static uint32_t chrono1 = millis();
        if (millis() - chrono1 >= LED_FLASH_DURATION)
        {
            chrono1 = millis();
            state = !state;
        }
        state ? strip.fill(strip.Color(255, 0, 0)) : strip.fill(strip.Color(0, 0, 0));
        break;

    case (RECUPERE_PIECE_VERTICALE): // LED FLASH VERT RAPIDE

        static uint32_t chrono2 = millis();
        if (millis() - chrono2 >= LED_FLASH_DURATION)
        {
            chrono2 = millis();
            state = !state;
        }
        state ? strip.fill(strip.Color(0, 255, 0)) : strip.fill(strip.Color(0, 0, 0));
        break;

    case (RECUPERE_PIECE_HORIZONTALE): // LED FLASH VERT moitier moitier RAPIDE

        FLASH_HALF(strip, {0, 255, 0}, {0, 0, 0});
        break;

    case (SEMI_MANUEL_AVEC_PIECE): // LED MAUVE

        strip.fill(strip.Color(255, 0, 255));
        break;

    case (SEMI_MANUEL_PIECE_SORT): // LED MAUVE FLASH LENTEMENT

        static uint32_t chrono4 = millis();
        if (millis() - chrono4 >= LED_FLASH_DURATION)
        {
            chrono4 = millis();
            state = !state;
        }
        state ? strip.fill(strip.Color(255, 0, 255)) : strip.fill(strip.Color(0, 0, 0));
        break;

    case (VITESSE_DEFAULT): // LED ORANGE BLEU FLASH FADE CEGEP MOYEN VITE

        FLASH_HALF(strip, orange, blue, 4);
        break;

    case (VITESSE_LENTE): // ^^ MOINS VITE

        FLASH_HALF(strip, orange, blue, 6);
        break;
    }

    strip.show();
}

void setup()
{
    Serial.begin(115200);
    init_led_strip(strip1);
}
void loop()
{
    static uint8_t curseur = 6;
    static uint32_t chrono10 = millis();
    if (millis() - chrono10 > 5000)
    {
        chrono10 = millis();
        curseur++;
        if (curseur >= LEDMode::MAX)
        {
            curseur = 0;
        }
        Serial.println((LEDMode)curseur);
    }
    led_Show(curseur, strip1);
}