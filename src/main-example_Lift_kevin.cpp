#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <Smoothed.h>
#include "AHRSProtocol.h"
#include <QuickPID.h>
#include <angles.hpp> // angles

using pin_t = uint8_t;
using namespace angles;

/**
 * assumes source domain is oposite of target domain
 */
float convert_domain(float angle, angles::domain target_domain)
{
    if (target_domain == angles::domain::continuous)
    {
        return angle >= 0 ? angle : 360 + angle;
    }
    else
    {
        return angle <= 180 ? angle : -360 + angle;
    }
}

class ReadPWM
{
    uint32_t _last, _timeout;
    uint8_t _pin, _mode;

public:
    ReadPWM(uint8_t pin, uint8_t mode = HIGH, uint32_t timeout = 1050)
        : _last(0), _timeout(timeout), _pin(pin), _mode(mode)
    {
        pinMode(pin, INPUT);
    }

    bool read(uint32_t &into)
    {
        into = pulseIn(this->_pin, this->_mode, this->_timeout);
        if (into == 0)
        {
            into = this->_last;
            return false;
        }
        else
        {
            this->_last = into;
            return true;
        }
    }
};

class PwmToAngleConverter
{
    float _offset;
    bool _reverse;
    uint32_t _min_pulse, _max_pulse;

public:
    static const angles::domain DOMAIN = angles::domain::continuous;
    static const angles::unit UNIT = angles::unit::radians;

    PwmToAngleConverter(bool reverse = false, float offset = 0, uint32_t min_pulse = 1, uint32_t max_pulse = 1024)
        : _offset(offset), _reverse(reverse), _min_pulse(min_pulse), _max_pulse(max_pulse) {}

    float convert(uint32_t pwm)
    {
        auto raw_angle = (pwm - this->_min_pulse) * 2 * PI / (this->_max_pulse - this->_min_pulse);
        raw_angle = this->_reverse ? (2 * PI) - raw_angle : raw_angle;
        return raw_angle + this->_offset;
    }

    void set_offset(float offset)
    {
        this->_offset = offset;
    }
};

float travel_deg(float from, float to)
{
    auto zeroed = to - from;
    if (zeroed > 180)
    {
        return zeroed - 360;
    }
    else if (zeroed < -180)
    {
        return zeroed + 360;
    }
    else
    {
        return zeroed;
    }
}

int8_t clean_joystick_input(int8_t input)
{
    if (abs(input) < 10)
        return 0;
    auto constrained = constrain(input, -127, 127); // Fix max value to be symmetric
    return constrained;
}

class NavX
{
    static const int ITERATION_DELAY_MS = 10;
    static const int NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT = 0x32;
    static const int NUM_BYTES_TO_READ = 8;

    static const int register_address = NAVX_REG_YAW_L;

    byte _data[512];

public:
    struct Heading
    {
        float yaw;
        float pitch;
        float roll;
        float heading;
    };

    struct FieldCentricInput
    {
        int8_t forward;
        int8_t strafe;
        double rotation;
    };

    NavX() : _data{0} {}

    Heading read()
    {
        /* Transmit I2C data request */
        Wire.beginTransmission(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
        Wire.write(NavX::register_address);                                // Sends starting register address
        Wire.write(NavX::NUM_BYTES_TO_READ);                               // Send number of bytes to read
        Wire.endTransmission();                                            // Stop transmitting

        /* Receive the echoed value back */
        Wire.beginTransmission(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);                    // Begin transmitting to navX-Sensor
        Wire.requestFrom(NavX::NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NavX::NUM_BYTES_TO_READ); // Send number of bytes to read
        for (size_t i = 0; Wire.available(); i++)
        { // Read data (slave may send less than requested)
            this->_data[i++] = Wire.read();
        }
        Wire.endTransmission(); // Stop transmitting

        /* Decode received data to floating-point orientation values */
        float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[0]) / 2.55;     // The cast is needed on arduino
        float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[2]) / 2.55;       // The cast is needed on arduino
        float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&this->_data[4]) / 2.55;      // The cast is needed on arduino
        float heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&this->_data[6]) / 2.55; // The cast is needed on arduino

        return (Heading){.yaw = yaw * 360,
                         .pitch = pitch * 360,
                         .roll = roll * 360,
                         .heading = heading * 360};
    }

    static FieldCentricInput convertToRobotCentric(double forward, double strafe, double rotation, double gyroAngle)
    {
        // Convert gyro angle to radians
        double angleRad = (gyroAngle * PI) / 180.0;

        // Perform field-centric to robot-centric conversion
        auto field_forward = constrain(forward * cos(angleRad) + strafe * sin(angleRad), -127, 128);
        auto field_strafe = constrain(-forward * sin(angleRad) + strafe * cos(angleRad), -127, 128);

        return (FieldCentricInput){.forward = field_forward,
                                   .strafe = field_strafe,
                                   .rotation = rotation};
    }
};

struct Joystick
{
    float x, y;
};

struct JoystickPair
{
    Joystick left, right;
};

/**
 * =============
 * CONFIGURATION
 * =============
 */

const pin_t WHEEL_FL_M_p = CRC_PWM_11;
const pin_t WHEEL_FR_M_p = CRC_PWM_2;
const pin_t WHEEL_BL_M_p = CRC_PWM_12;
const pin_t WHEEL_BR_M_p = CRC_PWM_4;

const pin_t LIFT_L_M_p = CRC_PWM_10;
const pin_t LIFT_R_M_p = CRC_PWM_3;
const pin_t LIFT_E_p = CRC_DIG_3; // lift height

const pin_t MANIP_PITCH_E_p = CRC_DIG_4;
const pin_t MANIP_PITCH_M_p = CRC_PWM_8;

const pin_t MANIP_ROLL_E_p = CRC_DIG_2;
const pin_t MANIP_ROLL_M_p = CRC_PWM_1; // NOTE: limit to 20% is a good default speed

const pin_t MANIP_BELT_A_p = CRC_PWM_9;
const pin_t MANIP_BELT_B_p = CRC_PWM_5;

const pin_t BEAM_p = CRC_DIG_1;

const int PRINT_TIMER_DELAY = 1000 / 20; // 20Hz

const double FIELD_CENTRIC_P = 7;
const double FIELD_CENTRIC_I = 0;
const double FIELD_CENTRIC_D = 0.0011;

const double FIELD_CENTRIC_OUTPUT_LIM = 60;
const double FIELD_CENTRIC_SAMPLE_FREQ_HZ = 50;

/**
 * ========================
 * WORKERS (OBJECTS & VARS)
 * ========================
 */

angle<domain::continuous, unit::degrees> field_centric_offset{0};

Servo manip_belt_a, manip_belt_b;

NavX navx;

CrcLib::Timer print_timer, battery_low_timeout;

ReadPWM lift_PWM(LIFT_E_p), pitch_PWM(MANIP_PITCH_E_p), roll_PWM(MANIP_ROLL_E_p);
PwmToAngleConverter lift_converter, pitch_converter, roll_converter;
angles::AngleMovingAvg lift_averager(0.2), pitch_averager(0.2), roll_averager(0.2);

float input, output, setpoint = 0;
QuickPID pid(&input, &output, &setpoint,
             FIELD_CENTRIC_P, FIELD_CENTRIC_I, FIELD_CENTRIC_D, QuickPID::Action::reverse);

void soft_kill()
{
    CrcLib::SetPwmOutput(WHEEL_BL_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_BR_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_FL_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_FR_M_p, 0);
    CrcLib::SetPwmOutput(LIFT_L_M_p, 0);
    CrcLib::SetPwmOutput(LIFT_R_M_p, 0);
    CrcLib::SetPwmOutput(MANIP_PITCH_M_p, 0);
    CrcLib::SetPwmOutput(MANIP_ROLL_M_p, 0);
    // manip_belt_a.write(0);
    // manip_belt_b.write(0);
    pid.SetOutputSum(0);
}

/**
 * ============
 * SETUP & LOOP
 * ============
 */

#include <Adafruit_NeoPixel.h>
#define LED_PIN CRC_PWM_6
#define NUM_LEDS 12

Adafruit_NeoPixel strip1(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// POSITION MOTEUR = ROUGE
// ^^ Quand il sort la pièce = FLASH ROUGE RAPIDE
// Quand il recupere picee VERTICALE = FLASH VERT RAPIDE
// Quand il recupere picee HORIZONTALE = FLASH VERT moitier moitier RAPIDE
// Semi-Manuel / tour = MAUVE => SORTIR PIECE = MAUVE LENTEMENT
// DEFAULT = FLASH FLADE 50/50 ORANGE BLEU CEGEP MOYEN VITE
// Vitesse lente = ^^ mais encore plus lentement

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
LEDMode current_state_machine = VITESSE_DEFAULT;
// LEDMode current_led2_mode = VITESSE_DEFAULT; // Si pont H fonctionne pas

LEDMode update_state_machine()
{
    return POSITION_MOTEUR;
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
    // Wire.begin();

    // CrcLib::Initialize();

    // CrcLib::InitializePwmOutput(WHEEL_FL_M_p, false);
    // CrcLib::InitializePwmOutput(WHEEL_FR_M_p, true); // Is normally true
    // CrcLib::InitializePwmOutput(WHEEL_BL_M_p, false);
    // CrcLib::InitializePwmOutput(WHEEL_BR_M_p, true); // Is normally true

    // CrcLib::InitializePwmOutput(LIFT_L_M_p, true);
    // CrcLib::InitializePwmOutput(LIFT_R_M_p, true);

    // CrcLib::InitializePwmOutput(MANIP_PITCH_M_p, false);
    // CrcLib::InitializePwmOutput(MANIP_ROLL_M_p, false);

    // pinMode(BEAM_p, INPUT);

    // // manip_belt_a.attach(MANIP_BELT_A_p);
    // // manip_belt_b.attach(MANIP_BELT_B_p);

    // print_timer.Start(PRINT_TIMER_DELAY);

    // pid.SetMode(QuickPID::Control::automatic);
    // pid.SetSampleTimeUs(1000 / FIELD_CENTRIC_SAMPLE_FREQ_HZ);
    // pid.SetOutputLimits(-FIELD_CENTRIC_OUTPUT_LIM, FIELD_CENTRIC_OUTPUT_LIM);

    init_led_strip(strip1);
    pinMode(BEAM_p, INPUT);
}

void loop()
{
    static uint8_t curseur = 6;
    // static uint32_t chrono10 = millis();
    // if (digitalRead(BEAM_p)) // si on a une piece dans le manip
    // {
    //     //chrono10 = millis();
    //     curseur++;
    //     if (curseur >= LEDMode::MAX)
    //     {
    //         curseur = 0;
    //     }
    //     Serial.println((LEDMode)curseur);
    //     while(true){yield();}
    // }
    static int i = 1;
    if (!digitalRead(BEAM_p))
    {
        i++;
        Serial.println(i);
        if (i > strip1.numPixels())
        {
            i = 1;
        }
    }
    Color color2 = {255, 255, 0};
    strip1.setPixelColor(i, color2.r, color2.g, color2.b);
    strip1.show();

    // current_state_machine = update_state_machine();
    // led_Show(curseur, strip1);
}