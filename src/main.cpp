/**
 * INFO: the interesting sections for users looking to customize the behavior of the robot (NON
 * contributors) are the following
 * - 2 CONFIGURATIONS
 * - 4.1 GUARDS
 * - 4.6 FEATURES
 * - 4.7 SERIAL REPORTING
 *
 * INDEX:
 *  1. TOOLS & UTILS
 *      > Local utilies and general implementations details.
 *  2. CONFIGURATIONS
 *      > Constants used to fine tune the behaviors. Pins, pid configurations, etc.
 *  3. WORKERS
 *      > Specific implementation details.
 *  4. MAIN
 *      > Program lifecycle (setup and loop) and lifecycle-related utilities (such as soft_kill).
 *      > A lot of the following sections contain togglable behaviors. To enable or disable these
 *      > behaviors, change the value of the first hard cooded boolean in the if statements that
 *      > encompass them. Ex:
 *      > ```cpp
 *      > if (true && <some other conditions>)
 *      > {
 *      >    // NAME: (FEATURE NAME OR DESCRIPTION)
 *      >   // the feature is enabled.
 *      > }
 *      > ```
 *      1. GUARDS
 *          > Safeties that prevent further execution in case some basic assertions are not met.
 *          > contains TOGGLEABLE BEHAVIOURS.
 *      2. CONTROLLER INPUT
 *          > Aquisition of controller state.
 *      3. SENSOR AQUISITIONS
 *          > Aquisition of sensor datas.
 *      4. DATA PROCESSING
 *          > Refinements to the aquisitions made above.
 *      5. PID SHENANIGANS
 *          > calculation of PID input values.
 *          > TODO: pid shenanigans should go in their appropriate features instead of here.
 *      6. FEATURES
 *          > Every feature of the robot (reactions to inputs and sensors).
 *          > contains TOGGLEABLE BEHAVIOURS.
 *      7. SERIAL REPORTING
 *          > contains TOGGLEABLE BEHAVIOURS.
 */

#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <Smoothed.h>
#include "AHRSProtocol.h"
#include <QuickPID.h>
#include <angles.hpp> // angles

using namespace angles;

/**
 * =============
 * TOOLS & UTILS
 * =============
 */

using pin_t = uint8_t;

class ReadPWM
{
    uint32_t _last, _timeout;
    uint8_t _pin, _mode;

public:
    ReadPWM(uint8_t pin, uint8_t mode = HIGH, uint32_t timeout = 2500)
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
public:
    static const angles::domain DOMAIN = angles::domain::continuous;
    static const angles::unit UNIT = angles::unit::radians;
    float _offset;
    bool _reverse;
    uint32_t _min_pulse, _max_pulse;

    PwmToAngleConverter(bool reverse = false, float offset = 0, uint32_t min_pulse = 1, uint32_t max_pulse = 1024)
        : _offset(offset), _reverse(reverse), _min_pulse(min_pulse), _max_pulse(max_pulse) {}

    angle<DOMAIN, UNIT> convert(uint32_t pwm)
    {
        auto raw_angle = (pwm - this->_min_pulse) * angle<DOMAIN, UNIT>::max_a() / (this->_max_pulse - this->_min_pulse);
        if (this->_reverse)
        {
            raw_angle = angle<DOMAIN, UNIT>::max_a() - raw_angle;
        }
        return angle<DOMAIN, UNIT>::from(raw_angle - this->_offset);
    }

    void set_offset(float offset)
    {
        this->_offset = offset;
    }
};

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

        return (FieldCentricInput){.forward = (int8_t)field_forward,
                                   .strafe = (int8_t)field_strafe,
                                   .rotation = rotation};
    }

    static float stupid_fix(float yaw)
    {
        // TODO: fix navx instead, will make a better resolution
        if (yaw < 180)
        {
            return map(yaw, 0.0, 100.0, 0.0, 180.0);
        }
        else
        {
            return map(yaw, 260.0, 360.0, 180.0, 360.0);
        }
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

struct PIDK
{
    float p, i, d;
};

struct PID_ios
{
    float input, output, setpoint;
};

/**
 * @param hz hertz
 * @returns delay time (in millis)
 */
uint32_t hz(float hz)
{
    auto ONE_SECOND = 1000;
    return ONE_SECOND / hz;
}

class FieldCentric
{
    // TODO: actually, this is technically just another smarthinge, but the angle is represented as a distance instead.
public:
    constexpr static const auto D = domain::continuous;
    constexpr static const auto U = unit::degrees;

    QuickPID &_pid;
    PID_ios &_ios;
    angle<D, U> _target;

    FieldCentric(QuickPID &pid, PID_ios &ios)
        : _pid(pid),
          _ios(ios),
          _target{NAN} {}

    /**
     * @returns did update
     */
    bool update(angle<D, U> current)
    {
        if (isnan(this->_target.value))
        {
            // NOTE: intialize target to the current rotation to avoid the robot trying to move right after turning on
            set_target(_target);
        }
        this->_ios.input = current.travel(this->_target).value;
        return this->_pid.Compute();
    }

    void set_target(angle<D, U> target)
    {
        this->_target = target;
    }
};

const float SAFETY_DEGREES_BUFFER = angle<domain::continuous, unit::degrees>::from(5)
                                        .template translate<unit::radians>()
                                        .value;
/**
 * we're working under the assumption that the bounds are within a single full continuous circle.
 */
class SmartHinge
{
public:
    constexpr static const auto D = domain::continuous;
    constexpr static const auto U = unit::radians;

    QuickPID &_pid;
    PID_ios &_ios;
    angle<D, U> _low_bound, _high_bound, _target;

    SmartHinge(QuickPID &pid, PID_ios &ios, const angle<D, U> low_bound, const angle<D, U> high_bound)
        : _pid(pid),
          _ios(ios),
          _low_bound{.value = low_bound.value + SAFETY_DEGREES_BUFFER},
          _high_bound{.value = high_bound.value - SAFETY_DEGREES_BUFFER},
          _target{NAN}
    {
    }

    /**
     * @returns did update
     */
    bool update(angle<D, U> current)
    {
        if (isnan(_target.value))
        {
            this->_target = current;
        }
        this->_ios.input = current.travel(_target).value;
        return this->_pid.Compute();
    }

    void set_target(angle<D, U> target)
    {
        this->_target = target.normalize();
        if (in_bounds(_low_bound.value, _high_bound.value, this->_target.value))
            return;
        if (abs(this->_target.travel(_low_bound).value) < abs(this->_target.travel(_high_bound).value))
        {
            this->_target = _low_bound;
        }
        else
        {
            this->_target = _high_bound;
        }
    }

    bool in_bounds(float low, float high, float to_test)
    {
        auto virt_low = angle<D, U>::from(low + abs(low)).normalize();
        auto virt_high = angle<D, U>::from(high + abs(low)).normalize();
        auto virt_to_test = angle<D, U>::from(to_test + abs(low)).normalize();
        return virt_low.value <= virt_to_test.value && virt_to_test.value <= virt_high.value;
    }
};

/**
 * =============================================================================================================================================
 * CONFIGURATION  =============================================================================================================================
 * =============================================================================================================================================
 */

/* pins */
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

/* timings */
const uint32_t PRINT_TIMER_DELAY = hz(20);
const uint32_t CONTROLLER_POLL_DELAY = hz(20);

/* PID configurations*/
void CONFIG_FIELD_CENTRIC_PID(QuickPID &pid)
{
    pid.SetTunings(
        7,
        0,
        0.0011);
    pid.SetControllerDirection(QuickPID::Action::reverse);
    pid.SetSampleTimeUs(hz(50));
    pid.SetOutputLimits(-60, 60);
}

void CONFIG_LIFT_PID(QuickPID &pid)
{
    pid.SetTunings(
        120,
        0.1,
        0);
    pid.SetSampleTimeUs(hz(50));
    pid.SetControllerDirection(QuickPID::Action::direct);
    pid.SetOutputLimits(-120, 120);
}

void CONFIG_ROLL_PID(QuickPID &pid)
{
    pid.SetTunings(
        1,
        0,
        0);
    pid.SetSampleTimeUs(hz(50));
    pid.SetOutputLimits(-20, 20);
}

void CONFIG_PITCH_PID(QuickPID &pid)
{
    pid.SetTunings(
        40,
        0,
        0);
    pid.SetSampleTimeUs(hz(50));
    pid.SetControllerDirection(QuickPID::Action::reverse);
    pid.SetOutputLimits(-20, 20);
}

/* fucakll/misc */
const double LIFT_CM_PER_RAD = 1;
const float PRECISION_MODE_REDUCTION = 0.3;
const float LIFT_ENCO_SMOOTHING = 0.1,
            PITCH_ENCO_SMOOTHING = 0.1,
            ROLL_ENCO_SMOOTHING = 0.1;

/**
 * ========================
 * WORKERS (OBJECTS & VARS)
 * ========================
 */

CrcLib::Timer print_timer, battery_low_timeout, controller_poll_timer;

/* lift and manipulator*/
Servo manip_belt_a, manip_belt_b;
ReadPWM lift_PWM(LIFT_E_p),
    pitch_PWM(MANIP_PITCH_E_p),
    roll_PWM(MANIP_ROLL_E_p);
PwmToAngleConverter
    lift_converter(true,                                               // GUILLAUME: premier param c'est si il est inversé ou pas
                   angle<domain::continuous, unit::degrees>::from(100) // GUILLAUME: changer le truc dans les parentheses du from pour set un offset du 0
                       .template translate<unit::radians>()
                       .value),
    pitch_converter(true,
                    angle<domain::continuous, unit::degrees>::from(174.5) // 185-182
                        .template translate<unit::radians>()
                        .value), // range: 0-57deg
    roll_converter(false,        // NOTE: for some reason, the real 90deg is read at 105deg
                   angle<domain::continuous, unit::degrees>::from(306.5)
                       .template translate<unit::radians>()
                       .value); // range: ~-15-~206deg
angles::AngleMovingAvg lift_averager(LIFT_ENCO_SMOOTHING), pitch_averager(PITCH_ENCO_SMOOTHING), roll_averager(ROLL_ENCO_SMOOTHING);
PID_ios lift_ios{0}, pitch_ios{0}, roll_ios{0};
QuickPID lift_pid(&lift_ios.input, &lift_ios.output, &lift_ios.setpoint);
QuickPID pitch_pid(&pitch_ios.input, &pitch_ios.output, &pitch_ios.setpoint);
QuickPID roll_pid(&pitch_ios.input, &pitch_ios.output, &pitch_ios.setpoint);

SmartHinge pitch_hinge(
    pitch_pid,
    pitch_ios,
    angle<domain::continuous, unit::radians>::from(0),   // GUILLAUME: hard stop LOW bound     ******************************************
    angle<domain::continuous, unit::radians>::from(57)); // GUILLAUME: hard stop HIGH bound   ******************************************
SmartHinge roll_hinge(
    roll_pid,
    roll_ios,
    angle<domain::continuous, unit::radians>::from(0),  // GUILLAUME: hard stop LOW bound    ******************************************
    angle<domain::continuous, unit::radians>::from(0)); // GUILLAUME: hard stop HIGH bound   ******************************************
SmartHinge lift_hinge(
    lift_pid,
    lift_ios,
    angle<domain::continuous, unit::degrees>::from(5).template translate<unit::radians>(),    // GUILLAUME: hard stop LOW bound   ******************************************
    angle<domain::continuous, unit::degrees>::from(220).template translate<unit::radians>()); // real max is 320, but we're limiting it to that for now

/* field centric */
NavX navx;
angle<domain::continuous, unit::degrees> field_centric_offset{0};
PID_ios fc_ios{0}; // setpoint will always be 0;
QuickPID field_centric_pid(&fc_ios.input, &fc_ios.output, &fc_ios.setpoint);
FieldCentric field_centric(field_centric_pid, fc_ios);

/**
 * ====
 * MAIN
 * ====
 */
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
    manip_belt_a.write(1500);
    manip_belt_b.write(1500);
    field_centric_pid.SetOutputSum(0);
    roll_pid.SetOutputSum(0);
    pitch_pid.SetOutputSum(0);
    lift_pid.SetOutputSum(0);
}

CrcLib::Timer print_timer, battery_low_timeout;

ReadPWM lift_PWM(LIFT_E_p), pitch_PWM(MANIP_PITCH_E_p), roll_PWM(MANIP_ROLL_E_p);
PwmToAngleConverter lift_converter, pitch_converter, roll_converter;
angles::AngleMovingAvg lift_averager(0.2), pitch_averager(0.2), roll_averager(0.2);

float input, output, setpoint = 0;
QuickPID pid(&input, &output, &setpoint,
             FIELD_CENTRIC_P, FIELD_CENTRIC_I, FIELD_CENTRIC_D, QuickPID::Action::reverse);

             /**
 * ============
 * SETUP & LOOP
 * ============
 */

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

// Leds
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
void init_state_machine()
{
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
const uint8_t roll_buffer = 10; // 10 degrees de tolérance pour différencier les pièces verticales des horizontales

const uint8_t default_bitmap = B00000000;
auto bitmap = default_bitmap;
uint8_t lecture_manette()
{
    if (CrcLib::ReadDigitalChannel(BUTTON::SELECT))
        bitmap &= (1 << LEDMode::POSITION_MOTEUR);
}

LEDMode update_state_machine()
{
    if (digitalRead(BEAM_p)) // si on a une piece dans le manip
    {
        uint16_t pich, roll, lift;
        pich = pitch_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value;
        roll = roll_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value;
        lift = lift_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value;

        roll %= 180;
        if (roll > 90 - roll_buffer && roll < 90 + roll_buffer) // si on a une piece verticale
        {
            return RECUPERE_PIECE_VERTICALE;
        }
        else if (roll > 180 - roll_buffer && roll > 0 + roll_buffer) // sinon on a une piece horizontale
        {
            return RECUPERE_PIECE_HORIZONTALE;
        }
    }
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
    Wire.begin();

    CrcLib::Initialize();

    CrcLib::InitializePwmOutput(WHEEL_FL_M_p, false);
    CrcLib::InitializePwmOutput(WHEEL_FR_M_p, true);
    CrcLib::InitializePwmOutput(WHEEL_BL_M_p, false);
    CrcLib::InitializePwmOutput(WHEEL_BR_M_p, true);

    CrcLib::InitializePwmOutput(LIFT_L_M_p, true);
    CrcLib::InitializePwmOutput(LIFT_R_M_p, false);

    CrcLib::InitializePwmOutput(MANIP_PITCH_M_p, false);
    CrcLib::InitializePwmOutput(MANIP_ROLL_M_p, false);

    pinMode(BEAM_p, INPUT);

    manip_belt_a.attach(MANIP_BELT_A_p);
    manip_belt_b.attach(MANIP_BELT_B_p);
    manip_belt_a.write(1500);
    manip_belt_b.write(1500);

    print_timer.Start(PRINT_TIMER_DELAY);
    controller_poll_timer.Start(CONTROLLER_POLL_DELAY);

    CONFIG_FIELD_CENTRIC_PID(field_centric_pid);
    field_centric_pid.SetMode(QuickPID::Control::automatic); // starts the PID
    CONFIG_LIFT_PID(lift_pid);
    lift_pid.SetMode(QuickPID::Control::automatic);
    CONFIG_PITCH_PID(pitch_pid);
    pitch_pid.SetMode(QuickPID::Control::automatic);
    CONFIG_ROLL_PID(roll_pid);
    roll_pid.SetMode(QuickPID::Control::automatic);

    pid.SetMode(QuickPID::Control::automatic);
    pid.SetSampleTimeUs(1000 / FIELD_CENTRIC_SAMPLE_FREQ_HZ);
    pid.SetOutputLimits(-FIELD_CENTRIC_OUTPUT_LIM, FIELD_CENTRIC_OUTPUT_LIM);
}

void loop()
{
    CrcLib::Update();

    // RESET STATE MACHINE
    bitmap = default_bitmap;

    /**
     * ------
     * GUARDS
     * ------
     */

    static float battery_voltage_limit = 11.0;
    if (false && CrcLib::GetBatteryVoltage() < battery_voltage_limit)
    {
        // TODO: figure out something more graceful.
        battery_voltage_limit = 15.0f;
        Serial.println("Battery LOW: " + String(CrcLib::GetBatteryVoltage()));
        soft_kill();
        return;
    }

    if (true && !CrcLib::IsCommValid())
    {
        // block everything if controller is not connected
        soft_kill();
        Serial.println("no com");
        return;
    }

    /**
     * ----------------
     * CONTROLLER INPUT
     * ----------------
     */

    auto joysticks_raw = (JoystickPair){
        .left = {
            .x = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X),
            .y = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_Y),
        },
        .right = {
            .x = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_X),
            .y = (float)CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_Y),
        }};

    uint8_t trig_L = int(CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_L)) + 128;
    uint8_t trig_R = int(CrcLib::ReadAnalogChannel(ANALOG::GACHETTE_R)) + 128;

    /**
     * ------------------
     * SENSOR AQUISITIONS
     * ------------------
     */

    uint32_t manip_pitch_signal;
    bool pitch_read = pitch_PWM.read(manip_pitch_signal);
    uint32_t manip_roll_signal;
    bool roll_read = roll_PWM.read(manip_roll_signal);
    uint32_t lift_height_signal;
    bool lift_read = lift_PWM.read(lift_height_signal);

    bool beam_obstructed = CrcLib::GetDigitalInput(BEAM_p);

    NavX::Heading h = navx.read();

    /**
     * ---------------
     * DATA PROCESSING
     * --------------
     */

    auto joysticks_clean = (JoystickPair){
        .left = {
            .x = (float)clean_joystick_input(joysticks_raw.left.x),
            .y = (float)clean_joystick_input(joysticks_raw.left.y),
        },
        .right = {
            .x = (float)clean_joystick_input(joysticks_raw.right.x),
            .y = (float)clean_joystick_input(joysticks_raw.right.y),
        }};

    lift_averager.add(lift_converter.convert(lift_height_signal)
                          .template convert<domain::mirror>());
    pitch_averager.add(pitch_converter.convert(manip_pitch_signal)
                           .template convert<domain::mirror>());
    roll_averager.add(roll_converter.convert(manip_roll_signal)
                          .template convert<domain::mirror>());

    float robot_rotation = NavX::stupid_fix(h.yaw);

    auto current_rotation = angle<domain::continuous, unit::degrees>::from(robot_rotation + field_centric_offset.value);

    /**
     * ---------------
     * PID SHENANIGANS
     * ---------------
     */

    // NOTE: this is where we calculate the set_targets
    {
        /* FIELD CENTRIC*/
        if (abs(joysticks_clean.right.x) > 5 || abs(joysticks_clean.right.y) > 5)
        {
            // TODO: ugly fucking line, clean up eventually. but for now, it works.
            auto ang = angle<domain::continuous, unit::degrees>::from(180 - (atan2(joysticks_clean.right.x, joysticks_clean.right.y) * 180 / PI));
            field_centric.set_target(ang);
        }
    }

    if (controller_poll_timer.IsFinished())
    {
        controller_poll_timer.Start(CONTROLLER_POLL_DELAY);
        {
            /* LIFT */
            const float lift_speed = M_PI / 10;           // set value in rads
            auto current_lift_angle = lift_hinge._target; //  lift_averager.calc();
            if (trig_R)
            {
                auto val = current_lift_angle.value + lift_speed;
                val = val > angle<domain::continuous, unit::radians>::max_a() ? angle<domain::continuous, unit::radians>::max_a() : val;
                lift_hinge.set_target(angle<domain::continuous, unit::radians>::from(val));
            }
            else if (trig_L)
            {
                auto val = current_lift_angle.value - lift_speed;
                val = val < angle<domain::continuous, unit::radians>::min_a() ? angle<domain::continuous, unit::radians>::min_a() : val;
                lift_hinge.set_target(angle<domain::continuous, unit::radians>::from(val));
            }
        }
        {
            /* PITCH */
            auto current = pitch_averager.calc().template convert<domain::continuous>();
            auto speed = M_PI/10; // 0.1pi/sec
            if (CrcLib::ReadDigitalChannel(BUTTON::ARROW_UP))
            {
                float v = current.value + speed;
                if (v > angle<domain::continuous, unit::radians>::max_a())
                {
                    v = angle<domain::continuous, unit::radians>::max_a();
                }
                pitch_hinge.set_target({v});
            }
            if (CrcLib::ReadDigitalChannel(BUTTON::ARROW_DOWN))
            {
                float v = current.value - speed;
                if (v < angle<domain::continuous, unit::radians>::min_a())
                {
                    v = angle<domain::continuous, unit::radians>::min_a();
                }
                pitch_hinge.set_target({v});
            }
        }
    }

    /**
     * --------
     * FEATURES
     * --------
     */

    if (CrcLib::ReadDigitalChannel(BUTTON::SELECT))
    {
        /* RESET FIELD CENTRIC REFERENCE*/
        field_centric_offset = angle<domain::continuous, unit::degrees>::from(field_centric_offset.value - current_rotation.value);
    }

    if (true && CrcLib::ReadDigitalChannel(BUTTON::COLORS_DOWN))
    {
        /* PRECISION MODE */
        joysticks_clean.left.x *= PRECISION_MODE_REDUCTION;
        joysticks_clean.left.y *= PRECISION_MODE_REDUCTION;
        joysticks_clean.right.x *= PRECISION_MODE_REDUCTION;
        joysticks_clean.right.y *= PRECISION_MODE_REDUCTION;

        bitmap &= (1 << LEDMode::VITESSE_LENTE);
    }
    else
    {
        bitmap &= (1 << LEDMode::VITESSE_DEFAULT);
    }

    if (false && field_centric.update(current_rotation))
    {
        /* PID-DRIVEN FIELD CENTRIC HOLONOMIC */
        // Convert joystick inputs to field-centric
        NavX::FieldCentricInput robotCentric = NavX::convertToRobotCentric(
            joysticks_clean.left.y, // Forward
            joysticks_clean.left.x, // Strafe
            fc_ios.output,          // Rotation
            -current_rotation.convert<angles::domain::mirror>().value);

        // Apply converted values to motors
        CrcLib::MoveHolonomic(
            robotCentric.forward,
            robotCentric.rotation,
            robotCentric.strafe,
            WHEEL_FL_M_p, WHEEL_BL_M_p, WHEEL_FR_M_p, WHEEL_BR_M_p);
    }

    if (true && lift_hinge.update(lift_averager.calc()
                                      .template convert<domain::continuous>()))
    {
        /* PID-DRIVEN LIFT */
        CrcLib::SetPwmOutput(LIFT_L_M_p, lift_ios.output);
        CrcLib::SetPwmOutput(LIFT_R_M_p, lift_ios.output);
    }

    if (false && pitch_hinge.update(pitch_averager.calc()
                                        .template convert<domain::continuous>()))
    {
        /* PID-DRIVEN MANIPULATOR PITCH */
        CrcLib::SetPwmOutput(MANIP_PITCH_M_p, pitch_ios.output);
    };

    if (false && roll_hinge.update(roll_averager.calc()
                                       .template convert<domain::continuous>()))
    {
        /* PID-DRIVEN MANIPULATOR ROLL */
        CrcLib::SetPwmOutput(MANIP_PITCH_M_p, pitch_ios.output);
    };

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_RIGHT))
    {
        /* MANIPULATOR BELTS */
        auto speed_L = map(trig_L, 0, 255, 1000, 2000);
        auto speed_R = map(trig_R, 0, 255, 1000, 2000);
        manip_belt_a.writeMicroseconds(speed_L);
        manip_belt_b.writeMicroseconds(speed_R);
    }
    else
    {
        manip_belt_a.writeMicroseconds(1500);
        manip_belt_b.writeMicroseconds(1500);
    }

    if (CrcLib::ReadDigitalChannel(BUTTON::START))
    {
        Serial.println("softkilling");
        soft_kill();
    }

    /**
     * ----------------
     * SERIAL REPORTING
     * ----------------
     */
    if (print_timer.IsFinished())
    {
        print_timer.Start(PRINT_TIMER_DELAY);

        /* battery voltage */
        if (false)
        {
            Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));
        }

        /* beam state */
        if (false)
        {
            Serial.println("beam: " + String(beam_obstructed));
        }

        /* reading encoders */
        if (true)
        {
            Serial.print("l: " + String(lift_read) +
                         "\tp: " + String(pitch_read) +
                         "\tr: " + String(roll_read));
            Serial.print("\t|    ");
            Serial.print("l: " + String(lift_converter.convert(lift_height_signal).template translate<unit::degrees>().value) +
                         "\tp: " + String(pitch_converter.convert(manip_pitch_signal).template translate<unit::degrees>().value) +
                         "\tr: " + String(roll_converter.convert(manip_roll_signal).template translate<unit::degrees>().value));
            Serial.print("\t|    ");
            Serial.println("l: " + String(lift_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value) +
                           "\tp: " + String(pitch_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value) +
                           "\tr: " + String(roll_averager.calc().template convert<domain::continuous>().template translate<unit::degrees>().value));
        }

        /* controller trigger states */
        if (false)
        {
            Serial.println("triggers:\tL:" + String(trig_L) + "\tR: " + String(trig_R));
        }

        /* cmp expected rotation with curent rotation*/
        if (false)
        {
            Serial.println("expected: " + String(field_centric._target.value) + "\tactual: " + String(current_rotation.value));
        }

        /* field centric PID info */
        if (false)
        {
            Serial.println("input: " + String(fc_ios.input) + "\toutput: " + String(fc_ios.output));
        }

        /* lift PID info */
        if (true)
        {
            Serial.println("input: " + String(lift_ios.input) + "\toutput: " + String(lift_ios.output) + "\ttarget: " + String(lift_hinge._target.template translate<unit::degrees>().value));
        }

        /* pitch PID info */
        if (false)
        {
            Serial.println("input: " + String(pitch_ios.input) + "\toutput: " + String(pitch_ios.output) + "\ttarget: " + String(pitch_hinge._target.template translate<unit::degrees>().value));
        }

        /* orientation */
        if (false)
        {
            Serial.println("current orientation: " + String(current_rotation.value));
        }
        // Serial.println("input: " + String(input) + "\toutput: " + String(output));

        // state machine
        current_state_machine = update_state_machine();
        led_Show(current_state_machine, strip1);
    }
}