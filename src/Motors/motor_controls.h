


class Motor {
private:
    int en_pin;
    int dir_pin1;
    int dir_pin2;

public:
    Motor(int p_en_pin, int p_dir_pin1, int p_dir_pin2) {
        en_pin = p_en_pin;
        dir_pin1 = p_dir_pin1;
        dir_pin2 = p_dir_pin2;

        setup();
    }

    // setup motor pins
    void setup() {
        pinMode(en_pin, OUTPUT);
        pinMode(dir_pin1, OUTPUT);
        pinMode(dir_pin2, OUTPUT);

        digitalWrite(en_pin, LOW);
        digitalWrite(dir_pin1, LOW);
        digitalWrite(dir_pin2, LOW);

        analogWriteFrequency(en_pin, 36621.09);
        analogWriteResolution(12);
    }

    void command_motor(int speed) {
        // Set  Motor Direction
        if (speed < 0) {
            // reverse
            digitalWrite(dir_pin1, LOW);
            digitalWrite(dir_pin2, HIGH);
        } else if (speed > 0) {
            // forward
            digitalWrite(dir_pin1, HIGH);
            digitalWrite(dir_pin2, LOW);
        } else {
            // stop
            digitalWrite(dir_pin1, LOW);
            digitalWrite(dir_pin2, LOW);
        }
        // apply correction factor
        motor_speed_output = calculateSpeedOutput(fabs(speed));
        analogWrite(en_pin, motor_speed_output);
    }

    uint32_t timer;
    uint32_t interval = 5;
    float exp_factor = 10.36;
    float inflection_point_x = 1000.0;
    float inflection_point_y = 2750.0;
    float linear_slope;
    float linear_intercept;
    double motor_speed_input;
    double motor_speed_output;

    void config_speed_output_params() {
        interval = 5;
        motor_speed_input = 0;
        linear_slope = (4095.0 - inflection_point_y) / (4095.0 - inflection_point_x);
        linear_intercept = 4095.0 - linear_slope * 4095.0;
        // Serial.println(linear_slope);
        // Serial.println(linear_intercept);
    }

    void config_motor_run() {
        // loop thru motor speeds (0-4095)
        if ((millis() - timer) >= interval) {
            if (motor_speed_input > 5000) {
                motor_speed_input = -1;
                command_motor(0);
            } else if (motor_speed_input > 4095) {
                command_motor(0);
            } else {
                command_motor(motor_speed_input);
            }
            // Serial.println(motor_speed_input);
            motor_speed_input++;
            timer = millis();
        }
    }

    /// @brief  apply correction to speed input in order to achieve a linear speed output
    /// @param input 
    /// @return 
    double calculateSpeedOutput(double input) {
        double output;
        if (input <= inflection_point_x) {
            output = pow(input, 1.0/exp_factor) / (pow(inflection_point_x, 1.0/exp_factor) / inflection_point_y);
        } else {
            // scale remaining output between inflection point  and 4095 from inflection point to 4095
            output = linear_slope * input + linear_intercept;
        }
        return output;
    }

};