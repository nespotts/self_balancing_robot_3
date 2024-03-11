


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

        analogWrite(en_pin, fabs(speed));
    }

};