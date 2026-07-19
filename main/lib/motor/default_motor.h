// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFAULT_MOTOR
#define DEFAULT_MOTOR

#include <Arduino.h>
#include "config.h"
#ifdef ESP32
inline void analogWriteFrequency(uint8_t pin, double frequency)
{
  analogWriteFrequency(frequency);
}
#elif defined(PICO)
inline void analogWriteFrequency(double frequency)
{
  analogWriteFreq(frequency);
}
inline void analogWriteFrequency(uint8_t pin, double frequency)
{
  analogWriteFreq(frequency);
}
#endif

#include "motor_interface.h"

class LUNA_MOTOR_DRIVE: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;
        int pwm_max_;

    protected:
        void forward(int pwm) override
        {
	    if (in_a_pin_ < 0) return;
#ifdef USE_SHORT_BRAKE
            analogWrite(in_a_pin_, pwm_max_ - abs(pwm));
            analogWrite(in_b_pin_, pwm_max_); // short brake
#else
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, abs(pwm));
#endif
        }

        void reverse(int pwm) override
        {
	    if (in_a_pin_ < 0) return;
#ifdef USE_SHORT_BRAKE
            analogWrite(in_b_pin_, pwm_max_ - abs(pwm));
            analogWrite(in_a_pin_, pwm_max_); // short brake
#else
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, abs(pwm));
#endif
        }

    public:
        LUNA_MOTOR_DRIVE(float pwm_frequency, int pwm_bits, bool invert, int unused, int in_a_pin, int in_b_pin):
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
	    if (in_a_pin_ < 0) return;
            pwm_max_ = (1 << pwm_bits) - 1;
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }

        LUNA_MOTOR_DRIVE(float pwm_frequency, int pwm_bits, bool invert, int in_a_pin, int in_b_pin):
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
	    if (in_a_pin_ < 0) return;
            pwm_max_ = (1 << pwm_bits) - 1;
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }

        void brake() override
        {
	    if (in_a_pin_ < 0) return;
#ifdef USE_SHORT_BRAKE
            analogWrite(in_a_pin_, pwm_max_);
            analogWrite(in_b_pin_, pwm_max_); // short brake
#else
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, 0);
#endif
        }
};

#endif
