RPyPCA9685
==========

The class PCA9685 uses the quick2wire API to talk to the
"Adafruit 16-Channel 12-bit PWM/Servo Driver I2C interface"
which uses the PCA9685 IC

It provides the simple method set_position(servo_id, position)
in order to drive up to 16 PWM servos.