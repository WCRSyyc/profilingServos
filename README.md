# Profiling Servos
Analyze the characteristics of a continuous turn servo

Or *at least* gather the data needed to create a profile externally

The WCRS [Omni3 Follower](https://github.com/WCRSyyc/omni3-follower "3 (omni-)wheel line follower") robots uses continuous turn servos for movement.  After initial manual calibration (servos are NOT all created equal), replacing a dead servo resulted in the need to calibarate again.  Since the initial line follower code was written, the Arduino Servo library has added a .writeMicroseconds method.  One of the limitations previously, was getting a sufficient range of useable setting values for speed control.  For that bot, speed control was VERY important.  Controlling direction is achived by setting the speed of each wheel / servo independantly, based on calculations from the paper [Three omni-directional wheels control on a mobile robot](
https://www.researchgate.net/publication/228786543_Three_omni-directional_wheels_control_on_a_mobile_robot)

Switching to Servo.writeMicroseconds() gives MUCH finer speed control than Servo.write().  For these servos, a range of about 250 usable values.  Instead of 14.  Much easier for the speed and direction control logic to find *good* numbers.

Anyway, this led to the need for new calibration information all around, plus the knowledge that it is not likely to be a one (more) time event.  This project is one of the results.  Another result, the opto-interrupter sensor, needs to be documented else where.  Basicly, it uses a transmissive optical sensor, and a piece of cardboard temporarily taped to the wheel, to allow detection of the time taken for full or partial servo rotations.

Once this got started, it took on a life of its own.  In the process discovering just how much variabilty there can be between supposedly identical tests of a single sero.

## Things discovered (about *these* servos)
* Clockwise and counterclockwise maximum speeds are different
* Speed at a fixed setting varies based on the rotation angle
  * At supposed constant speed, the wheel speeds up and slows down during a single rotation
* There is several percent variation in timings of what should be near identical conditions.
