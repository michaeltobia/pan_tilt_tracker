## Winter Project: Pan-Tilt Object Tracker for Projectile Trajectory Perdiction
#### Northwestern University | MSR
#### Michael Tobia

## Project Goal

The goal of this project is to use a high-speed, global shutter camera mounted
on servos to track a thrown object. The ultimate goal is to predict the
ultimate position of the projectile, and for another robot (likely Baxter or
Sawyer) to catch the ball. If the tracking portion of the project is fast enough,
it is feasible to have the future location of the ball found and sent fast enough
for another robot to catch it, though the time window is very short, around
a few hundreds of milliseconds.

## Hardware

The hardware used in this project is as follows:

*mvBlueFox-MLC200w Board-level, global shutter, grey scale video camera

*Edmunds 35.0mm FL, No IR-Cut Filter, f/2, Micro Video Lens

    ** 35mm Lens requires a 5mm Length S-Mount Extension Tube due to abnormal back focal length (18mm!)

*Sharp 2Y0A710 100-500cm laser range finder

*Two LewanSoul LX-16A Bus Servos

*LewanSoul Servo BusLinker, used to configure and control the two LX-16A Bus Servos

*Two Samsung 25R 18650 Batteries

*Teensy LC, for analog single reading from the range finder



## Software
Using the mvBlueFox camera in ROS requires both the mvBlueFox mvImpact Driver
and [KumarRobotics bluefox2](https://github.com/KumarRobotics/bluefox2) ROS driver package

Using the LX-16A servos along with the LewanSoul BusLinker requires their own
Bus Servo Terminal to change the servo IDs. A Windows PC is required for this
program (Windows Defender may pick the download up as a virus.) As for
communicating with the servos during operation [shaneyake's LX-16A servo.py](https://github.com/shaneyake/LX-16A/blob/master/servo.py) was used
extensively in this project.
