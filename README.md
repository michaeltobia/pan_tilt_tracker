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

- mvBlueFox-MLC200w Board-level, global shutter, gray scale video camera

- Edmunds 35.0mm FL, No IR-Cut Filter, f/2, Micro Video Lens

    - 35mm Lens requires a 5mm Length S-Mount Extension Tube due to abnormal back focal length (18mm!)


- Sharp 2Y0A710 100-500cm laser range finder

- Two LewanSoul LX-16A Bus Servos

- LewanSoul Servo BusLinker, used to configure and control the two LX-16A Bus Servos

- Two Samsung 25R 18650 Batteries

- Teensy LC, for analog single reading from the range finder

- Nscope, used to test (and, during prototyping, power) the Sharp range finder

![alt text](/images/overview.jpg)


## Software
Using the mvBlueFox camera in ROS requires both the mvBlueFox mvImpact Driver
and [KumarRobotics bluefox2](https://github.com/KumarRobotics/bluefox2) ROS driver package

Using the LX-16A servos along with the LewanSoul BusLinker requires their own
Bus Servo Terminal to change the servo IDs. A Windows PC is required for this
program (Windows Defender may pick the download up as a virus.) As for
communicating with the servos during operation [shaneyake's LX-16A servo.py](https://github.com/shaneyake/LX-16A/blob/master/servo.py) was used
extensively in this project.

`image_geometry` and `camera_calibration` packages were also used extensively
in this project.

#### balltrack.py

`balltrack.py` was used to color segment the camera image, draw a circle contour
around the found object, track the center point of the circle, and publish the
error difference between the circle center and the image frame center. An example
of it running, overlaid on a unaltered camera feed, can be seen below.

![alt text](/images/rqt_objtrk.png)

`balltrack.py` also subscribes to `/(camera_serial)/CameraInfo` to pull the camera's
calibration file. It then uses `image_geometry`'s `PinholeCameraModel` to simulate
the camera in an ideal state, and use  `projectPixelTo3dRay` to find the unit vector
that points to the tracked object. In the future, this unit vector will be used to
create servo commands and, when multiplied with the range finder's distance measurement,
find the location of the tracked object relative to the camera.

#### rangefind.py

`rangefind.py` is very simple relative to the other nodes. A simple analog pin reading
program was placed on the Teensy LC, which sends the signal, converted back to voltage,
over a serial port. This voltage is then converted to distance by an equation obtained
through careful calibration and curve fitting. Also, since the range finder suffers from
dramatic noise `rangefind.py` averages the analog readings over a 5 item simple running average.


#### servocontrol.py
`servocontrol.py` mostly contains code pulled from shaneyake's `servo.py` to handle
the UART communication protocol with the servos. The protocol can be found on
LewanSoul's webside. `servocontrol.py` subscribes to the center error messages
from `balltrack.py` and makes a rudimentary attempt at control. Unfortunately, this
does not yet work.

## Future Work

#### Actual servo control

Since `balltrack.py` uses projectPixelTo3dRay and publishes unit vectors pointing
to the tracked object's center, it should be possible to gather error information in
the form of the angle difference the 3dRay makes compared to a ray normal to the
camera image sensor. Obtaining this error would mean much simpler servo control, as
the commands sent to the servos would essentially just be the angle of the 3dRay unit
vector relative to a normal vector.

#### URDF and TF

A very quick next step will be creating a URDF file for the entire assembly (including
a yet unmade base) and using that to transform the vector pointing at the object relative
to the camera to the fixed world frame. This is a necessary step in reaching the project
goal.

#### Trajectory prediction

As mentioned above, the original intent of this project was to be able to predict
a thrown object's trajectory. Once proper servo control and tf data is obtained, this
will be the next and nearly final step. 
