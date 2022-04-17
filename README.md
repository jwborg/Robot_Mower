
My project to built a autonomous lawn mower, powered by a Raspberry Pi and Arduino:

- A user interface based upon TkInter with command menu and simulated map. As the user interface runs on the Rpi, during mowing the intend is to connect to the Rpi by VNC over wifi from another computer. The HMI allows setting up the mowing area manually or retrieve an existing mowing area from file. It has a simulated map of the mowing area and will show the position of the mower after completion of a leg of the path. 

- Location determination using Apriltags. The Apriltags color was changed to cyan/red to prevent camera bleeding in bright sunlight. The mower uses a Raspberry PI HQ camera with rotating mirror to determine the distance to the Apriltags in view and tri-laterates  the 3 best tags to calculate it's own position. The position is then used to correct the path. 

- The mower will be powered by 2 windscreen wiper dc motors. The dc motors are speed controlled by PWM using a Arduino. The Arduino uses a PID controller to synchronise progress of both motors and facilitate turning. The Arduino receives motor commands from the RPI over I2C. RPi is the master, Arduino is the slave in the I2C communication.

- The mowing module receives the mowing area coordinates and requested mowing pattern. It then calculates the desired path of the mower and powers the dc motors accordingly. Three mowing patterns are planned: spiral, traditional and random. At set intervals the mowing is interrupted, and the mower's position is determined to correct it's path.

Apriltags by: APRIL Robotics Laboratory at the University of Michigan  https://april.eecs.umich.edu/
PID controller by: Phil's lab  https://github.com/pms67/PID
Magnetic compass QMC5883 driver by: Niccolo Rigacci <niccolo@rigacci.org> https://github.com/RigacciOrg/py-qmc5883l