# Yaesu-G2800-remote-control

Leamington G-2800DXA remote control

Design based on Yeasu G-800 sketch by Glen Popiel, KW5GP. from “More Arduino Projects for Amateur Radio”, Chapter 18.
Sketch implemented in Arduino UNO R3 with prototype shield (for connection to G-2800DXA) and relay shield board.

- SUPPORTED SERIAL PROTOCOL
- R	Rotate clockwise
- L	Rotate counterclockwise
- A	Rotation STOP
- S	Rotation STOP
- Mxxx	Move to azimuth xxx (000-450) must be 3 digits
- C	Request current position.  Reply: +0yyy where yyy = azimuth
- Kx	Relay board (has nothing to do with rotator)  x=0-4
- 	These are general purpose Form C relay contacts (SPDT) for some future function.
- 		K0 = de-energizes all relays
- 		K1 = energizes relay 1 (disables other 3)
- 		K2 = energizes relay 2 (disables other 3)
- 		K3 = energizes relay 3 (disables other 3)
- 		K4 = energizes relay 4 (disables other 3)
- Xz	Rotation speed.  (z=1-4).  Default power-up = X4
- 		X1 = Low speed (25% PWM ~ 1.25V)
- 		X2 = Middle speed 1 (50% PWM ~ 2.5V)
- 		X3 = Middle speed 2 (75% PWM ~ 3.75V)
- 		X4 = High speed (100% PWM ~ 5V) 
- Fx	Calibration (x=0-5)
- 		F0 = 0 degree calibration
- 		F5 = 450 degree calibration
- 		F? = Display calibration values
-       CALIBRATION PROCEDURE:
- 	Enable calibration switch (on prototype board - toward outside of board.
- 	If an 'F' command is sent while disabled, controller replies "Calibration Disabled"
- 	Connect voltmeter to “position” test point on prototype board and ground lug.
- 	Manually Rotate CW to end stop (450 degrees).
- 	Adjust “OUT VOL ADJ” pot on rear of G-2800DXA to 5.00 volts on meter.
- Send F5 command
- 	Manually Rotate CCW to end stop (0 degrees).	Send F0 command
- 	Disable calibration switch (on prototype board - toward middle of board.
- 	Cycle power on Arduino UNO board.
