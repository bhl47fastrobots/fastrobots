# Lab 4 Report

In this lab, we hooked up the two dual motor drivers, demonstrated that we can control them from the Artemis, stripped down our RC car, and mounted / connected our kit components to the car. Furthermore, we demonstrated the RC car moving under open-loop control.

## Prelab

Below is a wiring diagram of the setup, with everything attached:

We want to power the Artemis and the motor drivers/motors from separate batteries because the motors consume high current (and can be pulsed too). This may cause issues with the power supply on the Artemis, which is sensitive and needs to be more precise. Also, this means that the high-current-carrying wires coming from the motor battery to the motors will not be running near the Artemis. If they did, they may emit EMI and interfere with the proper operation of the signals coming out of the Artemis.

## Lab Tasks

### PWM Outputs for Motor Drivers (_Tasks 1 and 2_)

In the first step, we soldered our motor drivers together according to the wiring diagram and connected each motor driver to a power supply. The battery provides 3.7 V, so to mimic the battery, we set the power supply to output 3.7 V. Since aren't driving any motors yet, the motor drivers should not draw much current at all, so we can set the current limit for the driver to 30 or 40 mA. 

Then, we uploaded this simple Arduino code to the Artemis and probed the two output wires of each motor driver to verify that it is outputting a PWM signal:

```cpp
#include <Arduino.h>

#define MTR1_IN1 A3
#define MTR1_IN2 4
#define MTR2_IN1 A5
#define MTR2_IN2 6

int speed = 0;
int up = 1; // up = |speed| increasing
int direction = 1; // 1 = forward, -1 = backward

void setup() {
  pinMode(MTR1_IN1, OUTPUT);
  pinMode(MTR1_IN2, OUTPUT);
  pinMode(MTR2_IN1, OUTPUT);
  pinMode(MTR2_IN2, OUTPUT);
}

void loop() {
  if (direction == 1 && up == 1) {
    analogWrite(MTR1_IN1, speed);
    analogWrite(MTR1_IN2, 0);

    analogWrite(MTR2_IN1, speed);
    analogWrite(MTR2_IN2, 0);

    speed++;
    
    if (speed == 255) {
      up = 0;
    } 
  } else if (direction == 1 && up == 0) {
      analogWrite(MTR1_IN1, speed);
      analogWrite(MTR1_IN2, 0);

      analogWrite(MTR2_IN1, speed);
      analogWrite(MTR2_IN2, 0);

      speed--;

    if (speed == 0) {
      up = 1;
      direction = -1;
    }
  } else if (direction == -1 && up == 1) {
    analogWrite(MTR1_IN1, 0);
    analogWrite(MTR1_IN2, speed);

    analogWrite(MTR2_IN1, 0);
    analogWrite(MTR2_IN2, speed);

    speed++;

    if (speed == 255) {
      up = 0;
    }
  } else {
    analogWrite(MTR1_IN1, 0);
    analogWrite(MTR1_IN2, speed);

    analogWrite(MTR2_IN1, 0);
    analogWrite(MTR2_IN2, speed);

    speed--;

    if (speed == 0) {
      up = 1;
      direction = 1;
    }
    
    delay(50);
  }
}
```

This code sweeps the PWM output from 0 to 255 on one of the control pins for both motors (forward), and then sweeps the PWM output from 0 to 255 on the other control pin for both motors (backward). Here is a video showing the PWM output:



Here is an oscilloscope screen capture of the waveform generated at a fixed PWM value of `128`, which is a 50% duty cycle:

### Motors Running from Power Supply (_Task 4_)

Next, we attached the motors to the outputs of the motor drivers as shown in the wiring diagram. Here are videos of one motor running:


And here is a video of the other motor running:

### Motors Running from Battery (_Task 5_)

Next, we attached the power supply wires for the motor drivers to the battery connector cables, plugged in the battery, and tested that both motors can run off of battery power. Here is the video showing this test:



### Mounting Hardware Inside Car (_Task 7_)

Next, we mounted everything inside the car. Double sided tape wasn't cutting it for me, so I ended up using electrical tape and duct tape to secure everything down. I refrained from using glue since glue is difficult to remove, in case repairs on the car are needed. Below is a picture of my finished car, with indicators pointing out relevant components:

Below is a video of the car on battery power running on the ground:

### Open Loop Drive Straight (_Task 9_)

Next, we added a calibration factor into the code so that the robot purposely sends a smaller duty cycle to one motor relative to the other. This is since the motors are intrinsically not exactly the same (and the motor drivers are also not exactly the same). This calibration factor tries to correct for that. Here the code on the Arduino side which implements the calibration factor:

```cpp

```

Here is a video demonstrating that the robot can drive in an approximately straight line for 6 feet:

### Open Loop Control (_Task 10_)

Finally, we add some turns to the video from the previous task to complete our demonstration of open-loop, untethered control of the robot:



### Lower Limit PWM (_Task 8, Task 12_)

### `analogWrite()` Frequency Discussion (_Task 11_)



## Acknowledgements