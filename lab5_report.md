# Lab 5 Report

In this lab, we implemented a PID control loop to control the robot's distance to a wall. We also tried to improve on the refresh rate of the ToF sensors by linearly interpolating the sensor readings between two samples.

## Prelab

### Logging Mechanism for PID Tuning

In this prelab, we were asked to implement a logging mechanism that will be useful for visualizing the performance of our PID controller on the computer. To do this, I implemented it very similarly to Labs 2 and 3 when I was doing sensor characterizations. I made three commands: `START_PID_MVMT`, `STOP_PID_MVMT`, and `SEND_PID_LOGS`. The first two simply set a flag to run the PID controller loop to be `true` or `false`, as well as a couple additional small initialization and ending procedures. The third command sends back all of the log data over Bluetooth to the computer for processing and visualization. To execute PID for a fixed time, I simply waited a certain number of seconds between calling `START_PID_MVMT` and `STOP_PID_MVMT` in the Python code.

On the Arduino side, I added these logging arrays and associated global variables:

```cpp
const int ctrl_log_size = 500;
float ctrl_output[ctrl_log_size];
unsigned long ctrl_times[ctrl_log_size];
int ctrl_arr_ix = 0;
unsigned long ctrl_start_time = 0;
```

In `handle_command()`, I added these `case` sections:

```cpp
/*
 * This command tells the Artemis to start running PID loop
 */
case START_PID_MVMT:
    run_pid_loop = true;
    ctrl_start_time = millis();

    break;

/*
 * This command tells the Artemis to stop running PID loop
 */
case STOP_PID_MVMT:
    run_pid_loop = false;

    analogWrite(MTR1_IN1, 0);
    analogWrite(MTR1_IN2, 0);
    analogWrite(MTR2_IN1, 0);
    analogWrite(MTR2_IN2, 0);

    break;

/*
 * This command tells the Artemis to send back all data and reset logging arrays from most recent PID loop run
 */
case SEND_PID_LOGS:
      // construct string to send back TOF data (data set 1)
      for (int i = 0; i < tof_arr_ix; i++) {
          sprintf(char_arr, "1|%u|%d.%02d|%d.%02d", tof_times[i],
                                        (int) tof_data_one[i], abs((int) (tof_data_one[i] * 100.0) % 100),
                                        (int) tof_data_two[i], abs((int) (tof_data_two[i] * 100.0) % 100));

          tx_estring_value.clear();
          tx_estring_value.append(char_arr);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
      }

      // reset the TOF array index
      tof_arr_ix = -1;

      // construct string to send back control output data (data set 2)
      for (int i = 0; i < ctrl_arr_ix; i++) {
          sprintf(char_arr, "2|%u|%d.%02d", ctrl_times[i],
                                        (int) ctrl_output[i], abs((int) (ctrl_output[i] * 100.0) % 100));

          tx_estring_value.clear();
          tx_estring_value.append(char_arr);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
      }

      // reset the control array index
      ctrl_arr_ix = 0;

      break;
```

Notice that the `START_PID_MVMT` command has the additional command of noting the start time of the movement (to be used during the PID movement to drive forward blindly until we are within sensing distance of the wall). Additionally, the `STOP_PID_MVMT` stops the motors at the end of the movement so that the robot doesn't continue to move once the PID movement has ended.

The `SEND_PID_LOGS` command sends back the ToF sensor readings as well as the output of the PID control loop. This is **NOT** the PWM signal sent to the motors, as there is some extra processing needed for that. This is simply the output of the PID control loop.

The Python side is implemented in almost exactly the same way as in Labs 2 and 3: write a notification handler, call the `START_PID_MVMT` command, wait a certain number of seconds for the movement to finish, call the `STOP_PID_MVMT` command, then call the `SEND_PID_LOGS` command to receive the data. Finally, generate plots of the data to visualize the controller performance. Below is the code:

```python
tof_one = list()
tof_two = list()
tof_times = list()

ctrl_outputs = list()
ctrl_times = list()

# define notification handler
def pid_log_notification_handler(uuid, characteristic):
    s = ble.bytearray_to_string(characteristic)
    strs = s.split('|')

    if (strs[0] == '1'):
        tof_times.append(int(strs[1]))
        tof_one.append(float(strs[2]))
        tof_two.append(float(strs[3]))
    else:
        ctrl_times.append(int(strs[1]))
        ctrl_outputs.append(float(strs[2]))
        
ble.start_notify(ble.uuid['RX_STRING'], pid_log_notification_handler)

# run pid loop for some number of seconds
run_pid_for_time = 5;

ble.send_command(CMD.START_PID_MVMT, "");
time.sleep(run_pid_for_time);
ble.send_command(CMD.STOP_PID_MVMT, "");

# clear the lists, then send command to get data back
tof_one.clear()
tof_two.clear()
tof_times.clear()
ctrl_outputs.clear()
ctrl_times.clear()

ble.send_command(CMD.SEND_PID_LOGS, "");

# Subtract first time from times to get 0-indexed time for both arrays
first_time = int(tof_times[0])

for i in range(len(tof_times)):
    tof_times[i] -= first_time
    tof_times[i] /= 1000.0 # convert to seconds

first_time = int(ctrl_times[0])

for i in range(len(ctrl_times)):
    ctrl_times[i] -= first_time
    ctrl_times[i] /= 1000.0 # convert to seconds

# Make a plot of control data
plt.figure(figsize = (12, 4))
plt.plot(ctrl_times, ctrl_outputs, 'r', label = "Control Output")
plt.xlabel('Time (s)')
plt.ylabel('Control Output (PWM units)')
plt.legend()
plt.show()

# Make a plot of ToF data
plt.figure(figsize = (12, 4))
plt.plot(tof_times, tof_one, 'r', label = "ToF 1 Data")
plt.plot(tof_times, tof_two, 'g', label = "ToF 2 Data")
plt.xlabel('Time (s)')
plt.ylabel('Distance (cm)')
plt.legend()
plt.show()
```

Here is an example plot of the control data from a run that I did:

![prelab_debugging_example](images/lab5/prelab_debugging_example.png)

### Setting Tuning Constants

To be able to update the tuning constants over Bluetooth (instead of re-programming the Artemis each time), I implemented a new command `SET_TUNING_CONSTS`, which is very similar to the `SEND_THREE_FLOATS` command implemented in Lab 1. On the Arduino side:

```cpp
/*
 * Set the tuning constants of the PID controller
 */
case SET_TUNING_CONSTS:

    // Extract the next value from the command string as a float
    success = robot_cmd.get_next_value(KP);
    if (!success)
        return;

    // Extract the next value from the command string as a float
    success = robot_cmd.get_next_value(KI);
    if (!success)
        return;

    // Extract the next value from the command string as a float
    success = robot_cmd.get_next_value(KD);
    if (!success)
        return;
        
    break;
```

On the Python side, this is called like so:

```python
ble.send_command(CMD.SET_TUNING_CONSTS, "4.0|0.3|2.0")
```

## Lab Tasks

### PID Loop Implementation

To implement the algorithm, I introduced the following global variables and constants:

```cpp
// tuning constants
float KP = 4.0;
float KI = 0.0;
float KD = 0.0;

#define CALIB_FAC 0.7
#define DEADBAND 40
#define SETPOINT 30.5


float integral = 0.0;
float prev_err = 0.0;
```

The `DEADBAND` was found in Lab 4. I implemented the basic PID control algorithm as follows:

```cpp
void run_pid() {
    // calculate error
    float err = (tof_arr_ix == 0) ? 0.0 : tof_data_two[tof_arr_ix - 1] - SETPOINT;
    float p = 0.0, i = 0.0, d = 0.0, tot = 0.0;

    // calculate p term
    p = KP * err;

    // calculate i term
    integral += err;
    i = KI * err;

    // calculate d term
    d = KD * (err - prev_err);
    prev_err = err;

    // sum the three to get the total input
    tot = p + i + d;

    // send the control action to the wheels
    straight(tot);

    // log the control output if space in array
    if (ctrl_arr_ix < ctrl_log_size) {
        ctrl_output[ctrl_arr_ix] = tot;
        ctrl_times[ctrl_arr_ix] = millis();
        ctrl_arr_ix++;
    }
}
```

This function is called in the main loop each time we update the ToF sensor data:

```cpp
// While central is connected
while (central.connected()) {
    
    // if want to run pid loop
    if (run_pid_loop) {

        // if space in the TOF sensor array
        if (tof_arr_ix < tof_log_size) {
            // if first measurement, simply start a measurement
            // else, if sensor(s) has/have data ready, take measurement(s) and record, and start a new measurement(s)
            if (tof_arr_ix == -1) {
                myTOF1.startRanging();
                myTOF2.startRanging();
                tof_arr_ix++;
            } else if (myTOF1.checkForDataReady() && myTOF2.checkForDataReady()) {
                tof_data_one[tof_arr_ix] = ((float) myTOF1.getDistance()) / 10.0;
                tof_data_two[tof_arr_ix] = ((float) myTOF2.getDistance()) / 10.0;
                tof_times[tof_arr_ix] = millis();

                myTOF1.clearInterrupt();
                myTOF1.stopRanging();
                myTOF1.startRanging();

                myTOF2.clearInterrupt();
                myTOF2.stopRanging();
                myTOF2.startRanging();

                tof_arr_ix++;

                // run the PID controller / send new pwms to the robot <------ CODE ADDED HERE --------
                run_pid();
            }
        }
    }
```

### Implementation of `straight()`

Notice that the output of the PID control loop in the `run_pid()` function above is used as the argument to the function `straight()` to drive the robot at the specified speed. However, the output of the controller cannot be directly applied to the PWM pins of the motors! First, the controller output is a floating-point number; `analogWrite()` takes integers only. We also want to offset the controller output by the `DEADBAND` so that the PID control output actually drives the robot forward and backward (instead of being unable to overcome static friction and probably having to use a large `KI` value to compensate -- thus increasing overshoot). Finally, the output of the controller is both positive and negative, so we need to translate the sign of the output into which control pins we actually drive our motors with. Below is the implemtation of `straight()` that accounts for all of the aforementioned issues:

```cpp
/*
 * Sends the requested speed in pwm units to the motors, 
 * with an offset of the DEADBAND defined above
 */
void straight(float pwm) {
    const float stoprange = 3.0;
    int adjusted_pwm_one = 0;
    int adjusted_pwm_two = 0;

    if (abs(pwm) < stoprange) {
        analogWrite(MTR1_IN1, 0);
        analogWrite(MTR1_IN2, 0);
        analogWrite(MTR2_IN1, 0);
        analogWrite(MTR2_IN2, 0);
    } else if (pwm >= stoprange) {
        adjusted_pwm_one = (int) (pwm * CALIB_FAC + DEADBAND + 0.5);
        adjusted_pwm_two = (int) (pwm + DEADBAND + 0.5);

        analogWrite(MTR1_IN1, adjusted_pwm_one);
        analogWrite(MTR1_IN2, 0);
        analogWrite(MTR2_IN1, 0);
        analogWrite(MTR2_IN2, adjusted_pwm_two);
    } else {
        adjusted_pwm_one = (int) (pwm * -1.0 * CALIB_FAC + DEADBAND + 0.5);
        adjusted_pwm_two = (int) (pwm * -1.0 + DEADBAND + 0.5);

        analogWrite(MTR1_IN1, 0);
        analogWrite(MTR1_IN2, adjusted_pwm_one);
        analogWrite(MTR2_IN1, adjusted_pwm_two);
        analogWrite(MTR2_IN2, 0);
    }   
}
```

Notice that we also take into acount the `DEADBAND`.

### PID Controller Tuning



## Acknowledgements

XD