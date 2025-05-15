# Lab 12 Report

In this lab, we tried to drive our robot in the map using all of that we have learned through the semester as fast as possible while hitting a desired sequence of waypoints.

## Lab Tasks

### Code Preparation

The first task was to combine code from multiple previous labs into something that I could use to command the robot to do movements on the maze.

I combined code from Labs 5 and 6 to give the robot the ability to do the following:

1. Run orientation control alone to rotate the robot in place toward a certain direction
2. Drive forward at a specified speed while running orientation control to maintain the robot's heading. This movement is ended simply by timeout in the Python code
3. Drive forward using linear control to a desired distance from the wall while running orientation control to maintain the robot's heading

The first movement allows the robot to turn toward a new orientation accurately.

The second movement allows the robot to do a timed drive while maintaining the heading when the nearest wall in front of the robot is too far away for the robot to accurately do linear control with the ToF sensor.

The third movement allows the robot to do a controlled forward drive towards the nearest wall in front of the robot while maintaining the heading.

I also copied the code for the localization procedure in Lab 11 into the code, but (spoiler alert) I didn't end up using it as the robot movements were too inaccurate.

The first thing I did was add a second copy of all of the global variables used in the control: one for the orientation control and one for the linear control. I also made the deadbands of both motors as well as the setpoints for both orientation and linear control settable through bluetooth commands. Here is the relevant code:

```cpp
// tuning constants
float linear_KP = 0.0;
float linear_KI = 0.0;
float linear_KD = 0.0;

float orient_KP = 0.0;
float orient_KI = 0.0;
float orient_KD = 0.0;

#define CALIB_FAC 0.7
#define MAX_INTEGRAL 30.0

const int ctrl_log_size = 5000;
float ctrl_output[ctrl_log_size];
float p_output[ctrl_log_size], i_output[ctrl_log_size], d_output[ctrl_log_size];
float extrap_tof_data[ctrl_log_size];
float linear_setpoint_arr[ctrl_log_size], orient_setpoint_arr[ctrl_log_size];
unsigned long ctrl_times[ctrl_log_size];
int ctrl_arr_ix = 0;
unsigned long ctrl_start_time = 0;

float linear_setpoint = 30.0, orient_setpoint = 0.0;
float linear_integral = 0.0, orient_integral = 0.0;
float linear_deriv = 0.0, orient_deriv = 0.0;
float linear_prev_err = 0.0, orient_prev_err;
float linear_deriv_lpf_alpha = 0.1, orient_deriv_lpf_alpha = 0.1;

int deadband_one = 40, deadband_two = 40;
```

Note all of the doubled sets of variables for the two kinds of control.

Next, in the `handle_command()` function, we expand the `SET_SETPOINTS` command and the `START_PID_MVMT` command to handle the functionality of both the linear and orientation control:

```cpp
switch (cmd_type) {
    // ...
    
    /*
     * This command tells the Artemis to start running PID loop
     */
    case START_PID_MVMT:
        run_pid_loop = true;
        ctrl_start_time = micros();
        linear_integral = 0.0, orient_integral = 0.0;
        linear_prev_err = 0.0, orient_prev_err = 0.0;
        break;
    
    // ...
    
    /*
     * Set the tuning constants of the PID controller
     */
    case SET_TUNING_CONSTS:

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(linear_KP);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(linear_KI);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(linear_KD);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(linear_deriv_lpf_alpha);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(orient_KP);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(orient_KI);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(orient_KD);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(orient_deriv_lpf_alpha);
        if (!success)
            return;
    
        break;
        
    /*
     * Set the tuning constants of the PID controller
     */
    case SET_SETPOINTS:
    
        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(linear_setpoint);
        if (!success)
            return;

        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(orient_setpoint);
        if (!success) {
            return;
        }

        // This variable sets the value of the boolean use_tof
        // Extract the next value from the command string as a float
        value = 0;
        success = robot_cmd.get_next_value(value);
        if (!success) {
            return;
        }
        use_tof = (value == 0) ? false : true;

        success = robot_cmd.get_next_value(deadband_one);
        if (!success) {
            return;
        }

        success = robot_cmd.get_next_value(deadband_two);
        if (!success) {
            return;
        }

        break;

    case RESET_ARRAYS:
        imu_arr_ix = 0;
        tof_arr_ix = -1;
        ctrl_arr_ix = 0;

        break;
} 
```

Note that we also added a command `RESET_ARRAYS` which resets the robot without sending data back (since the entire movement takes a long time to run, the amount of data sent back is very large, so to test quickly, we can just send this command to reset the arrays and run again). 

To combine the linear and orientation control, we modify the `drive()` function to take both a forward speed and differential speed, which are the outputs of the linear PID and orientation PID, respectively. If we are not using linear PID, we simply set the forward speed to be a constant value for that movement:

```cpp
/*
 * Sends the requested forward and differential speed in pwm units to the motors, 
 * with an offset of the DEADBAND defined above
 */
void drive (float forward_speed, float diff_speed) {
    const float stoprange = 3.0;
    int adjusted_pwm_one, adjusted_pwm_two, mtr_one_speed, mtr_two_speed;

    // if control action is small, just drive forward at the desired speed
    if (abs(diff_speed) < stoprange) {
        diff_speed = 0.0;
    } else if (abs(forward_speed) < stoprange) {
        forward_speed = 0.0;
    }

    // otherwise, take care of differential inputs
    // calculated motor one speed, accounting for desired forward speed
    mtr_one_speed = forward_speed + diff_speed;
    mtr_two_speed = forward_speed - diff_speed;

    // send command to motor one
    if (mtr_one_speed > stoprange) {
        adjusted_pwm_one = (int) ((float)mtr_one_speed * CALIB_FAC + deadband_one + 0.5);

        analogWrite(MTR1_IN1, adjusted_pwm_one);
        analogWrite(MTR1_IN2, 0);
    } else if (mtr_one_speed < (stoprange * -1.0)) {
        adjusted_pwm_one = (int) ((float)mtr_one_speed * -1.0 * CALIB_FAC + deadband_one + 0.5);
        
        analogWrite(MTR1_IN1, 0);
        analogWrite(MTR1_IN2, adjusted_pwm_one);
    } else {
        analogWrite(MTR1_IN1, 0);
        analogWrite(MTR1_IN2, 0);
    }

    // send command to motor two
    if (mtr_two_speed > stoprange) {
        adjusted_pwm_two = (int) ((float)mtr_two_speed + deadband_two + 0.5);

        analogWrite(MTR2_IN1, 0);
        analogWrite(MTR2_IN2, adjusted_pwm_two);
    } else if (mtr_two_speed < (stoprange * -1.0)) {
        adjusted_pwm_two = (int) ((float)mtr_two_speed * -1.0 + deadband_two + 0.5);

        analogWrite(MTR2_IN1, adjusted_pwm_two);
        analogWrite(MTR2_IN2, 0);
    } else {
        analogWrite(MTR2_IN1, 0);
        analogWrite(MTR2_IN2, 0);
    }
}
```

Finally, we merge the `run_pid()` functions for the orientation and linear controls as well into one big function. We always run orientation PID control, but only run linear PID sometimes (when the boolean `use_tof` is true, set in the command `SET_SETPOINTS`):

```cpp
void run_pid() {
    unsigned long curr_time = micros();

    // ********************************** ORIENTATION PID SECTION *********************** //
    // calculate error from imu data
    float err = (imu_arr_ix != 0) ? imu_yaw[imu_arr_ix - 1] - orient_setpoint : 0.0;
    float p = 0.0, i = 0.0, d = 0.0, linear_tot = 0.0, orient_tot = 0.0;
    float curr_deriv = 0.0;
    // time step between this control update and previous one, in seconds
    float dt = (ctrl_arr_ix == 0) ? 0.0 : (float)(curr_time - ctrl_times[ctrl_arr_ix - 1]) / 1000000.0;

    // calculate p term
    p = orient_KP * err;

    // calculate i term (rectangular riemann sum)
    // skip if integral is over max integral and we are adding on more with the same sign
    if (!(orient_integral > MAX_INTEGRAL && abs(orient_integral) < abs(orient_integral + err))) {
        orient_integral += err * dt;
    }
    // reset the integral on the zero crossing to get rid of windup
    if (orient_prev_err * err < 0.0) {
        orient_integral = 0.0;
    }
    i = orient_KI * orient_integral;

    // calculate d term
    curr_deriv = (ctrl_arr_ix == 0) ? 0.0 : (err - orient_prev_err) / dt;
    // take care of derivative kick:
    if (orient_prev_err == 0.0) {
        curr_deriv = 0.0;
    }
    orient_deriv = (orient_deriv_lpf_alpha) * curr_deriv + (1.0 - orient_deriv_lpf_alpha) * orient_deriv; // low-pass filter implementation
    d = orient_KD * orient_deriv;
    orient_prev_err = err;

    // sum the three to get the total input
    orient_tot = p + i + d;

    // ************************************* LINEAR PID SECTION *********************** //
    if (use_tof) {
        // get extrapolated tof data
        calculate_tof_data(curr_time);

        // calculate error using extrapolated tof data
        err = extrap_tof_data[ctrl_arr_ix] - linear_setpoint;
        p = 0.0, i = 0.0, d = 0.0;
        curr_deriv = 0.0;

        // calculate p term
        p = linear_KP * err;

        // calculate i term (rectangular riemann sum)
        // skip if integral is over max integral and we are adding on more with the same sign
        if (!(linear_integral > MAX_INTEGRAL && abs(linear_integral) < abs(linear_integral + err))) {
            linear_integral += err * dt;
        }
        // reset the integral on the zero crossing to get rid of windup
        if (linear_prev_err * err < 0.0) {
            linear_integral = 0.0;
        }
        i = linear_KI * linear_integral;

        // calculate d term
        curr_deriv = (ctrl_arr_ix == 0) ? 0.0 : (err - linear_prev_err) / dt;
        // take care of derivative kick:
        if (linear_prev_err == 0.0) {
            curr_deriv = 0.0;
        }
        linear_deriv = (linear_deriv_lpf_alpha) * curr_deriv + (1.0 - linear_deriv_lpf_alpha) * linear_deriv; // low-pass filter implementation
        d = linear_KD * linear_deriv;
        linear_prev_err = err;

        // sum the three to get the total input
        linear_tot = p + i + d;
    } else {
        // if not using tof, the linear setpoint is the forward speed of the robot
        linear_tot = linear_setpoint;
    }

    // send the control action to the wheels
    drive(linear_tot, orient_tot);

    // log the control output if space in array
    if (ctrl_arr_ix < ctrl_log_size) {
        ctrl_output[ctrl_arr_ix] = linear_tot;
        p_output[ctrl_arr_ix] = p;
        i_output[ctrl_arr_ix] = i;
        d_output[ctrl_arr_ix] = d;
        ctrl_times[ctrl_arr_ix] = curr_time;
        linear_setpoint_arr[ctrl_arr_ix] = linear_setpoint;
        orient_setpoint_arr[ctrl_arr_ix] = orient_setpoint;
        ctrl_arr_ix++;
    }
}
```
Notice the `else` statement associated with the `if` statement that turns the linear PID on or off; when we are not using linear PID, the forward speed provided as an argument to the `drive()` function is set to the linear setpoint, allowing us to use the same `SET_SETPOINT` command to run all three movements.

### Commanding the Robot

The important command to understand is the `SET_SETPOINT` command. It takes five arguments:

1. The linear control setpoint, which is used in two ways:
    * When not using the ToF sensor, it is the forward speed of the robot, in PWM units
    * When using the ToF sensor, it is the target distance the robot should stop in front of the wall, in cm
2. The orientation control setpoint, in degrees, measured in absolute angle relative to the worldframe
3. A boolean, 0 or 1, which tells the robot whether to use the ToF sensor for linear control. 0 is false (don't use the ToF sensor), and 1 is true (do use the ToF sensor)
4. The deadband of the right motor, in PWM units
5. The deadband of the left motor, in PWM units

The 4th and 5th arguments are necessary because the deadbands for straight movements (40 for both motors) are very different from the deadbands for on-axis turns (120 for one motor, 140 for the other).

To reiterate, the three movements we execute with the robot are:

1. Run orientation control alone to rotate the robot in place toward a certain direction
2. Drive forward at a specified speed while running orientation control to maintain the robot's heading. This movement is ended simply by timeout in the Python code
3. Drive forward using linear control to a desired distance from the wall while running orientation control to maintain the robot's heading

An example of **movement type 1** is the following Python command:

```python
ble.send_command(CMD.SET_SETPOINTS, "0.0|-45.0|0|120|140")
```

This says: turn the robot in place (forward speed of **0** PWM units) to a heading of **-45.0** degrees, **without** using linear PID control, and with the on-axis turn deadbands of **120** and **140**.

An example of **movement type 2** is the following Python command:

```python
ble.send_command(CMD.SET_SETPOINTS, "30.0|-45.0|0|40|40")
time.sleep(1.2);
```

This says: drive forward with a forward speed of **30** PWM units while maintaining a headin of **-45.0** degrees, **without** using linear PID control, and with the forward-speed deadbands of **40** for both motors. The `time.sleep()` call separating this command the next `SET_SETPOINTS` command dicates how far the robot will drive forward.

An example of **movement type 3** is the following Python command:

```python
ble.send_command(CMD.SET_SETPOINTS, "40.0|60.0|1|40|40")
```

This says: drive forward **with** linear PID control with a goal of ending up **40.0** cm away from the wall in front of the robot, while maintaining a heading of **60.0** degrees, using the forward-speed deadbands of **40** for both motors.

### Discussion of Strategy

Localization was tried, but it was extremely slow. While the localization of the robot did return accurate results (the robot was able to place itself in the map accurately), the movement of the robot to get from point A to point B was simply too inaccurate to make localization any more reliable than simply driving the robot using orientation control (always) and linear control (whenever possible). So, we employed the following strategy:

* For all turns, we use orientation control only (movement type 1)
* For any forward drive where the closest wall was either too far away or could be two different distances depending on the exact positioning of the robot, we used a timed forward drive with orientation control (movement type 2)
* For any forward drive where the closest wall could not be misinterpreted as a different wall, we used linear control and orientation control together (movement type 3)

This turned out to be mostly accurate enough (on a full battery) to make it through the course and very fast.

## Acknowledgements

* Prof. Helbling for setting up the second world towards the end of the lab time
* Sophia Lin (lab partner)
* Jeffery Cai and Adrienne Yoon (giving me some tips on how to go about doing the lab)

