/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.SubSystems;

/**
 * Add your docs here.
 */
public class PID_Controller {
    public double Kp;
    public double Ki;
    public double Kd;

    public double error_tolerance;
    public double target_value;
    public double now_value;

    public double error;
    public double last_error;
    public double prev_error;

    public double i_useful;
    public double integral;
    public double derivative;

    public double det;
    public boolean b_antidec;
    public double v_antidec;

    public double result;

    public double PIDUpdate_pos(double now) {

        // Give Value to the PIDController
        now_value = now;

        // Error
        error = target_value - now_value;

        // Integral
        integral += error;
        if (Math.abs(error) < error_tolerance) {
            integral = 0;
        } else if (Math.abs(error) > i_useful) {
            integral = 0;
        } else if (error * last_error <= 0) {
            integral = 0;
        }

        // Derivative
        derivative = error - last_error;

        // Lasterror
        last_error = error;

        // Result
        result = Kp * error + Ki * integral + Kd * derivative;
        return result;
    }

    public double PIDUpdate_vel(double now) {

        // Give Value to the PIDController
        now_value = now;

        // Error
        error = target_value - now_value;

        // Result
        det = Kp * (error - last_error) + Ki * error + Kd * (error - 2 * last_error + prev_error);

        // Decrease Suppression
        if (det < 0 && b_antidec)
            det = det * v_antidec;

        result += det;

        // Refresh error
        prev_error = last_error;
        last_error = error;

        return result;
    }

}
