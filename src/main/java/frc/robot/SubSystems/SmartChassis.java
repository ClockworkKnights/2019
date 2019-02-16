/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.SubSystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class SmartChassis implements PIDOutput {

    public Chassis Chas;
    public AHRS Gyro;

    private double PID_Result;

    public SmartChassis(Chassis _Chas, AHRS _Gyro) {
        Chas = _Chas;
        Gyro = _Gyro;
    }

    private double speedlimit(double input, double limit) {
        if (Math.abs(input) > Math.abs(limit)) {
            if (input > 0) {
                input = Math.abs(limit);
            } else {
                input = -Math.abs(limit);
            }
        }
        return limit;
    }

    public void GoStraight(int tick, double maxspeed) {
        PIDController orient_controller = new PIDController(0.02, 0, 0, Gyro, this);
        orient_controller.setAbsoluteTolerance(2);
        orient_controller.setSetpoint(Gyro.getYaw());
        orient_controller.enable();
        PID_Controller move_controller = new PID_Controller();
        move_controller.Kp = 0.005;
        move_controller.Kd = 0.001;
        move_controller.error_tolerance = 20;
        move_controller.target_value = Chas.LF.getSelectedSensorPosition() + tick;
        while (true) {
            move_controller.PIDUpdate_pos(Chas.LF.getSelectedSensorPosition());
            Chas.omnimotion(speedlimit(move_controller.result, 0.3), speedlimit(PID_Result, 0.3), 0);
            if (Math.abs(move_controller.error) < 3 * move_controller.error_tolerance
                    && move_controller.derivative <= 500)
                break;
            Timer.delay(0.05);
        }
        Chas.forward(0);
        orient_controller.close();
    }

    public void Turn(double angle, double maxspeed) {
        PID_Controller move_controller = new PID_Controller();
        move_controller.Kp = 0.5;
        move_controller.Kd = 0.1;
        move_controller.error_tolerance = 1;
        move_controller.target_value = Gyro.getYaw() + angle;
        while (true) {
            move_controller.PIDUpdate_pos(Chas.LF.getSelectedSensorPosition());
            Chas.turn(speedlimit(move_controller.result, 0.3));
            if (Math.abs(move_controller.error) < 3 * move_controller.error_tolerance
                    && move_controller.derivative <= 0.5)
                break;
            Timer.delay(0.05);
        }
        Chas.forward(0);
    }

    @Override
    public void pidWrite(double output) {
        PID_Result = output;
    }
}
