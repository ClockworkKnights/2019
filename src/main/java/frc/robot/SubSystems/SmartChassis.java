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
        return input;
    }

    public void GoStraight(int tick, double maxspeed) {
        PIDController orient_controller = new PIDController(0.02, 0, 0, Gyro, this);
        orient_controller.setAbsoluteTolerance(2);
        orient_controller.setSetpoint(Gyro.getYaw());
        orient_controller.enable();
        Timer.delay(0.1);

        System.out.println((Chas.LF.getSelectedSensorPosition() + Chas.RF.getSelectedSensorPosition()) / 2.0);

        PID_Controller move_controller = new PID_Controller();
        move_controller.Kp = 0.00035;
        move_controller.Ki = 0.00001;
        move_controller.Kd = 0.00050;
        move_controller.i_useful = 500;
        move_controller.error_tolerance = 50;
        move_controller.target_value = (Chas.LF.getSelectedSensorPosition() + Chas.RF.getSelectedSensorPosition()) / 2.0
                + tick;
        for (int i = 0; i < 120; i++) {
            move_controller
                    .PIDUpdate_pos((Chas.LF.getSelectedSensorPosition() + Chas.RF.getSelectedSensorPosition()) / 2.0);
            Chas.omnimotion(speedlimit(move_controller.result, maxspeed),
                    speedlimit(PID_Result, Math.min(move_controller.result, maxspeed)), 0); // TODO
            if (Math.abs(move_controller.error) < move_controller.error_tolerance && move_controller.derivative <= 50)
                break;
            Timer.delay(0.05);
        }
        Chas.forward(0);
        orient_controller.close();
    }

    public void TurnTo(int angle, double maxspeed) {
        PID_Controller move_controller = new PID_Controller();
        move_controller.Kp = 0.022;
        move_controller.Ki = 0.003;
        move_controller.Kd = 0.03;
        move_controller.error_tolerance = 2;
        move_controller.target_value = angle;
        for (int i = 0; i < 90; i++) {
            move_controller.PIDUpdate_pos(Gyro.getYaw());
            Chas.turn(speedlimit(move_controller.result, maxspeed));
            System.out.println("Error:" + move_controller.error + "T:" + speedlimit(move_controller.result, maxspeed));
            if (Math.abs(move_controller.error) < move_controller.error_tolerance)
                break;
            Timer.delay(0.05);
        }
        Chas.forward(0);
    }

    // public void PanStraight(double tick, double maxspeed) {
    // PIDController orient_controller = new PIDController(0.02, 0, 0, Gyro, this);
    // orient_controller.setAbsoluteTolerance(2);
    // orient_controller.setSetpoint(Gyro.getYaw());
    // orient_controller.enable();
    // Timer.delay(0.1);

    // System.out.println((Chas.LF.getSelectedSensorPosition() -
    // Chas.RF.getSelectedSensorPosition()) / 2.0);

    // PID_Controller move_controller = new PID_Controller();
    // move_controller.Kp = 0.00035;
    // move_controller.Ki = 0.00001;
    // move_controller.Kd = 0.00050;
    // move_controller.i_useful = 500;
    // move_controller.error_tolerance = 50;
    // move_controller.target_value = (Chas.LF.getSelectedSensorPosition() -
    // Chas.RF.getSelectedSensorPosition()) / 2.0
    // + tick;
    // for (int i = 0; i < 120; i++) {
    // move_controller
    // .PIDUpdate_pos((Chas.LF.getSelectedSensorPosition() -
    // Chas.RF.getSelectedSensorPosition()) / 2.0);
    // Chas.omnimotion(0, speedlimit(PID_Result, Math.min(move_controller.result,
    // maxspeed)),
    // speedlimit(move_controller.result, maxspeed)); // TODO
    // System.out.println(move_controller.error + " " +
    // speedlimit(move_controller.result, maxspeed));
    // if (Math.abs(move_controller.error) < move_controller.error_tolerance &&
    // move_controller.derivative <= 50)
    // break;
    // Timer.delay(0.05);
    // }
    // Chas.forward(0);
    // orient_controller.close();
    // }

    @Override
    public void pidWrite(double output) {
        PID_Result = output;
    }
}
