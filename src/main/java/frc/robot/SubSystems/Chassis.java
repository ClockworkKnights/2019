/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.SubSystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Add your docs here.
 */
public class Chassis {

    private TalonSRX LF;
    private VictorSPX LB;
    private TalonSRX RF;
    private VictorSPX RB;

    public Chassis(TalonSRX _LF, VictorSPX _LB, TalonSRX _RF, VictorSPX _RB) {
        LF = _LF;
        LB = _LB;
        RF = _RF;
        RB = _RB;
    }

    public void forward(double speed) {
        LF.set(ControlMode.PercentOutput, speed);
        LB.set(ControlMode.PercentOutput, speed);
        RF.set(ControlMode.PercentOutput, speed);
        RB.set(ControlMode.PercentOutput, speed);
    }

    public void pan(double speed) {
        LF.set(ControlMode.PercentOutput, -speed);
        LB.set(ControlMode.PercentOutput, speed);
        RF.set(ControlMode.PercentOutput, speed);
        RB.set(ControlMode.PercentOutput, -speed);
    }

    public void turn(double speed) {
        LF.set(ControlMode.PercentOutput, speed);
        LB.set(ControlMode.PercentOutput, speed);
        RF.set(ControlMode.PercentOutput, -speed);
        RB.set(ControlMode.PercentOutput, -speed);
    }

    public void omnimotion(double forward, double turn, double strafe) {
        LF.set(ControlMode.PercentOutput, forward + turn + strafe);
        RF.set(ControlMode.PercentOutput, forward - turn - strafe);
        LB.set(ControlMode.PercentOutput, forward + turn - strafe);
        RB.set(ControlMode.PercentOutput, forward - turn + strafe);
    }

    public void power_left(double speed) {
        LF.set(ControlMode.PercentOutput, speed);
        LB.set(ControlMode.PercentOutput, speed);
    }

    public void power_right(double speed) {
        RF.set(ControlMode.PercentOutput, speed);
        RB.set(ControlMode.PercentOutput, speed);
    }

}
