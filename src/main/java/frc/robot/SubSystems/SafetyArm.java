/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.SubSystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class SafetyArm {

    public TalonSRX ArmMotor;
    public DigitalInput SafetySwitchUp;
    public DigitalInput SafetySwitchDown;

    public SafetyArm(TalonSRX _ArmMotor, DigitalInput _SafetySwitchUp, DigitalInput _SafetySwitchDown) {
        ArmMotor = _ArmMotor;
        SafetySwitchUp = _SafetySwitchUp;
        SafetySwitchDown = _SafetySwitchDown;
    }

    public void Set(double value) // Positive:Down, Negative:Up
    {
        if (value > 0 && SafetySwitchDown.get() == false) // Down
        {
            ArmMotor.set(ControlMode.PercentOutput, value);
        } else if (value < 0 && SafetySwitchUp.get() == false) // Up
        {
            ArmMotor.set(ControlMode.PercentOutput, value);
            System.out.println("Not Triggered");
        } else {
            ArmMotor.set(ControlMode.PercentOutput, 0);
            System.out.println("Triggered");
        }
    }

}
