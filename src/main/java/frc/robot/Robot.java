/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.SubSystems.Chassis;
import frc.robot.SubSystems.PID_Controller;

@SuppressWarnings("deprecation")

public class Robot extends SampleRobot implements PIDOutput {

    // Motors
    public static TalonSRX MotorLF = new TalonSRX(0);
    public static VictorSPX MotorLB = new VictorSPX(1);
    public static TalonSRX MotorRF = new TalonSRX(2);
    public static VictorSPX MotorRB = new VictorSPX(3);
    public static VictorSPX Roller = new VictorSPX(4);
    public static VictorSPX LifterL = new VictorSPX(5);
    public static VictorSPX LifterR = new VictorSPX(6);
    public static TalonSRX Arm = new TalonSRX(7);

    private final Chassis Chas = new Chassis(MotorLF, MotorLB, MotorRF, MotorRB);

    // Cylinders
    public static Compressor Comp = new Compressor(10);
    public static Solenoid Pusher = new Solenoid(10, 2);
    public static Solenoid Clipper = new Solenoid(10, 1);
    public static DoubleSolenoid Expander = new DoubleSolenoid(10, 0, 4);
    public static Solenoid Backup = new Solenoid(10, 3);

    // Sensors
    public static AnalogInput LineTracker = new AnalogInput(0);
    private final AHRS AHRSSensor = new AHRS(SerialPort.Port.kMXP);

    // Driver Station Miscellaneous
    private final Joystick stick = new Joystick(0);
    private final Joystick pad = new Joystick(1);
    private final SendableChooser<String> Prog = new SendableChooser<>();

    double pi = 3.14159265358979323846264338328;
    double orienter_output; // The output value of the GyroPID on the Chassis

    public Robot() {
    }

    @Override
    public void robotInit() {

        // Reset the Controllers to prevent residual values
        MotorLF.configFactoryDefault();
        MotorLB.configFactoryDefault();
        MotorRF.configFactoryDefault();
        MotorRB.configFactoryDefault();
        Roller.configFactoryDefault();
        LifterL.configFactoryDefault();
        LifterR.configFactoryDefault();
        Arm.configFactoryDefault();

        // Set the Motor "Reversed"
        MotorLF.setInverted(true);
        MotorLB.setInverted(true);
        LifterR.setInverted(true);
        Arm.setInverted(true);

        // Config the MagEncoders on the Motor Controllers
        MotorLF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        MotorRF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        // Set the Encoder "Reversed"
        Arm.setSensorPhase(true);

        // Reset the sensors
        Arm.setSelectedSensorPosition(0);
        AHRSSensor.zeroYaw();

        // Initialize the Cylinders
        Expander.set(Value.kForward);
        Clipper.set(false);
        Pusher.set(false);
        Backup.set(false);

        // TODO: Temporary Prog Chooser
        Prog.addDefault("Prog 1", "1");
        Prog.addDefault("Prog 2", "2");
        Prog.addDefault("Prog 3", "3");

        stopAllMotors();

    }

    @Override
    public void autonomous() {
        int autoSelected = 0;

        switch (autoSelected) {
        case 0:

            break;
        case 1:
        default:

            break;
        }
    }

    @Override
    public void operatorControl() {

        stopAllMotors();
        resetAllCylinders();

        double forward, turn, strafe; // Temporary Values of the Joystick Axis

        PIDController orient_controller = new PIDController(0.02, 0, 0, AHRSSensor, this);
        orient_controller.setAbsoluteTolerance(2);
        orient_controller.setSetpoint(AHRSSensor.getYaw());
        orient_controller.enable();
        boolean user_turning = false;
        boolean use_correction = false;

        PID_Controller arm_controller = new PID_Controller();
        arm_controller.Kp = 0.00005;
        arm_controller.Ki = 0;
        arm_controller.Kd = 0;
        arm_controller.error_tolerance = 20;
        arm_controller.target_value = -Arm.getSelectedSensorPosition();

        Comp.setClosedLoopControl(true);

        while (isOperatorControl() && isEnabled()) {

            // Chassis Driving
            {
                // Read Joysticks and Move Headless in a Straight way
                forward = -stick.getRawAxis(1);
                turn = stick.getRawAxis(4);
                strafe = stick.getRawAxis(0);
                // User wants to turn, set flag
                if (Math.abs(turn) > 0.005)
                    user_turning = true;
                // User wanted to turn
                else if (user_turning) {
                    user_turning = false;
                    // Record the new orient
                    orient_controller.setSetpoint(AHRSSensor.getYaw());
                }
                // User don't want to turn, control now
                if (!user_turning && use_correction) {
                    turn = orienter_output; // PID(See the pidWrite function)
                }
                Chas.omnimotion(forward, turn, strafe);
            }
            // OneKey Movement
            {
                if (pad.getRawButton(1))
                    orient_controller.setSetpoint(0 + Math.round(AHRSSensor.getYaw() / 360.0) * 360.0);
                if (pad.getRawButton(2))
                    orient_controller.setSetpoint(30 + Math.round((AHRSSensor.getYaw() - 30.0) / 360.0) * 360.0);
                if (pad.getRawButton(3))
                    orient_controller.setSetpoint(90 + Math.round((AHRSSensor.getYaw() - 90.0) / 360.0) * 360.0);
                if (pad.getRawButton(4))
                    orient_controller.setSetpoint(150 + Math.round((AHRSSensor.getYaw() - 150.0) / 360.0) * 360.0);
                if (pad.getRawButton(5))
                    orient_controller.setSetpoint(180 + Math.round((AHRSSensor.getYaw() - 180.0) / 360.0) * 360.0);
            }

            // Pusher
            {
                if (stick.getRawButton(1))
                    Pusher.set(true);
                else
                    Pusher.set(false);
            }

            // Elevator Cylinders
            {
                if (stick.getRawButton(3)) {
                    while (stick.getRawButton(3)) {
                    }
                    if (Expander.get() == Value.kForward)
                        Expander.set(Value.kReverse);
                    else
                        Expander.set(Value.kForward);
                }
                if (stick.getRawButton(4)) {
                    while (stick.getRawButton(4)) {
                    }
                    Clipper.set(!Clipper.get());
                }
            }

            // Arm Motion
            {
                // Buttons
                if (stick.getRawAxis(2) >= 0.05 && Arm.getSelectedSensorPosition() < 195000) {
                    Arm.set(ControlMode.PercentOutput, 0.3); // Down
                    arm_controller.target_value = Arm.getSelectedSensorPosition();
                } else if (stick.getRawButton(5) && Arm.getSelectedSensorPosition() > 1000) {
                    Arm.set(ControlMode.PercentOutput, -0.4); // Up
                    arm_controller.target_value = Arm.getSelectedSensorPosition();
                } else { // PID Lock
                    // Speed Limit of the PID_Controller
                    if (arm_controller.result > 0.3)
                        arm_controller.result = 0.3;
                    else if (arm_controller.result < -0.3)
                        arm_controller.result = -0.3;
                    arm_controller.PIDUpdate_pos(Arm.getSelectedSensorPosition());
                    Arm.set(ControlMode.PercentOutput, arm_controller.result);
                }
                // Angle Limit of the PID_Controller
                if (arm_controller.target_value > 190000)
                    arm_controller.target_value = 190000;
                else if (arm_controller.target_value < 1000)
                    arm_controller.target_value = 1000;
                // Manual GROUND KILLER
                if (stick.getRawButton(2))
                    arm_controller.target_value = 240000; // TODO: Use Machinery if possible
            }

            // Ball Roller
            {
                if (stick.getRawButton(6)) {
                    Roller.set(ControlMode.PercentOutput, 1);
                } else {
                    if (stick.getRawAxis(3) > 0.2) {
                        Roller.set(ControlMode.PercentOutput, 0);
                        Timer.delay(0.2);
                        Roller.set(ControlMode.PercentOutput, -1);
                        while (stick.getRawAxis(3) > 0.2) {
                        }
                    } else if (Arm.getSelectedSensorPosition() > 10000)
                        Roller.set(ControlMode.PercentOutput, 0.1);
                }
            }

            // Elevator
            {
                // Forward
                if (stick.getPOV() == 0) {
                    LifterL.set(ControlMode.PercentOutput, 1);
                    LifterR.set(ControlMode.PercentOutput, 1);
                    use_correction = false; // Don't move while
                }
                // Backward
                else if (stick.getPOV() == 180) {
                    LifterL.set(ControlMode.PercentOutput, -0.5);
                    LifterR.set(ControlMode.PercentOutput, -0.5);
                    use_correction = false;
                }
                // Stop
                else {
                    LifterL.set(ControlMode.PercentOutput, 0);
                    LifterR.set(ControlMode.PercentOutput, 0);
                }
                // Manually reopen the orient-correction
                if (stick.getPOV() == 270) {
                    use_correction = true;
                }
                // Put down the arm, ENTER AN INDIVIDUAL MODE, press 7 to exit
                if (stick.getRawButton(8)) {
                    while (!stick.getRawButton(7))
                        if (stick.getRawButton(8)) {
                            Arm.set(ControlMode.PercentOutput, 0.001);
                        } else {
                            Arm.set(ControlMode.PercentOutput, 0);
                        }
                    arm_controller.target_value = Arm.getSelectedSensorPosition();
                }
            }

            orient_controller.close();
            System.out.println("Yaw:" + AHRSSensor.getYaw() + "   Arm:" + Arm.getSelectedSensorPosition());
            Timer.delay(0.005);

        }

        stopAllMotors();

    }

    @Override
    public void test() {
        // Reset the Controllers to prevent residual values
        MotorLF.configFactoryDefault();
        MotorLB.configFactoryDefault();
        MotorRF.configFactoryDefault();
        MotorRB.configFactoryDefault();
        Roller.configFactoryDefault();
        LifterL.configFactoryDefault();
        LifterR.configFactoryDefault();
        Arm.configFactoryDefault();

        // Set the Motor "Reversed"
        MotorLF.setInverted(true);
        MotorLB.setInverted(true);
        LifterR.setInverted(true);
        Arm.setInverted(true);

        // Config the MagEncoders on the Motor Controllers
        MotorLF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        MotorRF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        // Set the Encoder "Reversed"
        Arm.setSensorPhase(true);

        // Reset the sensors
        Arm.setSelectedSensorPosition(0);
        AHRSSensor.zeroYaw();

        // Initialize the Cylinders
        Expander.set(Value.kForward);
        Clipper.set(false);
        Pusher.set(false);
        Backup.set(false);

        while (isTest() && isEnabled()) {
            System.out.println("Yaw:" + AHRSSensor.getYaw() + "   Arm:" + Arm.getSelectedSensorPosition());
            Timer.delay(0.005);
        }
    }

    public void stopAllMotors() {
        MotorLF.set(ControlMode.PercentOutput, 0);
        MotorLB.set(ControlMode.PercentOutput, 0);
        MotorRF.set(ControlMode.PercentOutput, 0);
        MotorRB.set(ControlMode.PercentOutput, 0);
        Roller.set(ControlMode.PercentOutput, 0);
        LifterL.set(ControlMode.PercentOutput, 0);
        LifterR.set(ControlMode.PercentOutput, 0);
        Arm.set(ControlMode.PercentOutput, 0);
    }

    public void resetAllCylinders() {
        Expander.set(Value.kForward);
        Clipper.set(false);
        Pusher.set(false);
        Backup.set(false);
    }

    @Override
    public void pidWrite(double output) {
        orienter_output = output;
    }
}
