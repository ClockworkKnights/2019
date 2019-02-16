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
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.SubSystems.SafetyArm;
import frc.robot.SubSystems.SmartChassis;

@SuppressWarnings("deprecation")

public class Robot extends SampleRobot implements PIDOutput {

    // Sensors
    public static AnalogInput LineTracker = new AnalogInput(0);
    public static AHRS AHRSSensor = new AHRS(SerialPort.Port.kMXP);
    public static DigitalInput SafetySwitchUp = new DigitalInput(0);
    public static DigitalInput SafetySwitchDown = new DigitalInput(1);

    // Motors
    public static TalonSRX MotorLF = new TalonSRX(0);
    public static VictorSPX MotorLB = new VictorSPX(1);
    public static TalonSRX MotorRF = new TalonSRX(2);
    public static VictorSPX MotorRB = new VictorSPX(3);
    public static VictorSPX Roller1 = new VictorSPX(4);
    public static VictorSPX Roller2 = new VictorSPX(8);
    public static VictorSPX LifterL = new VictorSPX(5);
    public static VictorSPX LifterR = new VictorSPX(6);
    public static TalonSRX ArmMotor = new TalonSRX(7);
    public static SafetyArm Arm = new SafetyArm(ArmMotor, SafetySwitchUp, SafetySwitchDown);

    public static Chassis Chas = new Chassis(MotorLF, MotorLB, MotorRF, MotorRB);
    public static SmartChassis SChas = new SmartChassis(Chas, AHRSSensor);

    // Cylinders
    public static Compressor Comp = new Compressor(10);
    public static Solenoid Pusher = new Solenoid(10, 2);
    public static Solenoid Clipper = new Solenoid(10, 1);
    public static DoubleSolenoid Expander = new DoubleSolenoid(10, 0, 4);
    public static Solenoid Backup = new Solenoid(10, 3);

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
        Roller1.configFactoryDefault();
        Roller2.configFactoryDefault();
        LifterL.configFactoryDefault();
        LifterR.configFactoryDefault();
        ArmMotor.configFactoryDefault();

        // Set the Motor "Reversed"
        MotorLF.setInverted(true);
        MotorLB.setInverted(true);
        LifterR.setInverted(true);
        ArmMotor.setInverted(true);

        // Config the MagEncoders on the Motor Controllers
        MotorLF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        MotorRF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        ArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        // Set the Encoder "Reversed"
        ArmMotor.setSensorPhase(true);

        // Reset the sensors
        ArmMotor.setSelectedSensorPosition(0);
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
        SChas.GoStraight(500, 0.3);
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
        arm_controller.target_value = ArmMotor.getSelectedSensorPosition();

        boolean allowLift = false;

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

            // OneKey Movement TODO: Test Onekey
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

            // Arm Motion (Positive: Down, Negative: Up)
            {
                // Buttons
                if (stick.getRawAxis(2) >= 0.05 && ArmMotor.getSelectedSensorPosition() < 205000
                        && !SafetySwitchDown.get()) {
                    arm_controller.target_value = arm_controller.target_value + 900.0;
                }
                if (stick.getRawButton(5) && ArmMotor.getSelectedSensorPosition() > 100 && !SafetySwitchUp.get()) {
                    arm_controller.target_value = arm_controller.target_value - 1000.0;
                }

                if (ArmMotor.getSelectedSensorPosition() < 0) {
                    ArmMotor.setSelectedSensorPosition(0);
                    arm_controller.target_value = 0;
                }

                // Speed Limit of the PID_Controller
                arm_controller.PIDUpdate_pos(ArmMotor.getSelectedSensorPosition());
                if (arm_controller.result > 0.8)
                    arm_controller.result = 0.8;
                else if (arm_controller.result < -0.8)
                    arm_controller.result = -0.8;
                Arm.Set(arm_controller.result);
            }

            // Ball Roller
            {
                if (stick.getRawButton(6)) {
                    Roller1.set(ControlMode.PercentOutput, 1);
                    Roller2.set(ControlMode.PercentOutput, 1);
                } else {
                    if (stick.getRawAxis(3) > 0.2) {
                        Roller1.set(ControlMode.PercentOutput, 0);
                        Roller2.set(ControlMode.PercentOutput, 0);
                        Timer.delay(0.2);
                        Roller1.set(ControlMode.PercentOutput, -1);
                        Roller2.set(ControlMode.PercentOutput, -1);
                        while (stick.getRawAxis(3) > 0.2) {
                        }
                    } else if (ArmMotor.getSelectedSensorPosition() > 28000) {
                        Roller1.set(ControlMode.PercentOutput, 0.2);
                        Roller2.set(ControlMode.PercentOutput, 0.2);
                    }
                }
            }

            // Elevator
            {
                // Forward
                if (stick.getPOV() == 90) {
                    LifterL.set(ControlMode.PercentOutput, 1);
                    LifterR.set(ControlMode.PercentOutput, 1);
                    use_correction = false; // Don't move while
                }
                // Backward
                else if (stick.getPOV() == 180) {
                    LifterL.set(ControlMode.PercentOutput, -0.5);
                    LifterR.set(ControlMode.PercentOutput, -0.5);
                    use_correction = false;
                    allowLift = true;
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
                            Arm.Set(0.001);
                        } else {
                            Arm.Set(0);
                        }
                    arm_controller.target_value = ArmMotor.getSelectedSensorPosition();
                }
            }

            System.out.println("SwitchUp:" + SafetySwitchUp.get() + "   Down:" + SafetySwitchDown.get() + "   Arm:"
                    + ArmMotor.getSelectedSensorPosition() + "   Yaw:" + AHRSSensor.getYaw() + "   ChassisL:"
                    + MotorLF.getSelectedSensorPosition());
            Timer.delay(0.005);

        }
        orient_controller.close();

        stopAllMotors();

    }

    @Override
    public void test() {
        // Reset the Controllers to prevent residual values
        MotorLF.configFactoryDefault();
        MotorLB.configFactoryDefault();
        MotorRF.configFactoryDefault();
        MotorRB.configFactoryDefault();
        Roller1.configFactoryDefault();
        Roller2.configFactoryDefault();
        LifterL.configFactoryDefault();
        LifterR.configFactoryDefault();
        ArmMotor.configFactoryDefault();

        // Set the Motor "Reversed"
        MotorLF.setInverted(true);
        MotorLB.setInverted(true);
        LifterR.setInverted(true);
        ArmMotor.setInverted(true);

        // Config the MagEncoders on the Motor Controllers
        MotorLF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        MotorRF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        ArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        // Set the Encoder "Reversed"
        ArmMotor.setSensorPhase(true);

        // Reset the sensors
        ArmMotor.setSelectedSensorPosition(0);
        AHRSSensor.zeroYaw();

        // Initialize the Cylinders
        Expander.set(Value.kForward);
        Clipper.set(false);
        Pusher.set(false);
        Backup.set(false);

        while (isTest() && isEnabled()) {
            System.out.println("SwitchUp:" + SafetySwitchUp.get() + "   Down:" + SafetySwitchDown.get() + "   Arm:"
                    + ArmMotor.getSelectedSensorPosition() + "   Yaw:" + AHRSSensor.getYaw() + "   ChassisL:"
                    + MotorLF.getSelectedSensorPosition());
            LifterL.set(ControlMode.PercentOutput, -stick.getRawAxis(1));
            LifterR.set(ControlMode.PercentOutput, -stick.getRawAxis(1));
            Timer.delay(0.005);
        }
    }

    public void stopAllMotors() {
        MotorLF.set(ControlMode.PercentOutput, 0);
        MotorLB.set(ControlMode.PercentOutput, 0);
        MotorRF.set(ControlMode.PercentOutput, 0);
        MotorRB.set(ControlMode.PercentOutput, 0);
        Roller1.set(ControlMode.PercentOutput, 0);
        Roller2.set(ControlMode.PercentOutput, 0);
        LifterL.set(ControlMode.PercentOutput, 0);
        LifterR.set(ControlMode.PercentOutput, 0);
        Arm.Set(0);
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
