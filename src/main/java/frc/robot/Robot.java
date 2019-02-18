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

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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
    public static AHRS AHRSSensor = new AHRS(SerialPort.Port.kMXP);
    public static DigitalInput SafetySwitchUp = new DigitalInput(0);
    public static DigitalInput SafetySwitchDown = new DigitalInput(1);
    public static AnalogInput IFDist = new AnalogInput(0);

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

        // Vision Thread
        Thread visionThread = new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(160, 120);
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Main Camera", 160, 120);
            Mat mat = new Mat();
            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                    outputStream.notifyError(cvSink.getError());
                    continue;
                }
                // Put a rectangle on the image
                // Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new
                // Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
            }
        });
        visionThread.setDaemon(true);
        visionThread.start();

        stopAllMotors();

    }

    @Override
    public void autonomous() {

        AHRSSensor.zeroYaw();
        MotorLF.setSelectedSensorPosition(0);
        Timer.delay(0.6);
        Chas.forward(0.5);
        Timer.delay(0.5);
        Chas.forward(0);
        Timer.delay(0.8);
        Chas.forward(-0.3);
        Timer.delay(0.6);
        Chas.forward(0);
        Timer.delay(0.5);
        SChas.GoStraight(10000, 0.5);
        SChas.TurnTo(-30, 0.5);
        SChas.GoStraight(9000, 0.5);
        SChas.TurnTo(0, 0.5);
        SChas.GoStraight(10000, 0.5);
        SChas.TurnTo(90, 0.5);

    }

    @Override
    public void operatorControl() {

        stopAllMotors();
        resetAllCylinders();

        double forward, turn, strafe; // Temporary Values of the Joystick Axis

        // PIDController orient_controller = new PIDController(0.02, 0, 0, AHRSSensor,
        // this);
        // orient_controller.setAbsoluteTolerance(2);
        // orient_controller.setSetpoint(AHRSSensor.getAngle());
        // orient_controller.enable();

        PID_Controller orient_controller = new PID_Controller();
        orient_controller.Kp = 0.02;
        orient_controller.Ki = 0;
        orient_controller.Kd = 0;
        orient_controller.error_tolerance = 2;
        orient_controller.target_value = AHRSSensor.getAngle();

        boolean user_turning = false;
        boolean use_correction = true;
        double woodplate_angle = 0;

        PID_Controller arm_controller = new PID_Controller();
        arm_controller.Kp = 0.00005;
        arm_controller.Ki = 0;
        arm_controller.Kd = 0;
        arm_controller.error_tolerance = 20;
        arm_controller.target_value = ArmMotor.getSelectedSensorPosition();

        boolean allowLift = false;
        boolean RollerForce = false;

        Comp.setClosedLoopControl(true);

        while (isOperatorControl() && isEnabled()) {

            // Chassis Driving
            {
                // Read Joysticks and Move Headless in a Straight way
                if (Math.abs(stick.getRawAxis(1)) <= 0.6)
                    forward = -stick.getRawAxis(1) * 0.75;
                else if (-stick.getRawAxis(1) > 0)
                    forward = -stick.getRawAxis(1) / 8.0 * 11.0 - 0.375;
                else
                    forward = -stick.getRawAxis(1) / 8.0 * 11.0 + 0.375;

                if (Math.abs(stick.getRawAxis(4)) <= 0.6)
                    turn = stick.getRawAxis(4) * 0.75;
                else if (stick.getRawAxis(4) > 0)
                    turn = stick.getRawAxis(4) / 8.0 * 11.0 - 0.375;
                else
                    turn = stick.getRawAxis(4) / 8.0 * 11.0 + 0.375;

                if (Math.abs(stick.getRawAxis(0)) <= 0.6)
                    strafe = stick.getRawAxis(0) * 0.75;
                else if (stick.getRawAxis(0) > 0)
                    strafe = stick.getRawAxis(0) / 8.0 * 11.0 - 0.375;
                else
                    strafe = stick.getRawAxis(0) / 8.0 * 11.0 + 0.375;

                forward *= 0.8;
                turn *= 0.8;
                strafe *= 0.9;

                // User wants to turn, set flag
                if (Math.abs(stick.getRawAxis(4)) > 0.05)
                    user_turning = true;
                // User wanted to turn
                else if (user_turning) {
                    user_turning = false;
                    // Record the new orient
                    orient_controller.target_value = (AHRSSensor.getAngle());
                }
                // User don't want to turn, control now
                if (!user_turning && use_correction) {
                    orient_controller.PIDUpdate_pos(AHRSSensor.getAngle());
                    turn = orient_controller.result; // PID(See the pidWrite function)
                }
                Chas.omnimotion(forward, turn, strafe);
            }

            // OneKey Movement TODO: Test Onekey
            {
                if (stick.getRawButton(2))
                    orient_controller.target_value = (woodplate_angle
                            + Math.round((AHRSSensor.getAngle() - woodplate_angle) / 360.0) * 360.0);
                if (pad.getRawButton(2))
                    orient_controller.target_value = (30 + Math.round((AHRSSensor.getAngle() - 30.0) / 360.0) * 360.0);
                if (pad.getRawButton(3))
                    orient_controller.target_value = (90 + Math.round((AHRSSensor.getAngle() - 90.0) / 360.0) * 360.0);
                if (pad.getRawButton(4))
                    orient_controller.target_value = (150
                            + Math.round((AHRSSensor.getAngle() - 150.0) / 360.0) * 360.0);
                if (pad.getRawButton(5))
                    orient_controller.target_value = (180
                            + Math.round((AHRSSensor.getAngle() - 180.0) / 360.0) * 360.0);
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
                if (stick.getRawButton(5) && ArmMotor.getSelectedSensorPosition() > 1500 && !SafetySwitchUp.get()) {
                    arm_controller.target_value = arm_controller.target_value - 1500.0;
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
                    RollerForce = true;
                } else if (stick.getRawAxis(3) > 0.2) {
                    Roller1.set(ControlMode.PercentOutput, -1);
                    Roller2.set(ControlMode.PercentOutput, -1);
                    RollerForce = false;
                } else if (ArmMotor.getSelectedSensorPosition() > 28000 && RollerForce) {
                    Roller1.set(ControlMode.PercentOutput, 0.2);
                    Roller2.set(ControlMode.PercentOutput, 0.2);
                } else {
                    Roller1.set(ControlMode.PercentOutput, 0);
                    Roller2.set(ControlMode.PercentOutput, 0);
                }
                if (stick.getPOV() == 0) {
                    RollerForce = false;
                }
            }

            // Elevator
            {
                // Forward
                if (stick.getPOV() == 90 && allowLift) {
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
                    woodplate_angle = AHRSSensor.getAngle();
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
                    + ArmMotor.getSelectedSensorPosition() + "   Yaw:" + AHRSSensor.getAngle() + "   ChassisL:"
                    + MotorLF.getSelectedSensorPosition() + "  Dist:" + (IFDist.getVoltage() - 0.11) / 12.44);
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
                    + ArmMotor.getSelectedSensorPosition() + "   Yaw:" + AHRSSensor.getAngle() + "   ChassisL:"
                    + MotorLF.getSelectedSensorPosition() + "  Dist:" + IFDist.getVoltage());
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
