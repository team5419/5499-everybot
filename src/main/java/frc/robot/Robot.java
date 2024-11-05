// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
    private final VictorSPX rightBackMotor = new VictorSPX(1);
    private final TalonSRX rightFrontMotor = new TalonSRX(2);
    private final TalonSRX leftBackMotor = new TalonSRX(3);
    private final TalonSRX leftFrontMotor = new TalonSRX(4);
    private final VictorSPX shooterTop = new VictorSPX(5);
    private final VictorSPX shooterBottom = new VictorSPX(6);

    // 0 is the USB port to be used as indicated on Driver Station
    private XboxController controller = new XboxController(0);

    private final double SPEED = 1;
    private final double TURN_SPEED = 0.4;

    private boolean readying = false;
    private boolean ready = false;

    /**
      * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Declares new led object at PWM port 3
        // AddressableLED led = new AddressableLED(3);
        // // Creates buffer
        // AddressableLEDBuffer led_buffer = new AddressableLEDBuffer(5);
        // // Sets RBG value of the led at index 0
        // led_buffer.setRGB(0,0,255,0);
        // // Updates data to the buffer
        // led.setData(led_buffer);
        // // Continously writes data to the lecd from the buffer
        // led.start();
    }

    /**
      * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test modes.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    }

    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        rightBackMotor.follow(rightFrontMotor);
        rightFrontMotor.setInverted(false);
        rightBackMotor.setInverted(InvertType.OpposeMaster);

        leftBackMotor.follow(leftFrontMotor);
        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(InvertType.FollowMaster);

        // shooterTop.setInverted(true);
        // shooterBottom.setInverted(true);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double yInput = controller.getRawAxis(1);
        double xInput = controller.getRawAxis(0);
        double rightTriggerInput = controller.getRightTriggerAxis();
        double leftTriggerInput = controller.getLeftTriggerAxis();

        leftFrontMotor.set(ControlMode.PercentOutput, yInput * SPEED - xInput * TURN_SPEED);
        rightFrontMotor.set(ControlMode.PercentOutput, yInput * SPEED + xInput * TURN_SPEED);
        
        // Shooter intake
        if (!readying) {
          // Maybe change this to a constant motor output
          shooterTop.set(ControlMode.PercentOutput, -leftTriggerInput);
          shooterBottom.set(ControlMode.PercentOutput, -leftTriggerInput);
        }

        if (controller.getRightBumperPressed()) {
          readying = true;
          // Ready shooter here
        }
    }
}