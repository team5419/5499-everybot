package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final VictorSPX rightBackMotor = new VictorSPX(1);
  private final VictorSPX rightFrontMotor = new VictorSPX(2);
  private final TalonSRX leftBackMotor = new TalonSRX(3);
  private final TalonSRX leftFrontMotor = new TalonSRX(4);
  private final TalonSRX shooterTop = new TalonSRX(5);
  private final TalonSRX shooterBottom = new TalonSRX(6);
  // private final TalonSRX groundIntake = new TalonSRX(7);
  private final CANSparkMax coralIntake = new CANSparkMax(43, MotorType.kBrushless);

  // 0 is the USB port to be used as indicated on Driver Station
  private XboxController controller = new XboxController(0);

  // Number of LEDs on the strip
  private final int LED_COUNT = 8;
  // 0 is the port the LED strip is connected to on the RoboRIO
  private AddressableLED led = new AddressableLED(1);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_COUNT);

  // Autonomous
  private final double AUTO_WAIT_DELAY = 9;
  private final boolean AUTO_TAXI = true;

  // Speeds
  private final double SPEED = 1;
  private final double TURN_SPEED = 0.55;
  private final double INTAKE_STRENGTH = 0.5;
  private final double CORAL_INTAKE_STRENGTH = 0.25;

  // Ready
  private final double READY_DELAY = 1.5;
  private boolean ready = false;

  // Trigger input
  private final double TRIGGER_DEADZONE = 0.5;
  private boolean rightTriggerDown = false;
  private boolean leftTriggerDown = false;

  // LED strip
  private int lightIndex = 0;
  private int lightHue = 0;

  // Rumble
  private final double RUMBLE_CHANGE_SPEED = 0.02;
  private double rumble = 0;
  private double rumbleImportant = 0;

  // Delays
  private List<Delay> delays = new ArrayList<>();
  private Delay readyDelay;

  // Ground intake
  private final double GROUND_INTAKE_SPEED = 0.08;

  public class Delay {
    public double startTime;
    public double delayTime;
    public boolean isFinished = false;
    public Callback callback;

    public interface Callback {
      public void callback();
    }

    public Delay(double _delayTime, Callback _callback) {
      startTime = Timer.getFPGATimestamp();
      delayTime = _delayTime;
      callback = _callback;
    }

    public void update() {
      if (Timer.getFPGATimestamp() >= startTime + delayTime && !isFinished) {
        callback.callback();
        isFinished = true;
      }
    }
  }

  // Called when the robot starts up
  @Override
  public void robotInit() {
    rumble = 0;

    rightBackMotor.follow(rightFrontMotor);
    rightFrontMotor.setInverted(false);
    rightBackMotor.setInverted(InvertType.OpposeMaster);

    leftBackMotor.follow(leftFrontMotor);
    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(InvertType.FollowMaster);
  }

  // Called every 20 ms in every mode
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());

    controller.setRumble(RumbleType.kBothRumble, rumbleImportant > 0 ? rumbleImportant : rumble);

    // Update delays
    for (Delay delay : delays) delay.update();
    if (readyDelay != null) readyDelay.update();
  }

  // Called when autonomous is enabled
  @Override
  public void autonomousInit() {
    // Readying
    shooterTop.set(ControlMode.PercentOutput, 1.0);

    Timer.delay(4);

    // Shoot
    shooterBottom.set(ControlMode.PercentOutput, 1.0);

    Timer.delay(AUTO_WAIT_DELAY);

    // Stop shooters
    shooterTop.set(ControlMode.PercentOutput, 0);
    shooterBottom.set(ControlMode.PercentOutput, 0);

    if (AUTO_TAXI) {
      // Taxi
      rightFrontMotor.set(ControlMode.PercentOutput, 1);
      leftFrontMotor.set(ControlMode.PercentOutput, 1);

      Timer.delay(0.5);

      // Stop
      rightFrontMotor.set(ControlMode.PercentOutput, 0);
      leftFrontMotor.set(ControlMode.PercentOutput, 0);
    }

    lightHue = 0;
  }

  // Called every 60 ms in autonomous mode
  @Override
  public void autonomousPeriodic() {

  }

  // Called when teleop mode is enabled
  @Override
  public void teleopInit() {

  }

  // Called every 20 ms in teleop mode
  @Override
  public void teleopPeriodic() {
    double yInput = controller.getRawAxis(1);
    double xInput = controller.getRawAxis(0);
    double rightTriggerRaw = controller.getRightTriggerAxis();
    double leftTriggerRaw = controller.getLeftTriggerAxis();

    // Trigger deadzones
    rightTriggerDown = rightTriggerRaw >= TRIGGER_DEADZONE;
    leftTriggerDown = leftTriggerRaw >= TRIGGER_DEADZONE;

    leftFrontMotor.set(ControlMode.PercentOutput, yInput * SPEED - xInput * TURN_SPEED);
    rightFrontMotor.set(ControlMode.PercentOutput, yInput * SPEED + xInput * TURN_SPEED);

    // Shooting
    // if (rightTriggerDown && ready) {
    //   shooterBottom.set(ControlMode.PercentOutput, 1);
    // } else {
    //   shooterBottom.set(ControlMode.PercentOutput, 0);
    // }

    // Ground intake
    // if (leftTriggerDown) {
    //   groundIntake.set(ControlMode.PercentOutput, GROUND_INTAKE_SPEED);
    // } else {
    //   groundIntake.set(ControlMode.PercentOutput, 0);
    // }

    // Coral intake
    if (leftTriggerDown) {
      coralIntake.set(CORAL_INTAKE_STRENGTH);
    } else if (rightTriggerDown) {
      coralIntake.set(-CORAL_INTAKE_STRENGTH);
    } else {
      coralIntake.set(0);
    }

    // Readying
    // shooterTop.set(ControlMode.PercentOutput, controller.getLeftBumper() ? 1 : 0);

    // if (controller.getLeftBumperPressed()) {
    //   ready = false;
    //   readyDelay = new Delay(READY_DELAY, () -> { ready = true; });
    // }

    // if (controller.getLeftBumperReleased()) {
    //   ready = false;
    // }

    // if (controller.getLeftBumper()) {
    //   rumble = Math.min(rumble + RUMBLE_CHANGE_SPEED, 1);
    // } else {
    //   // Intake
    //   shooterTop.set(ControlMode.PercentOutput, -leftTriggerRaw * INTAKE_STRENGTH);
    //   shooterBottom.set(ControlMode.PercentOutput, -leftTriggerRaw * INTAKE_STRENGTH);

    //   rumble = Math.max(rumble - RUMBLE_CHANGE_SPEED, 0);
    // }
  }

  @Override
  public void teleopExit() {
    rumble = 0;
  }
}
