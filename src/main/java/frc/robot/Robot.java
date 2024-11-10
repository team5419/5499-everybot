package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

  // Number of LEDs on the strip
  private final int LED_COUNT = 8;
  // 0 is the port the LED strip is connected to on the RoboRIO
  private AddressableLED led = new AddressableLED(1);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_COUNT);

  private final double AUTO_WAIT_DELAY = 9;

  // Speeds
  private final double SPEED = 1;
  private final double TURN_SPEED = 0.55;
  private final double INTAKE_STRENGTH = 0.5;

  // Ready
  private final double READY_DELAY = 1.5;
  private boolean ready = false;

  // Clean mode
  private final double CLEAN_MODE_DRIVE_SPEED = 0.1;
  private final double CLEAN_MODE_SHOOTER_SPEED = 0.1;
  private final double CLEAN_MODE_ACTIVATE_TIME = 3;
  private boolean cleanMode = false;
  private double cleanTimestamp = 0.0;
  private boolean cleanPaused = false;
  private double cleanSpeedBase = 1;

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

    /*
    // Sets HSV value of the led at index 0
    ledBuffer.setHSV(lightIndex, lightHue, 255, 255);
    // Updates data to the buffer
    led.setData(ledBuffer);
    // Continously writes data to the led from the buffer
    led.start();

    lightIndex++;
    lightHue++;

    if (lightIndex > LED_COUNT) lightIndex = 0;
    if (lightHue > 180) lightHue = 0;
    */
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

    // Move back
    rightFrontMotor.set(ControlMode.PercentOutput, 1);
    leftFrontMotor.set(ControlMode.PercentOutput, 1);

    Timer.delay(0.5);

    // Stop
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftFrontMotor.set(ControlMode.PercentOutput, 0);

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
    if (controller.getAButtonPressed()) cleanTimestamp = Timer.getFPGATimestamp();
    if (
      controller.getAButton() &&
      Timer.getFPGATimestamp() > cleanTimestamp + CLEAN_MODE_ACTIVATE_TIME
    ) {
      cleanMode = !cleanMode;
      cleanTimestamp = Timer.getFPGATimestamp();
      cleanPaused = false;
      cleanSpeedBase = 1;
    }

    if (cleanMode) {
      if (controller.getBButtonPressed()) {
        cleanPaused = !cleanPaused;
      }

      if (!cleanPaused) {
        rightFrontMotor.set(ControlMode.PercentOutput, CLEAN_MODE_DRIVE_SPEED * cleanSpeedBase);
        leftFrontMotor.set(ControlMode.PercentOutput, CLEAN_MODE_DRIVE_SPEED * cleanSpeedBase);
        shooterBottom.set(ControlMode.PercentOutput, CLEAN_MODE_SHOOTER_SPEED * cleanSpeedBase);
        shooterTop.set(ControlMode.PercentOutput, CLEAN_MODE_SHOOTER_SPEED * cleanSpeedBase);

        double speedChange = (leftTriggerDown ? 1 : 0) + (rightTriggerDown ? -1 : 0);
        cleanSpeedBase += speedChange * 0.02;
      }
    } else {
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
      if (rightTriggerDown && ready) {
        shooterBottom.set(ControlMode.PercentOutput, 1);
      } else {
        shooterBottom.set(ControlMode.PercentOutput, 0);
      }

      // Readying
      shooterTop.set(ControlMode.PercentOutput, controller.getLeftBumper() ? 1 : 0);

      if (controller.getLeftBumperPressed()) {
        ready = false;
        readyDelay = new Delay(READY_DELAY, () -> { ready = true; });
      }

      if (controller.getLeftBumperReleased()) {
        ready = false;
      }

      if (controller.getLeftBumper()) {
        rumble = Math.min(rumble + RUMBLE_CHANGE_SPEED, 1);
      } else {
        // Intake
        shooterTop.set(ControlMode.PercentOutput, -leftTriggerRaw * INTAKE_STRENGTH);
        shooterBottom.set(ControlMode.PercentOutput, -leftTriggerRaw * INTAKE_STRENGTH);

        rumble = Math.max(rumble - RUMBLE_CHANGE_SPEED, 0);
      }
    }
  }

  @Override
  public void teleopExit() {
    rumble = 0;
  }
}
