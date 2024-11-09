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

  // Magic constants
  private final double SPEED = 1;
  private final double TURN_SPEED = 0.4;
  private final double AUTO_WAIT_DELAY = 10;
  private final double RUMBLE_CHANGE_SPEED = 0.02;
  private final double INTAKE_STRENGTH = 0.5;

  private boolean readying = false;

  // Trigger input
  private final double TRIGGER_DEADZONE = 0.5;
  private boolean rightTriggerDown = false;
  private boolean leftTriggerDown = false;

  // LED strip
  private int lightIndex = 0;
  private int lightHue = 0;

  private double rumble = 0;
  private double rumbleImportant = 0;

  private List<Delay> delays = new ArrayList<>();

  public class Delay {
    private double startTime;
    private double delayTime;
    Callback callback;

    public interface Callback {
      public void callback();
    }

    public Delay(double _startTime, double _delayTime, Callback _callback) {
      startTime = _startTime;
      delayTime = _delayTime;
      callback = _callback;
    }

    public void update() {
      if (Timer.getFPGATimestamp() >= startTime + delayTime) {

      }
    }
  }

  // Called when the robot starts up
  @Override
  public void robotInit() {
    hapticTap(3);

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

    for (Delay delay : delays) delay.update();
  }

  // Called when autonomous is enabled
  @Override
  public void autonomousInit() {
    hapticTap(2);

    // Readying
    shooterTop.set(ControlMode.PercentOutput, 1.0);

    Timer.delay(3);

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
    hapticTap(2);

    lightHue = 150;
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
    shooterBottom.set(ControlMode.PercentOutput, rightTriggerDown ? 1 : 0);

    // Readying
    readying = controller.getLeftBumper();
    shooterTop.set(ControlMode.PercentOutput, readying ? 1 : 0);

    if (!readying) {
      // Intake
      shooterTop.set(ControlMode.PercentOutput, -leftTriggerRaw * INTAKE_STRENGTH);
      shooterBottom.set(ControlMode.PercentOutput, -leftTriggerRaw * INTAKE_STRENGTH);

      rumble = Math.max(rumble - RUMBLE_CHANGE_SPEED, 0);
    } else {
      rumble = Math.min(rumble + RUMBLE_CHANGE_SPEED, 1);
    }
  }

  public void hapticTap(int count) {
    for (int i = 0; i < count; i++) {
      Delay rumbleDelay = new Delay(
        Timer.getFPGATimestamp(),
        0.5,
        () -> { rumble = 1; }
      );

      delays.add(rumbleDelay);
    }
  }
}
