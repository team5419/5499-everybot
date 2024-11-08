package frc.robot;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
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
  private AddressableLED led = new AddressableLED(0);
  private AddressableLEDBuffer led_buffer = new AddressableLEDBuffer(LED_COUNT);

  // Magic constants
  private final double SPEED = 1;
  private final double TURN_SPEED = 0.4;
  private final double AUTO_WAIT_DELAY = 11;

  private boolean readying = false;

  // Trigger input
  private final double TRIGGER_DEADZONE = 0.5;
  private boolean rightTriggerDown = false;
  private boolean leftTriggerDown = false;
  
  private int lightIndex = 0;

  // Called when the robot starts up
  @Override
  public void robotInit() {
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

    // Sets RBG value of the led at index 0
    led_buffer.setRGB(lightIndex, 255, 0, 255);
    // Updates data to the buffer
    led.setData(led_buffer);
    // Continously writes data to the lecd from the buffer
    led.start();

    lightIndex++;
    if (lightIndex > LED_COUNT) {
      
    }
  }

  // Called when autonomous is enabled
  @Override
  public void autonomousInit() {
    // Readying
    shooterTop.set(ControlMode.PercentOutput, 1.0);
    
    Timer.delay(2);
    
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
      shooterTop.set(ControlMode.PercentOutput, -leftTriggerRaw);
      shooterBottom.set(ControlMode.PercentOutput, -leftTriggerRaw);
    }     
  }
}