/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.SPI;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Double m_leftStick;
  private Double m_rightStick;
  private final XboxController m_controller = new XboxController(0);

  private static final int leftDeviceID1 = 2;
  private static final int leftDeviceID2 = 3;
  private static final int rightDeviceID1 = 4;
  private static final int rightDeviceID2 = 1;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;
  // private MotorControllerGroup leftMotors;
  // private MotorControllerGroup rightMotors;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final I2C.Port expansionPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor1 = new ColorSensorV3(i2cPort);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(expansionPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;
  private RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;

  private ShootingMechanism shooter = ShootingMechanism.getInstance();
  private IntakeMechanism intake = IntakeMechanism.getInstance();

  private final double DISTANCE_PER_ROTATION = 1.0d / 8.0d * 6.1d * Math.PI; // inches
  // 42 counts per revolution for the encoders

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  private final Servo swivelServo = new Servo(1);
  private final Servo tiltServo = new Servo(0);
  private double axisCameraY = 1;
  private double axisCameraZ = 1;

  private final Timer m_timer = new Timer();

  AHRS gyro;

  private PIDController turnController;
  double rotateToAngleRate;
  boolean turnControllerEnabled = false;
  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;

  static final double kTargetAngleDegrees = 0.0f;
  float yawF = 0;
  double yawD = 0;

  private final SendableChooser<String> chooser = new SendableChooser<>();

  @Override
  public void robotPeriodic() {
    Color detectedColor1 = m_colorSensor1.getColor();
    double IR1 = m_colorSensor1.getIR();

    String colorString1;
    ColorMatchResult match1 = m_colorMatcher.matchClosestColor(detectedColor1);

    if (match1.color == kBlueTarget) {
      colorString1 = "Blue";
    } else if (match1.color == kGreenTarget) {
      colorString1 = "Green";
    } else if (match1.color == kRedTarget) {
      colorString1 = "Red";
    } else if (match1.color == kYellowTarget) {
      colorString1 = "Yellow";
    } else {
      colorString1 = "Unknown";
    }

    SmartDashboard.putNumber("Red1", detectedColor1.red);
    SmartDashboard.putNumber("Blue1", detectedColor1.blue);
    SmartDashboard.putNumber("Green1", detectedColor1.green);
    SmartDashboard.putNumber("IR1", IR1);
    SmartDashboard.putNumber("Confidence1", match1.confidence);
    SmartDashboard.putString("Detected Color1", colorString1);

    int proximity1 = m_colorSensor1.getProximity();

    SmartDashboard.putNumber("Proximity1", proximity1);

    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

    SmartDashboard.putNumber("Left Encoders", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoders", getRightEncoderDistance());

    SmartDashboard.putBoolean("Magnetic Disturbance", has_interference);

  }

  @Override
  public void robotInit() {
    /**
     * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax
     * object
     * 
     * The CAN ID, which can be configured using the SPARK MAX Client, is passed as
     * the
     * first parameter
     * 
     * The motor type is passed as the second parameter. Motor type can either be:
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
     * 
     * The example below initializes four brushless motors with CAN IDs 1 and 2.
     * Change
     * these parameters to match your setup
     */
    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushless);

    // leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    // rightMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_leftMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();

    m_leftMotor1.setInverted(true);
    m_leftMotor2.setInverted(true);

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    leftEncoder1 = m_leftMotor1.getEncoder();
    leftEncoder2 = m_leftMotor2.getEncoder();
    rightEncoder1 = m_rightMotor1.getEncoder();
    rightEncoder2 = m_rightMotor2.getEncoder();

    m_leftMotor1.setIdleMode(IdleMode.kCoast);
    m_rightMotor1.setIdleMode(IdleMode.kCoast);

    leftEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    leftEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);

    resetEncoders();



    m_leftMotor1.setOpenLoopRampRate(0.5);
    m_leftMotor2.setOpenLoopRampRate(0.5);
    m_rightMotor1.setOpenLoopRampRate(0.5);
    m_rightMotor2.setOpenLoopRampRate(0.5);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    // CameraServer.startAutomaticCapture();
    gyro = new AHRS(SPI.Port.kMXP);
    turnController = new PIDController(kP, kI, kD);
    turnController.enableContinuousInput(-180.0f, 180.0f);
    
    chooser.addOption("Task 1-A", "task 1-A");
    chooser.addOption("Task 1-B", "task 1-B");
    chooser.addOption("Task 2-A", "task 2-A");
    chooser.addOption("Task 1-A", "task 1-A");
    chooser.addOption("Square Task", "square task");
    chooser.addOption("Test Task", "test task");
    SmartDashboard.putData("Auto Task", chooser);

  }

  @Override
  public void disabledInit() {
    m_leftMotor1.setIdleMode(IdleMode.kCoast);
    m_rightMotor1.setIdleMode(IdleMode.kCoast);

    m_controller.setRumble(RumbleType.kLeftRumble, 0);
    m_controller.setRumble(RumbleType.kRightRumble, 0);

    System.out.println(m_timer.get());
  }

  private void resetEncoders() {
    leftEncoder1.setPosition(0.0d);
    leftEncoder2.setPosition(0.0d);
    rightEncoder1.setPosition(0.0d);
    rightEncoder2.setPosition(0.0d);
  }

  public double getLeftEncoderDistance() {
    return Math.abs(((leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2.0d));
  }

  public double getRightEncoderDistance() {
    return Math.abs(((rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2.0d));
  }

  private int state = 0;
  private int time = 0;
  private double turn_to = 0.0;
  private boolean finished_drive = false;
  private double target_angle = 90;
  private float initial_compass_heading = 0.0f;
  private float current_compass_heading = 0.0f;
  private boolean has_interference = false;
  private double total_angle = 0;
  private double current_angle = 0;

  private int task_status = 0;



  private void task_1a() {

    switch (task_status) {
      case 0: {
        intake.indexerAndIntakes();
        //shooter.forceShoot();
        // enable flywheel
        boolean reached_position = drive(45, 0.6);
        if (reached_position) {
          task_status++;
          intake.off();
          resetEncoders();
        }
        break;
      }
      case 1: {
        //intake.indexerAndIntakes();
        //shooter.forceShoot();
        boolean reached_position = drive(25, -0.6);
        if (reached_position) {
          task_status++;
          //intake.off();
          time = 0;
          resetEncoders();
        }
        break;
      }
      case 2: {
        intake.indexerAndIntakes();
        shooter.forceShoot();
        boolean reached_angle = slow_turn(10);
        if (reached_angle) {
          task_status++;
          stop_motors();
        }
        break;
      }
      case 3: {
        //intake.indexerAndIntakes();
        shooter.forceShoot();
        // shooter.forceShoot();
        // intake.indexerAndIntakes();
        //shooter.ballFeederON();
        time += 1;
        if (time >= 30) {
          //shooter.ballFeederOFF();
          intake.off();
          shooter.off();
          task_status++;
        }
        break;
      }
      case 4: {
        boolean reached_angle = turn(86);
        if (reached_angle) {
          task_status+=2; // skip 5
          stop_motors();
        }
        break;
      }
      case 5: {
        boolean reached_angle = slow_turn(86);
        if (reached_angle) {
          task_status++;
          stop_motors();
          resetEncoders();
          
        }
        break;
      }
      case 6: {
        intake.indexerAndIntakes();
        boolean reached_distance = drive(135, 0.6);
        if (reached_distance) {
          task_status++;
          resetEncoders();
          // intake.off();
        }
        break;
      }
      case 7: {
        boolean reached_angle = turn(175);
        if (reached_angle) {
          task_status++;
          stop_motors();
        }

        break;
      }
      case 8: {

        boolean reached_angle = slow_turn(175);
        if (reached_angle) {
          task_status++;
          stop_motors();
          resetEncoders();
        }
        break;
      }
      case 9: {
        // intake.indexerAndIntakes();
        boolean reached_distance = drive(90, 0.6);

        if (reached_distance) {
          task_status++;
        }

        break;
      }
      case 10: {
        boolean reached_angle = turn(110);

        if (reached_angle) {
          task_status+= 2; // skip 11
          intake.off();
          resetEncoders();
        }

        break;
      }
      case 11: {
        boolean reached_angle = slow_turn(110);

        if (reached_angle) {
          task_status++;
          resetEncoders();
        }
        break;
      }
      case 12: {
        //intake.indexerAndIntakes();
        //shooter.forceShoot();
        boolean reached_distance = drive(98, -0.6);

        if (reached_distance) {
          task_status++;
          resetEncoders();
        }
        break;
      }
      case 13: {
        intake.indexerAndIntakes();
        shooter.forceShoot();
        boolean reached_angle = slow_turn(113);

        if (reached_angle) {
          task_status++;
          time = 0;
        }
        break;
      }
      case 14: {
        time++;
        intake.indexerAndIntakes();
        shooter.forceShoot();
        // shooter.forceShoot();
        //intake.indexerAndIntakes();
        //shooter.ballFeederON();
        if (time >= 400) {
          task_status++;
          shooter.off();
          intake.off();
        }
      }
      
      default: {} // leave this empty
    }
  }

  private void square_task() {
    if (state == 0) {

      finished_drive = drive(80, 0.4);

      if (finished_drive) {

        state = 1;
        // current_angle += target_angle;
      }
    } else if (state == 1) {
      // System.out.println(gyro.getAngle() + " -> " + angle);

      boolean reached_angle = turn(target_angle + current_angle);

      if (reached_angle) {
        state = 2;
        System.out.println("State 2");
        return;
      }

      // turn(target_angle + current_angle);
    } else if (state == 2) {

      // System.out.println(gyro.getAngle() + " -> " + angle);

      boolean reached_angle = slow_turn(target_angle + current_angle);

      if (reached_angle) {
        state = 0;
        time = 0;
        total_angle += gyro.getAngle();
        gyro.zeroYaw();
        return;
      }

      has_interference = gyro.isMagneticDisturbance();
      // System.out.println(has_interference);
      current_compass_heading = gyro.getCompassHeading();
      // System.out.println(current_angle + " " + total_angle + " " + target_angle + "
      // " + current_compass_heading);
    }
    // gyro.getAngle() >= -10 &&
  }

  private final double MAX_SPEED = 0.6;
  private int drive_state = 0;
  private double current_distance = 0;


  private void stop_motors() {
    m_leftMotor1.set(0);
    m_leftMotor2.set(0);
    m_rightMotor1.set(0);
    m_rightMotor2.set(0);
  }

  private boolean drive(double distance, double speed) {
    speed = MathUtil.clamp(speed, -MAX_SPEED, MAX_SPEED);
    distance = Math.abs(distance);
    if (drive_state == 0) {
      resetEncoders();
      drive_state = 1;
    }
    if (drive_state == 1) {
      // m_myRobot.arcadeDrive(speed, 0); // second arg can be used to counter robot
      // drifting left/right
      m_myRobot.tankDrive(
          speed, // * 1.05,
          speed);

      current_distance = Math.min(getLeftEncoderDistance(), getRightEncoderDistance());
      if (current_distance >= distance - 1) {
        drive_state = 2;
      }
    }
    if (drive_state == 2) {
      double dir = (speed / Math.abs(speed)) * 2;
      m_myRobot.tankDrive(
          MathUtil.clamp((getLeftEncoderDistance() - distance) * -dir, -0.2, 0.2),
          MathUtil.clamp((getRightEncoderDistance() - distance) * -dir, -0.2, 0.2));
      current_distance = Math.min(getLeftEncoderDistance(), getRightEncoderDistance());
      if (current_distance >= distance - 1 && current_distance <= distance + 1) {
        drive_state = 0;
        return true;
      }
    }
    return false;

  }

  private boolean turn(double angle) {

    double _angle = gyro.getAngle();
    if (_angle < -180.0) {
      while (_angle < -180.0) {
        _angle += 360.0;
      }
    } else if (_angle > 180.0) {
      while (_angle > 180.0) {
        _angle -= 360.0;
      }
    }

    if (angle < -180.0) {
      while (angle < -180.0) {
        angle += 360.0;
      }
    } else if (angle > 180.0) {
      while (angle > 180.0) {
        angle -= 360.0;
      }
    }

    if (_angle <= (-angle) + 7 && _angle >= (-angle) - 7) {

      return true;
    }

    if (!turnControllerEnabled) {
      turnController.setSetpoint(kTargetAngleDegrees);
      rotateToAngleRate = 0; // This value will be updated by the PID Controller
      turnControllerEnabled = true;
    }
    rotateToAngleRate = (turnController.calculate(gyro.getAngle() + angle));

    rotateToAngleRate *= 15;
    rotateToAngleRate *= (rotateToAngleRate * rotateToAngleRate) + 5;
    rotateToAngleRate = (MathUtil.clamp(rotateToAngleRate / 10, -0.35, 0.4));
    m_myRobot.tankDrive(-rotateToAngleRate, rotateToAngleRate);
    return false;
  }

  private boolean slow_turn(double angle) {

    double _angle = gyro.getAngle();
    if (_angle < -180.0) {
      while (_angle < -180.0) {
        _angle += 360.0;
      }
    } else if (_angle > 180.0) {
      while (_angle > 180.0) {
        _angle -= 360.0;
      }
    }

    if (angle < -180.0) {
      while (angle < -180.0) {
        angle += 360.0;
      }
    } else if (angle > 180.0) {
      while (angle > 180.0) {
        angle -= 360.0;
      }
    }

    if (_angle <= (-angle) + 0.35 && _angle >= (-angle) - 0.35) {
      return true;
    }

    if (!turnControllerEnabled) {
      turnController.setSetpoint(kTargetAngleDegrees);
      rotateToAngleRate = 0; // This value will be updated by the PID Controller
      turnControllerEnabled = true;
    }
    rotateToAngleRate = (turnController.calculate(gyro.getAngle() + angle));

    rotateToAngleRate *= 15;
    rotateToAngleRate *= (rotateToAngleRate * rotateToAngleRate) + 5;
    rotateToAngleRate = (MathUtil.clamp(rotateToAngleRate / 15, -0.325, 0.375));
    m_myRobot.tankDrive(-rotateToAngleRate, rotateToAngleRate);
    return false;
  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    gyro.zeroYaw();
    //gyro.calibrate();
    resetEncoders();
    drive_state = 0;
    initial_compass_heading = gyro.getCompassHeading();
    m_leftMotor1.setIdleMode(IdleMode.kBrake);
    m_rightMotor1.setIdleMode(IdleMode.kBrake);
    task_status = 0;
    time = 0;
    shooter.off();
    intake.off();
    task = chooser.getSelected();
  }

  String task = "None";
  @Override
  public void autonomousPeriodic() {

    switch (task) {
      case "task 1-A": {
        task_1a();
        break;
      }
      case "task 1-B": {
        break;
      }
      case "task 2-A": {
        break;
      }
      case "task 2-B": {
        break;
      }
      case "square task": {
        square_task();
        break;
      }
      case "test task": {
        break;
      }
      default: {} // leave empty
    }

    task_1a();
    shooter.update();
    intake.update();
  }

  @Override
  public void teleopInit() {
    gyro.zeroYaw();
    m_leftMotor1.setIdleMode(IdleMode.kBrake);
    m_rightMotor1.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {

    if (m_controller.getLeftBumper()) {
      /*
       * While this button is held down, rotate to target angle. Since a Tank drive
       * system cannot move forward simultaneously while rotating, all joystick input
       * is ignored until this button is released.
       */

      turn(gyro.getAngle());
    } else if (m_controller.getRightBumper()) {
      /*
       * "Zero" the yaw (whatever direction the sensor is pointing now will become the
       * new "Zero" degrees.
       */
      gyro.zeroYaw();
    } else if (m_controller.getStartButton()) {
      /*
       * While this button is held down, the robot is in "drive straight" mode.
       * Whatever direction the robot was heading when "drive straight" mode was
       * entered will be maintained. The average speed of both joysticks is the
       * magnitude of motion.
       */
      if (!turnControllerEnabled) {
        // Acquire current yaw angle, using this as the target angle.
        turnController.setSetpoint(gyro.getYaw());
        rotateToAngleRate = 0; // This value will be updated by the PID Controller
        turnControllerEnabled = true;
      }
      rotateToAngleRate = MathUtil.clamp(turnController.calculate(gyro.getAngle()), -1.0, 1.0);
      double magnitude = (m_controller.getLeftY() + m_controller.getRightY()) / 2;
      double leftStickValue = magnitude + rotateToAngleRate;
      double rightStickValue = magnitude - rotateToAngleRate;
      m_myRobot.tankDrive(leftStickValue, rightStickValue / 2);
    } else {
      /* If the turn controller had been enabled, disable it now. */
      if (turnControllerEnabled) {
        turnControllerEnabled = false;
      }
      /* Standard tank drive, no driver assistance. */

      m_leftStick = m_controller.getRawAxis(1);
      m_rightStick = m_controller.getRawAxis(4);
      m_myRobot.arcadeDrive(-m_leftStick, -m_rightStick);
    }

    /*
     * swivelServo.set((m_rightStick.getY() + 1) / 2);
     * tiltServo.set((m_rightStick.getZ() + 1) / 2);
     */

    if (m_controller.getXButton()) { // button X
      axisCameraY = 0;
      axisCameraZ = 0;
      shooter.forceShoot();
      intake.indexerAndIntakes();

    } else if (m_controller.getAButton()) { // button A
      axisCameraY = 0;
      axisCameraZ = 1;
    } else if (m_controller.getBButton()) { // button B
      axisCameraY = 0.5;
      axisCameraZ = 0.5;
    } else if (m_controller.getYButton()) { // button Y
      axisCameraY = 1;
      axisCameraZ = 0;
    } else {
      swivelServo.set(axisCameraZ);
      tiltServo.set(axisCameraY);

      shooter.off();
      intake.off();

    }
    ;

    // gyro.get
    // m_controller.setRumble(RumbleType.kLeftRumble,
    // Math.abs(leftEncoder1.getVelocity()/1000));
    m_controller.setRumble(RumbleType.kRightRumble, Math.abs(rightEncoder1.getVelocity() / 1000));
    // m_controller.setRumble(RumbleType.kRightRumble,
    // Math.abs(m_rightMotor1.get()/1000));
    // m_controller.setRumble(RumbleType.kRightRumble,
    // Math.abs(m_leftMotor1.get()/1000));

    intake.update();
    shooter.update();

  }
}
