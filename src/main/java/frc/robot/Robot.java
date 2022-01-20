/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private static final int leftDeviceID1 = 1; 
  private static final int leftDeviceID2 = 2; 
  private static final int rightDeviceID1 = 3;
  private static final int rightDeviceID2 = 4;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;
  //private MotorControllerGroup leftMotors;
  //private MotorControllerGroup rightMotors;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final I2C.Port expansionPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor1 = new ColorSensorV3(i2cPort);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(expansionPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget =  new Color(0.561, 0.232, 0.114);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  
  @Override
  public void robotPeriodic() {
      Color detectedColor1 = m_colorSensor1.getColor();
      double IR1 = m_colorSensor1.getIR();

      String colorString1;
      ColorMatchResult match1 = m_colorMatcher.matchClosestColor(detectedColor1);

      if (match1.color == kBlueTarget) {colorString1 = "Blue";}
      else if (match1.color == kGreenTarget) {colorString1 = "Green";}
      else if (match1.color == kRedTarget) {colorString1 = "Red";}
      else if (match1.color == kYellowTarget) {colorString1 = "Yellow";}
      else {colorString1 = "Unknown";}

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

      if (match.color == kBlueTarget) {colorString = "Blue";}
      else if (match.color == kGreenTarget) {colorString = "Green";}
      else if (match.color == kRedTarget) {colorString = "Red";}
      else if (match.color == kYellowTarget) {colorString = "Yellow";}
      else {colorString = "Unknown";}

      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("IR", IR);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);

      int proximity = m_colorSensor.getProximity();

      SmartDashboard.putNumber("Proximity", proximity);

  }
  
  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushless);

    //leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    //rightMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);


    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}
