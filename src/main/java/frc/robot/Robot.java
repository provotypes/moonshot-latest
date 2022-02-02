/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Double m_leftStick;
  private Double m_rightStick;
  private final XboxController m_controller = new XboxController(0);

  private static final int leftDeviceID1 = 1; 
  private static final int leftDeviceID2 = 2; 
  private static final int rightDeviceID1 = 3;
  private static final int rightDeviceID2 = 4;
  /***************************************************** */
  private static final int protoID5 = 5;
  private static final int protoID6 = 6;
  private static final int protoID7 = 7;
  private static final int protoID8 = 8;
  /***************************************************** */
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;
  /***************************************************** */
  private CANSparkMax m_protoMotor5;
  private CANSparkMax m_protoMotor6;
  private CANSparkMax m_protoMotor7;
  private CANSparkMax m_protoMotor8;
  /***************************************************** */
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

  private final Servo swivelServo = new Servo(1);
  private final Servo tiltServo = new Servo(0);
  private double axisCameraY = 1;
  private double axisCameraZ = 1;

  
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

    /***************************************************** */
    m_protoMotor5 = new CANSparkMax(protoID5, MotorType.kBrushed);
    m_protoMotor6 = new CANSparkMax(protoID6, MotorType.kBrushed);
    m_protoMotor7 = new CANSparkMax(protoID7, MotorType.kBrushed);
    m_protoMotor8 = new CANSparkMax(protoID8, MotorType.kBrushed);
    /***************************************************** */
    
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

    m_protoMotor5.setOpenLoopRampRate(1);
    m_protoMotor6.setOpenLoopRampRate(1);
    m_protoMotor7.setOpenLoopRampRate(1);
    m_protoMotor8.setOpenLoopRampRate(1);


    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);


    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void teleopPeriodic() {
    m_leftStick = m_controller.getRawAxis(1);
    m_rightStick = m_controller.getRawAxis(4);
    m_myRobot.arcadeDrive((m_rightStick) / 3, -m_leftStick);

/*      swivelServo.set((m_rightStick.getY() + 1) / 2);
      tiltServo.set((m_rightStick.getZ() + 1) / 2);
*/

      if (m_controller.getPOV() == 0) { // up button
        axisCameraY = 0.5;
        axisCameraZ = 0.5;
      }
      else if (m_controller.getPOV() == 180) { // down button
        axisCameraY = 0.5;
        axisCameraZ = 0.5;
      }
      else if (m_controller.getPOV() == 270) { // left button
        axisCameraY = 0;
        axisCameraZ = 1;
      }
      else if (m_controller.getPOV() == 90) { // right button
        axisCameraY = 1;
        axisCameraZ = 0;
      }
      else{
        swivelServo.set(axisCameraZ);
        tiltServo.set(axisCameraY);

      };


      /***************************************************** */
      if (m_controller.getXButton()){
        m_protoMotor5.set(.5);
      }
      else{
        m_protoMotor5.set(0);
      };
      if (m_controller.getBButton()){
        m_protoMotor6.set(.5);
      }
      else{
        m_protoMotor6.set(0);
      };
      if (m_controller.getYButton()){
        m_protoMotor7.set(.5);
      }
      else{
        m_protoMotor7.set(0);
      };
      if (m_controller.getAButton()){
        m_protoMotor8.set(.5);
      }
      else{
        m_protoMotor8.set(0);
      };

      /***************************************************** */

    

  }
}
