/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Double m_leftStick;
  private Double m_rightStick;
  private final XboxController m_controller = new XboxController(0);

  private static final int leftDeviceID1 = 1; 
  private static final int leftDeviceID2 = 2; 
  private static final int rightDeviceID1 = 3;
  private static final int rightDeviceID2 = 4;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;



  private double rampRate = .3;

  
  @Override
  public void robotPeriodic() {
      ColorSensor.ColorSensor_robotPeriodic();
      CameraServer.startAutomaticCapture();

  }
  
  @Override
  public void robotInit() {
    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushless);

    m_leftMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();

    m_leftMotor1.setInverted(true);
    m_leftMotor2.setInverted(true);
    
    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

    m_leftMotor1.setOpenLoopRampRate(rampRate);
    m_leftMotor2.setOpenLoopRampRate(rampRate);
    m_rightMotor1.setOpenLoopRampRate(rampRate);
    m_rightMotor2.setOpenLoopRampRate(rampRate);

    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    ColorSensor.ColorSensor_robotInit();


  }

  @Override
  public void teleopPeriodic() {
    m_leftStick = m_controller.getLeftY();
    m_rightStick = m_controller.getRightX();
    m_myRobot.arcadeDrive(m_leftStick, -(m_rightStick / 2));
    
    AxisCamera.cameraStuff();
  }
}
