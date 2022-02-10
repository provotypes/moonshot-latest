package frc.robot;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;



public class AxisCamera {
  public static final XboxController m_controller = new XboxController(0);

  public static final Servo swivelServo = new Servo(1);
  public static final Servo tiltServo = new Servo(0);
  public static double axisCameraY = 1;
  public static double axisCameraZ = 1;

  
  public static void cameraStuff() {
  
      if (m_controller.getPOV(0) == 1) { // up button
        axisCameraY = 0;
        axisCameraZ = 0;
      }
      else if (m_controller.getPOV(90) == 1) { // right button 
        axisCameraY = 0;
        axisCameraZ = 1;
      }
      else if (m_controller.getPOV(180) == 1) { // down button
        axisCameraY = 0.5;
        axisCameraZ = 0.5;
      }
      else if (m_controller.getPOV(270) == 1) { // left button
        axisCameraY = 1;
        axisCameraZ = 0;
      }
      else{
        swivelServo.set(axisCameraZ);
        tiltServo.set(axisCameraY);
      };
  }
};
