package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ProtoMotors {

  private static final XboxController m_controller = new XboxController(0);


  private static final int protoID5 = 5;
  private static final int protoID6 = 6;
  private static final int protoID7 = 7;
  private static final int protoID8 = 8;
  private static CANSparkMax m_protoMotor5;
  private static CANSparkMax m_protoMotor6;
  private static CANSparkMax m_protoMotor7;
  private static CANSparkMax m_protoMotor8;

  private static final DigitalInput toplimitSwitch = new DigitalInput(2);


  public static void setMotorSpeed(double speed) {
    if(m_controller.getBButton()) {
        if (speed > 0) {
            if (toplimitSwitch.get()) {
                // We are going up and top limit is tripped so stop
                m_protoMotor6.set(0);
            } else {
                // We are going up but top limit is not tripped so go at commanded speed
                m_protoMotor6.set(speed);
            }
        }
      }
      else {
        m_protoMotor6.set(0);
      }
}


  public static void Proto_RobotPeriodic() {
    setMotorSpeed(0.25);
  }
  
  public static void Proto_RobotInit() {
    m_protoMotor5 = new CANSparkMax(protoID5, MotorType.kBrushed);
    m_protoMotor6 = new CANSparkMax(protoID6, MotorType.kBrushed);
    m_protoMotor7 = new CANSparkMax(protoID7, MotorType.kBrushed);
    m_protoMotor8 = new CANSparkMax(protoID8, MotorType.kBrushed);
    
    m_protoMotor5.setOpenLoopRampRate(1);
    m_protoMotor6.setOpenLoopRampRate(1);
    m_protoMotor7.setOpenLoopRampRate(1);
    m_protoMotor8.setOpenLoopRampRate(1);

  }

  public static void Proto_TeleopPeriodic() {
      if (m_controller.getXButton()){
        m_protoMotor5.set(.5);
      }
      else{
        m_protoMotor5.set(0);
      };

      if (m_controller.getYButton())
      {
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

    
  }
    

}
