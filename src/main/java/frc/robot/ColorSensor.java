package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;



public class ColorSensor{
  private static final I2C.Port i2cPort = I2C.Port.kOnboard;
  private static final I2C.Port expansionPort = I2C.Port.kMXP;
  
  private static final ColorSensorV3 m_colorSensor1 = new ColorSensorV3(i2cPort);
  private static final ColorSensorV3 m_colorSensor = new ColorSensorV3(expansionPort);
  private static final ColorMatch m_colorMatcher = new ColorMatch();

  private static final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private static final Color kRedTarget =  new Color(0.561, 0.232, 0.114);
  private static final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private static final Color kYellowTarget = new Color(0.361, 0.524, 0.113);




public static void ColorSensor_robotPeriodic(){
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

    };
    public static void ColorSensor_robotInit(){
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }
};