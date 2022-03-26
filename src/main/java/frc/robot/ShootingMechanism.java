package frc.robot;

import java.util.Map;
import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;  // why do none of these libraries exist anymore???
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ControlType;

public class ShootingMechanism {
        
    private static ShootingMechanism instance;
    private LimelightVisionTracking limelight = LimelightVisionTracking.getInstance();
    private TalonSRX ballFeeder = new TalonSRX(2);


    private CANSparkMax shooter = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax shooter_b = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    //These motor controllers have enocders in them
    private RelativeEncoder shooterEncoder = shooter.getEncoder();
    private SparkMaxPIDController pidController;
    private Timer shooterTimer = new Timer();

    private final double SHOOTER_KP = 0.0004;
    private final double SHOOTER_KI = 0.000000575;
    private final double SHOOTER_KD = 1.0;
    private final double SHOOTER_I_ZONE = 2000;
    private final double FEED_FORWARD = 0.0;
    
    private final double FLY_WHEEL_SPEED_THRESH = 30;
    private final double BALL_FEEDER_SPEED = -0.9;
    private final double FEEDER_IDLE_POWER = 0.1;


    private ShootingMechanism() {

        ballFeeder.setNeutralMode(NeutralMode.Brake);
        
        
        ballFeeder.configVoltageCompSaturation(11);
        ballFeeder.enableVoltageCompensation(true);

        shooter_b.follow(shooter, true);
        shooter.setIdleMode(IdleMode.kCoast);
        shooter_b.setIdleMode(IdleMode.kCoast);

        pidController = shooter.getPIDController();

        shooter.setSmartCurrentLimit(45);
        shooter_b.setSmartCurrentLimit(45);

        shooterTimer.start();

        pidController.setP(SHOOTER_KP);
        pidController.setI(SHOOTER_KI);
        pidController.setD(SHOOTER_KD);
        pidController.setIZone(SHOOTER_I_ZONE);
        pidController.setFF(FEED_FORWARD);
        pidController.setOutputRange(-1, 0);

        SmartDashboard.putNumber("Shooter P", SHOOTER_KP);
        SmartDashboard.putNumber("Shooter I", SHOOTER_KI);
        SmartDashboard.putNumber("Shooter D", SHOOTER_KD);
        SmartDashboard.putNumber("Shooter FF", FEED_FORWARD);
        SmartDashboard.putNumber("Shooter IZone", SHOOTER_I_ZONE);


        SmartDashboard.putNumber("set shooter dis", 0);
        SmartDashboard.putNumber("set shooter RPM", 0);
        SmartDashboard.putNumber("set shooter angle", 0);
        SmartDashboard.putBoolean("set Shooter Table val", false);
    }

    public static ShootingMechanism getInstance() {
        if (instance == null) {
            instance = new ShootingMechanism();
        }
            return instance;
    }

    enum ShooterMechanismModes {
        shoot,
        forceShoot,
        slowShoot,
        feederReverse,
        off; // upon release in teleop
    }

    private ShooterMechanismModes curMode = ShooterMechanismModes.off;
    
    final Map<ShooterMechanismModes, Runnable> shootingModes = Map.ofEntries(
                entry(ShooterMechanismModes.off, this::executeOff),
                entry(ShooterMechanismModes.shoot, this::executeShoot),
                entry(ShooterMechanismModes.forceShoot, this::executeForceShoot),
                entry(ShooterMechanismModes.slowShoot, this::executeSlowShoot),
                entry(ShooterMechanismModes.feederReverse, this::executeFeedeReverse)
    );

    public void update() {
        shootingModes.get(curMode).run();
        SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter pow", shooter.get());
        SmartDashboard.putNumber("shooter current", shooter.getOutputCurrent());

        if (SmartDashboard.getBoolean("set Shooter Table val", false)) {
            ShooterCalculator.setValues(
            (int)SmartDashboard.getNumber("set shooter dis", 0),
            SmartDashboard.getNumber("set shooter RPM", 0),
            SmartDashboard.getNumber("set shooter angle", 0)
            );

            SmartDashboard.putBoolean("set Shooter Table val", false);
        }
        
        SmartDashboard.putNumber("shooterTimer", shooterTimer.get());

        pidController.setP(SmartDashboard.getNumber("Shooter P", SHOOTER_KP));
        pidController.setI(SmartDashboard.getNumber("Shooter I", SHOOTER_KI));
        pidController.setD(SmartDashboard.getNumber("Shooter D", SHOOTER_KD));
        pidController.setIZone(SmartDashboard.getNumber("Shooter IZone", SHOOTER_I_ZONE));
        pidController.setFF(SmartDashboard.getNumber("Shooter FF", FEED_FORWARD));
        
    }

    public void off() {
        this.curMode = ShooterMechanismModes.off;
    }

    public void executeOff() {
        shooterOFF();
        ballFeederOFF();
    }

    public void shoot() {
        this.curMode = ShooterMechanismModes.shoot;
    }

    public void forceShoot() {
        this.curMode = ShooterMechanismModes.forceShoot;
    }

    public void slowShoot() {
        this.curMode = ShooterMechanismModes.slowShoot;
    }

    public void reverseFeeder() {
        this.curMode = ShooterMechanismModes.feederReverse;
    }

    private void executeShoot() {
        shooterON();
        double distance = limelight.getDistance();
        if ((shooterEncoder.getVelocity() < (-ShooterCalculator.calculateRPM(distance) + FLY_WHEEL_SPEED_THRESH))
                        && limelight.targetFound()
                        && inRange(limelight.getHorizontalAngle(), -2, 2)
                        && inRange(distance - ShooterCalculator.roundDis(distance), -5, 5)) {
            ballFeederON();
        } else {
            ballFeederOFF();
        }
    }

    private void executeForceShoot() {
        shooterON();
        ballFeederON();
    }

    private void executeSlowShoot() {
        shooterSlow();
        ballFeederON();
    }

    private void executeFeedeReverse() {
        feederReverse();
        shooterOFF();
    }

    public void ballFeederON() {
        ballFeeder.set(ControlMode.PercentOutput, BALL_FEEDER_SPEED);
    }

    public void ballFeederOFF() {
        ballFeeder.set(ControlMode.PercentOutput, FEEDER_IDLE_POWER);
    }

    private void feederReverse() {
        ballFeeder.set(ControlMode.PercentOutput, -BALL_FEEDER_SPEED);

    }

    public void shooterON() {
        double flyWheelSpeed = -ShooterCalculator.calculateRPM(limelight.getDistance());
        pidController.setReference(flyWheelSpeed, ControlType.kVelocity);
        // shooter.set(-0.4);
    }

    public void flywheelON() {
        double flyWheelSpeed = -2300.0;
        pidController.setReference(flyWheelSpeed, ControlType.kVelocity);
    }

    public void flywheelOFF() {
        pidController.setReference(0, ControlType.kVelocity);
    }

    private void shooterSlow() {
        shooter.set(-0.15);
    }

    public void shooterOFF() {
        shooter.set(0);
    }

    public double shooterVelocity() {
      return shooterEncoder.getVelocity();
    }
    
    public double shooterSetpoint() {
        return -ShooterCalculator.calculateRPM(limelight.getDistance());
    }

    /**
     * 
     * @param input input value
     * @param low lower bound
     * @param high upper bound
     * @return if input is between low and high (inclusive)
     */
    private boolean inRange(double input, double low, double high) {
        if ((input >= low) && (input <= high)) {
            return true;
        }
        else {
            return false;
        }
    }

}
