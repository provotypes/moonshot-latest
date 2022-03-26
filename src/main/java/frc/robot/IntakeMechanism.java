package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;

public class IntakeMechanism {

    private static IntakeMechanism instance;
    private static TalonSRX outerIntakeWheels = new TalonSRX(4);
	private static TalonSRX innerIntakeWheels = new TalonSRX(1);
    private static TalonSRX indexer = new TalonSRX(3);
    private static TalonSRX greenWheels = new TalonSRX(7);
   
    private final double INDEXER_PERCENT = 0.4;
    private final double INDEXER_REVERSE = -0.9;
    private final double OUTER_INTAKE_PERCENT = 0.95;
    private final double INNER_INTAKE_PERCENT = -0.3;
    private final double OUTER_INTAKE_REVERSE = -0.6;
    private final double INNER_INTAKE_REVERSE = 0.9;
    private final double OUTER_INTAKE_REVERSE_LOW = 0.0;
    private Timer time = new Timer();

    private IntakeMechanism() {
        time.start();

        // indexer.enableCurrentLimit(true);
        // indexer.configContinuousCurrentLimit(10);
    }

    public static IntakeMechanism getInstance() {
        if(instance == null) { 
            instance = new IntakeMechanism();
         }
        return instance;
    }
    
    enum IntakeMechanismModes {
        indexerAndIntakes,
        indexer,
        indexerSin,
        reverseEverything,
        indexerReverse,
        intakeIdle,
        off
    }

    final Map<IntakeMechanismModes, Runnable> intakeModes = Map.ofEntries(
			entry(IntakeMechanismModes.off, this::executeOff),
			entry(IntakeMechanismModes.intakeIdle, this::executeIntakeIdle),
			entry(IntakeMechanismModes.indexerAndIntakes, this::executeIndexerAndIntakes),
            entry(IntakeMechanismModes.indexer, this::executeIndexer),
            entry(IntakeMechanismModes.indexerSin, this::executeIndexerSin),
            entry(IntakeMechanismModes.reverseEverything, this::executeReverse),
            entry(IntakeMechanismModes.indexerReverse, this::executeIndexReverse)
	    );

    private IntakeMechanismModes mode = IntakeMechanismModes.off;

    public void update() {
        intakeModes.get(mode).run();
    }

    public void off() {
        this.mode = IntakeMechanismModes.off;
    }

    public void intakeIdle() {
        this.mode = IntakeMechanismModes.intakeIdle;
    }

    public void indexerAndIntakes() {
        this.mode = IntakeMechanismModes.indexerAndIntakes;
    }

    public void indexer() {
        this.mode = IntakeMechanismModes.indexer;
    }

    public void indexerSwitch() {
        this.mode = IntakeMechanismModes.indexerSin;
    }

    public void reverseEverything() {
        this.mode = IntakeMechanismModes.reverseEverything;
    }

    public void reverseIndexer() {
        this.mode = IntakeMechanismModes.indexerReverse;
    }
  
    private void executeOff() {
        outerIntakeWheelsOFF();
        innerIntakeWheelsOFF();
        indexerOFF();
        greenWheelsOFF();
    }

    private void executeIntakeIdle() {
        outerIntakeWheelsReverseLow();
        innerIntakeWheelsOFF();
        indexerOFF();
        greenWheelsOFF();
    }

    private void executeIndexerAndIntakes() {
        outerIntakeWheelsON();
        innerIntakeWheelsON();
        indexerON();
        greenWheelsON();
    }

    private void executeIndexer() {
        innerIntakeWheelsON();
        outerIntakeWheelsReverseLow();
        indexerON();
        greenWheelsON();
    }
    private void executeIndexerSin() {
        innerIntakeWheelsON();
        outerIntakeWheelsON();
        indexerSIN();
        greenWheelsON();
    }

    private void executeReverse() {
        innerIntakeWheelsReverse();
        outerIntakeWheelsReverse();
        indexerReverse();
        greenWheelsON();
    }

    private void executeIndexReverse() {
        innerIntakeWheelsOFF();
        outerIntakeWheelsOFF();
        indexerReverse();
        greenWheelsOFF();
    }

    private void outerIntakeWheelsON() {
        outerIntakeWheels.set(ControlMode.PercentOutput, OUTER_INTAKE_PERCENT);
    }

    private void outerIntakeWheelsOFF() {
        outerIntakeWheels.set(ControlMode.PercentOutput, 0);
    }

    // This will be constantly running when intake is off
    private void outerIntakeWheelsReverseLow() {
        outerIntakeWheels.set(ControlMode.PercentOutput, OUTER_INTAKE_REVERSE_LOW);
    }

    // This along with inner intake wheels reverse will have a dedicated button in case we jam
    private void outerIntakeWheelsReverse() {
        outerIntakeWheels.set(ControlMode.PercentOutput, OUTER_INTAKE_REVERSE);
    }

    private void innerIntakeWheelsON() {
        innerIntakeWheels.set(ControlMode.PercentOutput, INNER_INTAKE_PERCENT);
    }

    private void innerIntakeWheelsOFF() {
        innerIntakeWheels.set(ControlMode.PercentOutput, 0);
    }

    private void innerIntakeWheelsReverse() {
        innerIntakeWheels.set(ControlMode.PercentOutput, INNER_INTAKE_REVERSE);
    }

    private void greenWheelsON() {
        greenWheels.set(ControlMode.PercentOutput, -0.3);
    }

    private void greenWheelsOFF() {
        greenWheels.set(ControlMode.PercentOutput, 0);
    }

    private void indexerON() {
        indexer.set(ControlMode.PercentOutput, INDEXER_PERCENT);
    }

    private void indexerSIN() {
        indexer.set(ControlMode.PercentOutput, (Math.sin(3.0 * time.get()) * 0.5d) + 0.3);
    }

    private void indexerOFF() {
        indexer.set(ControlMode.PercentOutput, 0);
    }

    private void indexerReverse() {
        indexer.set(ControlMode.PercentOutput, INDEXER_REVERSE);
    }
}
