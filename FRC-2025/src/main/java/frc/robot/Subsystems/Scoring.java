package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoring extends SubsystemBase {

    public static SparkMax wrist;
    public static SparkMax elevator;
    public static SparkMax roller;
    // public static DigitalInput sensor;
    public static RelativeEncoder wristEncoder;
    public static RelativeEncoder elevatorEncoder;
    public static boolean manualControl;
    public static boolean isResetting = true;

    private static Scoring m_Instance = null;


    public Scoring(int angleID, int elevatorID, int rollerID, int sensorID) {
        // Motor Declarations
        wrist = new SparkMax(angleID, MotorType.kBrushless);
        elevator = new SparkMax(elevatorID, MotorType.kBrushless);
        roller = new SparkMax(rollerID, MotorType.kBrushless);
        // sensor = new DigitalInput(sensorID);
        // Encoder Declarations
        wristEncoder = wrist.getEncoder();
        elevatorEncoder = elevator.getEncoder();
        // Motor Configurations
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        SparkMaxConfig wristConfig = new SparkMaxConfig();

       wristConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.idleMode(IdleMode.kBrake);
       wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //limit switch
       // sensor = new DigitalInput(3);
        recalibratePosition();
    }

    public void scoringPeriodic(){
        // if(isResetting){
        //     if(isScoringMecClear()){
        //         elevatorEncoder.setPosition(0.0);
        //         elevator.set(0);
        //         isResetting = false;
        //     }
        // }else{
            elevator.set(Constants.PIDs.elevatorPID.calculate(elevatorEncoder.getPosition()));
            wrist.set(Constants.PIDs.wristPID.calculate(wristEncoder.getPosition()));
       // }
    }

    public void recalibratePosition(){
        elevatorEncoder.setPosition(0);
    }

    // Manual control lets the elevator be controlled from the driver2 (operator) left/right stick
    public void toggleManualControl() {
        manualControl = !manualControl;
    }

    public boolean isManualControl() {
        return manualControl;
    }

    public void manualControlElevator(double speed) {
        Constants.PIDs.elevatorPID.setSetpoint(elevatorEncoder.getPosition() - speed * 25);
    }

    public void manualControlWrist(double speed) {
        if (speed != 0) Constants.PIDs.wristPID.setSetpoint(wristEncoder.getPosition() + speed * 25);
    }

    public double getElevatorEncoder() {
        return elevatorEncoder.getPosition();
    }

    public double getAngleEncoder() {
         return wristEncoder.getPosition();
    }

    // Turns on Roller on scoring mecanism
    public void runRollerIn(double speed) {
        roller.set(-speed);
    }

    public void runRollerOut(double speed) {
        roller.set(speed);
    }

    public void stopRoller() {
        roller.set(0);
    }

    public void elevatorRun(int level) {
        if (level == 0) {
            Constants.PIDs.elevatorPID.setSetpoint(Constants.Scoring.levelElevator0);
        } else if (level == 1) {
            Constants.PIDs.elevatorPID.setSetpoint(Constants.Scoring.levelElevator1);
        } else if (level == 2) {
            Constants.PIDs.elevatorPID.setSetpoint(Constants.Scoring.levelElevator2);
        } else if (level == 3) {
            Constants.PIDs.elevatorPID.setSetpoint(Constants.Scoring.levelElevator3);
        } else if (level == 4) {
            Constants.PIDs.elevatorPID.setSetpoint(Constants.Scoring.levelElevator4);
        } else {
            System.out.println("elevatorRun level incorrect, level value = " + level);
        }
    }

    public void wristRun(int level) {
        if (level == 0) {
            Constants.PIDs.wristPID.setSetpoint(Constants.Scoring.levelWrist0);
        } else if (level == 1) {
            Constants.PIDs.wristPID.setSetpoint(Constants.Scoring.levelWrist1);
        } else if (level == 2) {
            Constants.PIDs.wristPID.setSetpoint(Constants.Scoring.levelWrist2);
        } else if (level == 3) {
            Constants.PIDs.wristPID.setSetpoint(Constants.Scoring.levelWrist3);
        } else if (level == 4) {
            Constants.PIDs.wristPID.setSetpoint(Constants.Scoring.levelWrist4);
        } else {
            System.err.println("wristRun level incorrect, level value = " + level);
        }
    }

    public void clearStickyFaults() {
        wrist.clearFaults();
        elevator.clearFaults();
        roller.clearFaults();
    }


    // public void elevatorReset() {
    //     goToLevel(0);
    // }

    // Set elevator and anlge to levels with encoder ticks

    // private void goToLevel(int l) {
       // Constants.PIDs.wristPID.setSetpoint(Constants.Scoring.elevatorAngles[l]);
    //     Constants.PIDs.elevatorPID.setSetpoint(Constants.Scoring.elevatorLevels[l]);
    // }

    // public void elevatorUp() {
    //     if (elevatorLevel > 3) {
    //         elevatorLevel = 3;
    //     }
    //     goToLevel(elevatorLevel+1);
        
    // }

    // public void elevatorDown() {
    //     if (elevatorLevel < 1) {
    //         elevatorLevel = 1;
    //     }
    //     goToLevel(elevatorLevel-1);
        
    // }

    // public void setLevel(int l) {
    //     elevatorLevel = Math.max(Math.min(l,4),0);
    //     if (elevatorLevel < 0) {
    //         elevatorLevel = 0;
    //     }
    //     if (elevatorLevel > 4) {
    //         elevatorLevel = 4;
    //     }
    //     goToLevel(l);
    // }

    public static Scoring getInstance() {
        if (m_Instance == null)
            m_Instance = new Scoring(Constants.Scoring.wristID, Constants.Scoring.elevatorID,
                    Constants.Scoring.rollerID, Constants.Scoring.sensorID);
        return m_Instance;
    }
    

}
