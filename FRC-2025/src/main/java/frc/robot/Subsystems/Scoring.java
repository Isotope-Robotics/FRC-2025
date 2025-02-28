package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoring extends SubsystemBase {

    public static SparkMax angle;
    public static SparkMax elevator;
    public static SparkMax roller1;
    public static SparkMax roller2;
    public static DigitalInput sensor;
    public static RelativeEncoder angleEncoder;
    public static RelativeEncoder elevatorEncoder;
   // public static angleConfig;

    private static Scoring m_Instance = null;

    private int elevatorLevel = 0;

    public Scoring(int angleID, int elevatorID, int roller1ID, int roller2ID, int sensorID) {
        // Moter Declarations
        angle = new SparkMax(angleID, MotorType.kBrushless);
        elevator = new SparkMax(elevatorID, MotorType.kBrushless);
        roller1 = new SparkMax(roller1ID, MotorType.kBrushless);
        roller2 = new SparkMax(roller2ID, MotorType.kBrushless);
        //sensor = new DigitalInput(sensorID);
        // Encoder Declarations
        angleEncoder = angle.getEncoder();
        elevatorEncoder = elevator.getEncoder();
        // Moter Configurations
        //SparkMaxConfig angleConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
       // angleConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.idleMode(IdleMode.kBrake);
       // angle.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void scoringPeriodic(){
        angle.set(Constants.PIDs.wristPID.calculate(angleEncoder.getPosition()));
        elevator.set(Constants.PIDs.elevatorPID.calculate(angleEncoder.getPosition()));
    }

    // Checks if limit switch is clear
    public boolean isScoringMecClear() {
        return sensor.get();
    }

    // Turns on Roller on scoring mecanism
    public void runRollerIn() {
        roller1.set(0.8);
        roller2.set(0.8);
    }

    public void runRollerOut() {
        roller1.set(-0.8);
        roller2.set(-0.8);
    }

    public void stopRoller() {
        roller1.set(0);
        roller2.set(0);
    }


    public void elevatorReset() {
        goToLevel(0);
    }

    // Set elevator and anlge to levels with encoder ticks

    private void goToLevel(int l) {
        Constants.PIDs.wristPID.setSetpoint(Constants.Scoring.elevatorAngles[l]);
        Constants.PIDs.elevatorPID.setSetpoint(Constants.Scoring.elevatorLevels[l]);
    }

    public void elevatorUp() {
        if (elevatorLevel > 3) {
            elevatorLevel = 3;
        }
        goToLevel(elevatorLevel+1);
        
    }

    public void elevatorDown() {
        if (elevatorLevel < 1) {
            elevatorLevel = 1;
        }
        goToLevel(elevatorLevel-1);
        
    }

    public void setLevel(int l) {
        elevatorLevel = Math.max(Math.min(l,4),0);
        if (elevatorLevel < 0) {
            elevatorLevel = 0;
        }
        if (elevatorLevel > 4) {
            elevatorLevel = 4;
        }
        goToLevel(l);
    }

    public static Scoring getInstance() {
        if (m_Instance == null)
            m_Instance = new Scoring(Constants.Scoring.angleID, Constants.Scoring.elevatorID,
                    Constants.Scoring.roller1ID, Constants.Scoring.roller2ID, Constants.Scoring.sensorID);
        return m_Instance;
    }
    public double getElevatorEncoder() {
        return elevatorEncoder.getPosition();
    }
    public double getAngleEncoder() {
        return angleEncoder.getPosition();
    }

}
