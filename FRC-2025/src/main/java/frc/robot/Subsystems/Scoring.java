package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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
    public static final PIDController wristPID = new PIDController(Constants.Scoring.kP, Constants.Scoring.kI,
            Constants.Scoring.kD);

    private static Scoring m_Instance = null;

    private int elevatorLevel = 0;

    public Scoring(int angleID, int elevatorID, int roller1ID, int roller2ID, int sensorID) {
        // Moter Declarations
        angle = new SparkMax(angleID, MotorType.kBrushless);
        elevator = new SparkMax(elevatorID, MotorType.kBrushless);
        roller1 = new SparkMax(roller1ID, MotorType.kBrushless);
        roller2 = new SparkMax(roller2ID, MotorType.kBrushless);
        sensor = new DigitalInput(sensorID);
        // Encoder Declarations
        angleEncoder = angle.getEncoder();
        elevatorEncoder = elevator.getEncoder();
        // Moter Configurations
        SparkMaxConfig angleConfig = new SparkMaxConfig();
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        angleConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.idleMode(IdleMode.kBrake);
        angle.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        elevatorLevel0();
        elevatorLevel = 0;
    }

    // Set elevator and anlge to levels with encoder ticks
    public void elevatorLevel0() {
        angle.set(wristPID.calculate(angleEncoder.getPosition(), 0.0));
        elevator.set(wristPID.calculate(elevatorEncoder.getPosition(), 0.0));
    }

    public void elevatorLevel1() {
        angle.set(wristPID.calculate(angleEncoder.getPosition(), 10));
        elevator.set(wristPID.calculate(elevatorEncoder.getPosition(), 20));
    }

    public void elevatorLevel2() {
        angle.set(wristPID.calculate(angleEncoder.getPosition(), 20));
        elevator.set(wristPID.calculate(elevatorEncoder.getPosition(), 50));
    }

    public void elevatorLevel3() {
        angle.set(wristPID.calculate(angleEncoder.getPosition(), 30));
        elevator.set(wristPID.calculate(elevatorEncoder.getPosition(), 60));
    }

    public void elevatorLevel4() {
        angle.set(wristPID.calculate(angleEncoder.getPosition(), 40));
        elevator.set(wristPID.calculate(elevatorEncoder.getPosition(), 80));
    }

    public void goToLevel() {
        switch (elevatorLevel % 5) {
            case 0: {
                elevatorLevel0();
            }
            case 1: {
                elevatorLevel1();
            }
            case 2: {
                elevatorLevel2();
            }
            case 3: {
                elevatorLevel3();
            }
            case 4: {
                elevatorLevel4();
            }
        }
    }

    public void elevatorUp() {
        elevatorLevel++;
        goToLevel();
    }

    public void elevatorDown() {
        elevatorLevel--;
        goToLevel();
    }

    public static Scoring getInstance() {
        if (m_Instance == null)
            m_Instance = new Scoring(Constants.Scoring.angleID, Constants.Scoring.elevatorID,
                    Constants.Scoring.roller1ID, Constants.Scoring.roller2ID, Constants.Scoring.sensorID);
        return m_Instance;
    }
}
