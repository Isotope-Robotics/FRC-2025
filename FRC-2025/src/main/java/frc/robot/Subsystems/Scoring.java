package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoring extends SubsystemBase {
    public static SparkMax scoringMotor1;
    public static SparkMax scoringMotor2;
    public static SparkMax scoringMotor3;
    public static SparkMax roller;
    public static DigitalInput sensor;
    private static Scoring m_Instance = null;

    public Scoring(int scoringMotor1ID, int scoringMotor2ID, int scoringMotor3ID, int rollerID, int sensorID) {
        scoringMotor1 = new SparkMax(scoringMotor1ID, MotorType.kBrushless);
        scoringMotor2 = new SparkMax(scoringMotor2ID, MotorType.kBrushless);
        scoringMotor3 = new SparkMax(scoringMotor3ID, MotorType.kBrushless);
        sensor = new DigitalInput(sensorID);
    }

    public boolean isScoringMecCLear(){
        return sensor.get();
    }

    public void setRollerSpeed(float speed) {
        roller.set(speed);
    }
 
    public static Scoring getInstance() {
        if(m_Instance == null)
            m_Instance = new Scoring(Constants.Scoring.scoringMotor1ID, Constants.Scoring.scoringMotor2ID, Constants.Scoring.scoringMotor3ID, Constants.Scoring.rollerID, Constants.Scoring.sensorID);
            return m_Instance;
        }
    }
