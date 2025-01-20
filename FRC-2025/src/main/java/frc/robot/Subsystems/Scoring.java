package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Scoring extends SubsystemBase {
    public static SparkMax angle;
    public static SparkMax elevator;
    public static SparkMax roller;
    public static DigitalInput sensor;
    public static DutyCycleEncoder dutyCycleEncoder;
    public static final double m_fullRange = 1.3;
    public static final double m_expectedZero = 0;

    private static Scoring m_Instance = null;

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;
    
        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;
    
        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;
    
        return input;
      }
    
    public Scoring(int angleID, int elevatorID, int rollerID, int sensorID, int dutyCycleEncoderID) {
        angle = new SparkMax(angleID, MotorType.kBrushless);
        elevator = new SparkMax(elevatorID, MotorType.kBrushless);
        roller = new SparkMax(rollerID, MotorType.kBrushless);
        sensor = new DigitalInput(sensorID);
        dutyCycleEncoder = new DutyCycleEncoder(dutyCycleEncoderID, m_fullRange, m_expectedZero);
    }

    double output = dutyCycleEncoder.get();
    double percentOfRange = m_fullRange * 0.1;
    double shiftedOutput = inputModulus(output, 0 - percentOfRange, m_fullRange - percentOfRange);

    // Checks if limit switch is clear
    public boolean isScoringMecCLear(){
        return sensor.get();
    }
    // Turns on Roller on scoring mecanism
    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }
    // Set elevator and anlge to levels with encoder ticks
    public void elevatorLevel0(){
    }
    
    public void elevatorLevel1() {
    }

    public void elevatorLevel2() {

    }

    public void elevatorLevel3() {
    }

    public void elevatorLevel4() {

    }

    public static Scoring getInstance() {
        if(m_Instance == null)
            m_Instance = new Scoring(Constants.Scoring.angleID, Constants.Scoring.elevatorID, Constants.Scoring.rollerID, Constants.Scoring.sensorID, Constants.Scoring.dutyCycleEncoderID);
            return m_Instance;
        }
    }
