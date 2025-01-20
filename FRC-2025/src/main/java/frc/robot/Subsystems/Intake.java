package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    // this spins in an arbitrary direction to intake a coral
    public static SparkMax intakeMotor;

    // this runs between two set positions to pivot the intake
    public static SparkMax pivotMotor;

    public static final PIDController pivotPID = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);

    // photoelectric sensor to detect coral
    public static DigitalInput coralDetector;

    public static RelativeEncoder pivotEncoder;

    private static Intake m_Instance = null;

    public Intake (int intakeMotorID, int pivotMotorID) {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        pivotConfig.idleMode(IdleMode.kBrake);
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = pivotMotor.getEncoder();

        coralDetector = new DigitalInput(0); // this port number is probably wrong!!!
    }

    public void extend () {
        // move pivot motor to floor positon
        
    }

    public void retract () {
        // move pivot motor to initial position
    }

    public void runIn () {
        // power intake motor
        intakeMotor.set(1.0); // this speed might be wrong!!!
    }

    public void runOut () {
        intakeMotor.set(-0.25);
    }

    public void intakeStop () {
        // cut power to intake motor
        intakeMotor.set(0.0);
    }

    public void pivotStop () {
        pivotMotor.set(0.0);
    }

    public boolean hasCoral () {
        return !coralDetector.get();
    }

    public static Intake getInstance() {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID, Constants.Intake.pivotMotorID);
        return m_Instance;
    }

}