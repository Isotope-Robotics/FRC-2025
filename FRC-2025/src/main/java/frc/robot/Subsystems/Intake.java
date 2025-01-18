package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    // this spins in an arbitrary direction to intake a coral
    public static SparkMax intakeMotor;

    // this runs between two set positions to pivot the intake
    public static SparkMax pivotMotor;

    // photoelectric sensor to detect coral
    public static DigitalInput coralDetector;

    private static Intake m_Instance = null;

    public Intake (int intakeMotorID, int pivotMotorID) {
        intakeMotor = new SparkMax(intakeMotorID, motortype.kBrushless);
        pivotMotor = new SparkMax(pivotMotorID, motortype.kBrushless);

        intakeMotor.setIdleMode(Constants.Intake.Brake);
        pivotMotor.setIdleMode(Constants.Intake.Brake);

        coralDetector = new DigitalInput(0); // this port number is probably wrong!!!
    }

    public void extend () {
        // move pivot motor to floor positon
    }

    public void retract () {
        // move pivot motor to initial position
    }

    public void run () {
        // power intake motor
        intakeMotor.set(1.0); // this speed might be wrong!!!
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
        if (m_Instance == null) {
            m_Instance = new Intake(Constants.Intake.MotorID);
        }
        return m_Instance;
    }

}