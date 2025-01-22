package frc.robot.Subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    // this spins in an arbitrary direction to intake a coral
    public static SparkMax intakeMotor;

    // photoelectric sensor to detect coral having been taken in
    public static DigitalInput coralDetector;

    private static Intake m_Instance = null;

    private static boolean coralStatus = false;

    public Intake (int intakeMotorID) {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        coralDetector = new DigitalInput(0); // this port number is probably wrong!!!
    }
    
    public void runIn () {
        // power intake motor
        intakeMotor.set(1.0); // this speed might be wrong!!!
        if (coralDetector.get())
            coralStatus = true;
        else
            coralStatus = false;
    }

    public void runOut () {
        intakeMotor.set(-0.25);
    }

    public void intakeStop () {
        // cut power to intake motor
        intakeMotor.set(0.0);
    }

    public boolean getCoralStatus () {
        return coralStatus;
    }

    public static Intake getInstance() {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID);
        return m_Instance;
    }

}