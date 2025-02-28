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
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends SubsystemBase {

    // this spins in an arbitrary direction to intake a coral
    public static SparkMax intakeMotor;

    // photoelectric sensor to detect coral having been taken in
    public static DigitalInput coralDetector;

    private static Intake m_Instance = null;

    public boolean pickingUp = false;

    Swerve swerve;
    Scoring scoring;
    IntakeArm intakeArm;

    public Intake(int intakeMotorID) {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        coralDetector = new DigitalInput(2); // this port number is probably wrong!!!
    }

    // public void intakePeriodic(){
    //     if(coralDetector.get() && pickingUp){
    //         //intakeMotor.set(0);
    //         //coralPhase2();
    //     }
    // }

    public boolean getCoralDetector() {
        return coralDetector.get();
    }

    public void runIn(double speed) {
        intakeMotor.set(speed);
    }

    public void runOut(double speed) {
        intakeMotor.set(-speed);
    }

    public void intakeStop() {
        intakeMotor.set(0.0);
    }

    public void clearStickyFaults() {
        intakeMotor.clearFaults();
    }

    // public Command groundCoralCommand() {
    //     return this.runOnce(() -> coralPhase0());
    // }

    // public Command dropCoralCommand() {
    //     scoring.setLevel(3);
    //     return this.runOnce(() -> coralPhase3());
    // }

    public static Intake getInstance() {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID);
        return m_Instance;
    }

}