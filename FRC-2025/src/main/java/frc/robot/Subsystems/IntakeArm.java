package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
    private SparkMax armMotor;
    private static IntakeArm m_Instance = null;
    private DigitalInput outsideLimitSwitch;
    private DigitalInput insideLimitSwitch;

    public IntakeArm(int intakeArmMotorID) {
        armMotor = new SparkMax(intakeArmMotorID, MotorType.kBrushless);
        outsideLimitSwitch = new DigitalInput(0);
        insideLimitSwitch = new DigitalInput(1);
    }

    public boolean isInsideSwitchPressed() {
        return insideLimitSwitch.get();
    }

    public boolean isOutsideSwitchPressed() {
        return outsideLimitSwitch.get();
    }

    public void moveArmOut() {
        if (!isOutsideSwitchPressed())
            armMotor.set(.15);
        else
            armMotor.set(0);
    }

    public void moveArmIn() {
        if (!isInsideSwitchPressed())
            armMotor.set(-0.15);
        else
            armMotor.set(0);
    }

    public void stopArm() {
        armMotor.set(0);
    }

    public static IntakeArm getInstance() {
        if (m_Instance == null)
            m_Instance = new IntakeArm(Constants.IntakeArm.intakeArmMotorID);
        return m_Instance;
    }
}
