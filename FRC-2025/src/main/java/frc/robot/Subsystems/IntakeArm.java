package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
    private static SparkMax armMotor;
    private static IntakeArm m_Instance = null;
    DigitalInput outsideLimitSwitch = new DigitalInput(0);
    DigitalInput insideLimitSwitch = new DigitalInput(1);

    public IntakeArm(int intakeArmMotorID) {
        armMotor = new SparkMax(intakeArmMotorID, MotorType.kBrushless);
    }

    public boolean isInsideSwitchPressed() {
        return insideLimitSwitch.get();
    }

    public boolean isOutsideSwitchPressed() {
        return outsideLimitSwitch.get();
    }

    public void moveArmOut() {
        if (!isOutsideSwitchPressed())
            armMotor.set(1);
        else
            armMotor.set(0);
    }

    public void moveArmIn() {
        if (!isInsideSwitchPressed())
            armMotor.set(-1);
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
