package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
    public SparkMax armMotor;
    public static IntakeArm m_Instance = null;
    public DigitalInput insideLimitSwitch;
    public RelativeEncoder armEncoder;
    public boolean isResetting;

    public IntakeArm(int intakeArmMotorID) {
        armMotor = new SparkMax(intakeArmMotorID, MotorType.kBrushless);
        insideLimitSwitch = new DigitalInput(1);
        armEncoder = armMotor.getEncoder();
    }

    public void reset(){
        armMotor.set(0.1);
    }

    public boolean isInsideSwitchPressed() {
        return insideLimitSwitch.get();
    }

    public void armPeriodic(){ // to be called peroidically inside robot.java
        if(armMotor.get() < 0 && isInsideSwitchPressed()){
            armMotor.set(0);
            if(isResetting){
                armEncoder.setPosition(0);
                Constants.PIDs.intakeArmPID.reset();
            }
        }
        
        if(isResetting) return;

        armMotor.set(Constants.PIDs.intakeArmPID.calculate(armEncoder.getPosition()*15/16));
    }

    public void setArmPos(double angle){ // in degrees
        Constants.PIDs.intakeArmPID.setSetpoint(angle);
    }

    public static IntakeArm getInstance() {
        if (m_Instance == null)
            m_Instance = new IntakeArm(Constants.IntakeArm.intakeArmMotorID);
        return m_Instance;
    }
}
