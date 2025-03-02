package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

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
        Constants.PIDs.intakeArmPID.setTolerance(2);
    }

    public void reset(){
        armMotor.set(0.1);
    }

    public double getArmEncoder() {
        return armEncoder.getPosition();
    }

    public boolean isInsideSwitchPressed() {
        return insideLimitSwitch.get();
    }


        
        public void intakeArmPeriodic() {
            
            armMotor.set(Constants.PIDs.intakeArmPID.calculate(armEncoder.getPosition()));
        }

        
    

    public void setArmPosIn(){ 
        Constants.PIDs.intakeArmPID.setSetpoint(4);
    }
    public void setArmPosOut(){ 
        Constants.PIDs.intakeArmPID.setSetpoint(-45);
    }

    public void clearStickyFaults() {
        armMotor.clearFaults();
    }

    public static IntakeArm getInstance() {
        if (m_Instance == null)
            m_Instance = new IntakeArm(Constants.IntakeArm.intakeArmMotorID);
            return m_Instance;
    }

}
