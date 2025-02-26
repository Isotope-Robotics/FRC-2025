package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class Climber {
        private static SparkMax climbMotor;
        public static RelativeEncoder climbEncoder;
        public static final PIDController wristPID = new PIDController(Constants.Climber.kP, Constants.Climber.kI,
        Constants.Climber.kD);

        public Climber(int climbMoterID) {
            climbMotor = new SparkMax(climbMoterID, MotorType.kBrushless);
            climbEncoder = climbMotor.getEncoder();
            SparkMaxConfig climbConfig = new SparkMaxConfig();
            climbConfig.idleMode(IdleMode.kBrake);
            climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void setClimbPosition(){
            climbMotor.set(wristPID.calculate(climbEncoder.getPosition(),210));
        }
        public void setNonClimbPosition(){
            climbMotor.set(wristPID.calculate(climbEncoder.getPosition(),0));
        }

}
