package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class Climber {
        private static SparkMax climbMotor;
        public static RelativeEncoder climbEncoder;

        private static Climber m_Instance = null;


        public Climber(int climbMotorID) {
            climbMotor = new SparkMax(climbMotorID, MotorType.kBrushless);
            climbEncoder = climbMotor.getEncoder();
            SparkMaxConfig climbConfig = new SparkMaxConfig();
            climbConfig.idleMode(IdleMode.kBrake);
            climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void climberPeriodic(){
            climbMotor.set(Constants.PIDs.climberPID.calculate(climbEncoder.getPosition()));
        }

        public void hang(){
            Constants.PIDs.climberPID.setSetpoint(210);
        }
        public void release(){
            Constants.PIDs.climberPID.setSetpoint(0);
        }

        public void clearStickyFaults() {
            climbMotor.clearFaults();
        }

        public static Climber getInstance() {
            if (m_Instance == null)
                m_Instance = new Climber(Constants.Climber.climbMotorID);
            return m_Instance;
        }

}
