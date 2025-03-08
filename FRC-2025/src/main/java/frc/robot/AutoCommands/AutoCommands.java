package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.*;

public class AutoCommands {

    // public static Scoring m_Scoring = Scoring.getInstance();
    public static Swerve m_Swerve = Swerve.getInstance();


    public AutoCommands() {
        
    }

    public static Command alignRobot() {
        return Commands.runOnce(() -> {
            m_Swerve.AlignRobot(new Pose2d(0.3175,-0.6,new Rotation2d(0)));
        });
    }

    public static Command alignRobotId(int id) {
        return Commands.runOnce(() -> {
            m_Swerve.AlignRobot(new Pose2d(0.3175,-0.6,new Rotation2d(0)), id);
        });
    }

}