package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.*;

public class AutoCommands {

    // public static Scoring m_Scoring = Scoring.getInstance();
    public static Swerve m_Swerve = Swerve.getInstance();
    public static Scoring m_Scoring = Scoring.getInstance();

    public AutoCommands() {
        
    }

    public static Command scoreL1 () {
        return Commands.runOnce(() -> {
            m_Scoring.elevatorRun(1);
        }).andThen(() -> {
            m_Scoring.runRollerOut(0.5);
        }).andThen(Commands.waitSeconds(2)).andThen(() -> {
            m_Scoring.stopRoller();
        }).andThen(() -> {
            m_Scoring.elevatorRun(0);
        });
    }

    public static Command scoreL2 () {
        return Commands.runOnce(() -> {
            m_Scoring.elevatorRun(2);
        }).andThen(() -> {
            m_Scoring.runRollerOut(0.5);
        }).andThen(Commands.waitSeconds(2)).andThen(() -> {
            m_Scoring.stopRoller();
        }).andThen(() -> {
            m_Scoring.elevatorRun(0);
        });
    }

    public static Command scoreL3 () {
        return Commands.runOnce(() -> {
            m_Scoring.elevatorRun(3);
        }).andThen(() -> {
            m_Scoring.runRollerOut(0.5);
        }).andThen(Commands.waitSeconds(2)).andThen(() -> {
            m_Scoring.stopRoller();
        }).andThen(() -> {
            m_Scoring.elevatorRun(0);
        });
    }

    public static Command scoreL4 () {
        return Commands.runOnce(() -> {
            m_Scoring.elevatorRun(4);
        }).andThen(() -> {
            m_Scoring.runRollerOut(0.5);
        }).andThen(Commands.waitSeconds(2)).andThen(() -> {
            m_Scoring.stopRoller();
        }).andThen(() -> {
            m_Scoring.elevatorRun(0);
        });
    }

    public static Command intakeFromStation () {
        return Commands.runOnce(() -> {
            m_Scoring.elevatorRun(1);
        }).andThen(() -> {
            m_Scoring.runRollerIn(0.5);
        }).andThen(Commands.waitSeconds(5)).andThen(() -> {
            m_Scoring.elevatorRun(0);
        });
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