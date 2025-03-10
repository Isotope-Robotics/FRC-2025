package frc.robot;
import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.AutoCommands.AutoCommands;


//Class for Auto Commands Only
public class RobotContainer {
    SendableChooser<Command> autoChooser;

    public Swerve swerve;
    // public Scoring scoring;


    public RobotContainer() {
        swerve = Swerve.getInstance();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autochooser", autoChooser);
       
        NamedCommands.registerCommand("Intake Coral from Station", AutoCommands.intakeFromStation());
        NamedCommands.registerCommand("Drop Coral", AutoCommands.scoreL1());
        NamedCommands.registerCommand("Align Robot", AutoCommands.alignRobot());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
