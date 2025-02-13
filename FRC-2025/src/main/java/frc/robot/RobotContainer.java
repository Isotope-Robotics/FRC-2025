package frc.robot;
import frc.robot.Subsystems.*;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

//Class for Auto Commands Only
public class RobotContainer {
    SendableChooser<Command> autoChooser;

    public Swerve swerve;
    public Scoring scoring;
    public Intake intake;
    public IntakeArm intakeArm;

    public RobotContainer() {
        swerve = Swerve.getInstance();
        scoring = Scoring.getInstance();
        intake = Intake.getInstance();
        intakeArm = IntakeArm.getInstance();
        NamedCommands.registerCommand("Drop Coral", intake.dropCoralCommand());
        NamedCommands.registerCommand("Intake Coral from Ground", intake.groundCoralCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
