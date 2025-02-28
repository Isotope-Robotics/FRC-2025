package frc.robot;
import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


//Class for Auto Commands Only
public class RobotContainer {
    SendableChooser<Command> autoChooser;

    public Swerve swerve;
    public Scoring scoring;
    public Intake intake;
    public IntakeArm intakeArm;

    public RobotContainer() {
        swerve = Swerve.getInstance();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autochooser", autoChooser);
        scoring = Scoring.getInstance();
        intake = Intake.getInstance();
        intakeArm = IntakeArm.getInstance();
        //TODO: VERY IMPORTANT, LOOK AT LAST YEARS CODE TO SEE HOW TO CREATE AUTO COMMANDS, youll have to create new subsytems just for commands
        // i would create them but im so tired
        //NamedCommands.registerCommand("Intake Out", intake.whatever());
        //NamedCommands.registerCommand("Intake Coral from Ground", intake.groundCoralCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
