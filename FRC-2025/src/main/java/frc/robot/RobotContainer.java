package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

//Class for Auto Commands Only
public class RobotContainer {
    SendableChooser<Command> autoChooser;

    public RobotContainer(){

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}
