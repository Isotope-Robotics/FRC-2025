// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Scoring;
import frc.robot.Subsystems.IntakeArm;
import frc.robot.Subsystems.Intake;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public RobotContainer robotContainer;
  private Command m_AutonomousCommand;

  // Swerve Drive Varibles
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public Swerve swerve;
  public Conveyor conveyor;
  public Scoring scoring;
  public IntakeArm intakeArm;
  public Intake intake;

  public final XboxController myController = new XboxController(0);

  boolean isCoralReady = false;
  boolean isConvayorActive = false;
  boolean isIntakeReady = true;
  boolean isIntakePhase1 = false;
  boolean isIntakePhase2 = false;
  boolean isIntakePhase3 = false;
  boolean isIntakePhase4 = false;
  boolean IsArmMovingOut = false;
  Integer elevatorLevel = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    swerve = Swerve.getInstance();
    conveyor = Conveyor.getInstance();
    scoring = Scoring.getInstance();
    intakeArm = IntakeArm.getInstance();
    intake = Intake.getInstance();
    robotContainer = new RobotContainer();
    

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    swerve.swerveCurrents();
    RobotTelemetry();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Command m_AutonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    swerve.swerveOdometry.update(swerve.getGyroYaw(), swerve.getModulePositions());
    RobotTelemetry();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {// Destroy Auto Commands When Switching To TeleOP
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.cancel();
    }


    swerve.zeroHeading();
    RobotTelemetry();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    swerve.swerveOdometry.update(swerve.getPosGyroYaw(), swerve.getModulePositions());

    Driver1Controls();

    RobotTelemetry();
    
    if (isIntakeReady){
      if (myController.getAButtonPressed()){
        isIntakeReady = false;
        isIntakePhase1 = true;
      }
    }
    if(intake.getCoralStatus())
      intakeArm.moveArmIn();
    else
      intakeArm.moveArmOut();


    //when the first sensor detects something, the conveyor activates
    if (!conveyor.isConveyorClear()){
      conveyor.setSpeed(.8);
      scoring.setRollerSpeed(.8);
      isConvayorActive = true;
    }

    //when the second sensor detects something, the conveyor turns off and activates isCoralReady
    if (isConvayorActive){
      if (!scoring.isScoringMecCLear()){
        conveyor.setSpeed(0);
        scoring.setRollerSpeed(0);
        isConvayorActive = false;
        isCoralReady = true;
      }
    }
    
    //sends a signal to the driver saying that the coral is ready
    //if (isCoralReady)
      //send an LED signal
    //else
      //stop LED signal

    //moves the elevator position
    if (isCoralReady){
      if (myController.getRightBumperButtonPressed()){
        if (elevatorLevel < 4)
          elevatorLevel = elevatorLevel + 1;
        else if (elevatorLevel == 4)
          elevatorLevel = 0;
      }
      else if (myController.getLeftBumperButtonPressed()){
        if (elevatorLevel > 0)
          elevatorLevel = elevatorLevel - 1;
      }
    }

    if (elevatorLevel == 0){
      scoring.elevatorLevel0();
    }
    else
      scoring.elevatorLevel0();

    if (elevatorLevel == 1){
      scoring.elevatorLevel1();
    }
    else
      scoring.elevatorLevel1();

    if (elevatorLevel == 2){
      scoring.elevatorLevel2();
    }
    else
      scoring.elevatorLevel2();

    if (elevatorLevel == 3){
      scoring.elevatorLevel3();
    }
    else
      scoring.elevatorLevel3();

    if (elevatorLevel == 4){
      scoring.elevatorLevel4();
    }
    else
      scoring.elevatorLevel4();

    //switches control to automatic
    if (isCoralReady){
      if (myController.getXButton()){
        //wheel control = 2
      }

      if (myController.getBButton()){
        //wheel control = 3
      }
    }

    if (myController.getYButton()){
      isCoralReady = false;
      isIntakeReady = true;
      elevatorLevel = 0;
      //wheel control = 0
    }

    }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  // Add Telemetry Data for Robot
  private void RobotTelemetry() {

  }

  private void Driver1Controls() {
    // Back to robot centric while button seven is pushed
    if (Constants.Controllers.driver1.getRawButton(2)) {
      swerve.zeroHeading();
      System.out.println("Gyro reset");
    }
  }

  private void SwerveDrive(boolean isFieldRel) {
    // Controller Deadbands (Translation, Strafe, Rotation)

    double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1)
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);
    double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0)
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);
    double rot = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(3) // we made it unnegatived
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);

    // Drive Function
    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

  }
}
