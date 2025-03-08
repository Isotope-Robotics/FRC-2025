// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.math.MathUtil;

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
  // public Climber climber;
  // public Scoring scoring;

  public int POVPressTime;

  public boolean isCoralReady = false;

  // 0 represents that the intake is ready
  public int coralPhase = 0;
  
  
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
      swerve = Swerve.getInstance();
      // climber = Climber.getInstance();
      // scoring = Scoring.getInstance();
      // intake = Intake.getInstance();
      // intakeArm = IntakeArm.getInstance();
      
      robotContainer = new RobotContainer();
      
      // scoring.clearStickyFaults();
      // intake.clearStickyFaults();
      // intakeArm.clearStickyFaults();
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
      
      // scoring.scoringPeriodic();

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
    public void teleopInit() {
      // Destroy Auto Commands When Switching To TeleOP
      if (m_AutonomousCommand != null) {
        m_AutonomousCommand.cancel();
      }
      swerve.zeroHeading();
      RobotTelemetry();
      // scoring.elevatorRun(0);
      // scoring.wristRun(0);
    }
  
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
  
      swerve.swerveOdometry.update(swerve.getPosGyroYaw(), swerve.getModulePositions());
  
       Driver1Controls();
  
      // Driver1ControlsXbox();
  
      //Driver2Controls();

      RobotTelemetry();

      //SmartDashboard.putNumber("Elevator Encoder",scoring.getElevatorEncoder());
      //SmartDashboard.putNumber("Angle Encoder", scoring.getAngleEncoder());

    
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

  // This function is called periodically whilst in simulation.
  @Override
  public void simulationPeriodic() {
  }

  // Add Telemetry Data for Robot
  private void RobotTelemetry() {
    for (SwerveModule mod : swerve.mSwerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANCoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveCurrent());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current", mod.getDriveCurrent());
      
      
       
    }
   // SmartDashboard.putNumber("Elevator Encoder", scoring.getElevatorEncoder());
   // SmartDashboard.putNumber("Scoring Angle Encoder", scoring.getAngleEncoder());

  }

  //Move Robot to position and rotation compared to April Tag
  /*private void limelightAprilTagAim(boolean isFieldRel) {
    
    double tx = april.getEntry("tx").getFloat(700);
    // System.out.println("tx april: " + tx);
    double tx_max = 30.0f; // detemined empirically as the limelights field of view
    double error = 0.0f;
    double kP = .5f; // should be between 0 and 1, but can be greater than 1 to go even faster
    double kD = 0.0f; // should be between 0 and 1
    double steering_adjust = 0.0f;
    double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
    if (tx != 0.0f) { // use the limelight if it recognizes anything, and use the gyro otherwise
      error = 1.0f * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1
                                                     // being 180 degrees away
    }
    if (limelightAprilTagLastError == 0.0f) {
      limelightAprilTagLastError = tx;
    }
    double error_derivative = error - limelightAprilTagLastError;
    limelightAprilTagLastError = tx; // setting limelightlasterror for next loop

    if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
      steering_adjust = (kP * error + kD * error_derivative);
    }

    final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband);
    final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband);
    swerve.drive(new Pose2d(new Translation2d(0,0), new Rotation2d(limelightAprilTagLastError)).times(Constants.Swerve.maxSpeed), isFieldRel, false);

    // System.out.println("raw angle: " + currentGyro + ", mapped angle: " +
    // mappedAngle + ", april tag error: " + error);
  }*/

  public void stopAligning(){
    swerve.isAligning = false;
  }


  private void Driver1Controls() {

    // Swerve Control
    // If button 3 is pressed the swerve will be robot centric - not recommended for
    // daily driving
    // Else swerve will be field centric - recommended for daily driving

    double speedfactor = (Constants.Controllers.driver1.getRawButton(1) ? 0.25 : 1 );

    double xSpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband * speedfactor);
    double ySpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband * speedfactor);
    double rot = -MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(3),
        Constants.Controllers.stickDeadband * speedfactor);
    
    // Queue robot's trajectory
    
    swerve.drive(
      new Pose2d(xSpeed*Constants.Swerve.maxSpeed,ySpeed*Constants.Swerve.maxSpeed,
        new Rotation2d(rot * Constants.Swerve.maxAngularVelocity)),
      !Constants.Controllers.driver1.getRawButton(5)
    );

    //Controls for auto-aligning robot

    if (Constants.Controllers.driver1.getRawButtonPressed(4)) { // Align right reef
      swerve.AlignRobot(new Pose2d(-0.013,-0.6,new Rotation2d(180)));
    } 
    if (Constants.Controllers.driver1.getRawButtonPressed(3)) { // Align left reef
      swerve.AlignRobot(new Pose2d(0.3175,-0.6,new Rotation2d(180)));
    } 
    if (Constants.Controllers.driver1.getRawButtonReleased(3) || Constants.Controllers.driver1.getRawButtonReleased(4)){
      stopAligning();
    }
    /*if(Constants.Controllers.driver1.getRawButton(6)) { // Hang from climber
      limelightAprilTagAim(false);
    }*/
    if (Constants.Controllers.driver1.getRawButton(2)) { // cancel all autonomous actions
      swerve.zeroHeading();
       System.out.println("Gyro reset");
    } 
  }


  // Remember, ctrl + k + c to comment, ctrl + k + u to uncomment

  // private void Driver1ControlsXbox() {
  //   // Back to robot centric while button seven is pushed
  //   if (Constants.Controllers.driver1Xbox.getLeftBumperButton()) {
  //     swerve.zeroHeading();
  //     System.out.println("Gyro reset");
  //   }


  //   double xSpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1Xbox.getLeftY()
  //       * 0.5 * (1 + Constants.Controllers.driver1Xbox.getRightTriggerAxis()),
  //       Constants.Controllers.stickDeadband);
  //   double ySpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1Xbox.getLeftX()
  //       * 0.5 * (1 + Constants.Controllers.driver1Xbox.getRightTriggerAxis()),
  //       Constants.Controllers.stickDeadband);
  //   double rot = -MathUtil.applyDeadband(Constants.Controllers.driver1Xbox.getRightX()
  //       * 0.5 * (1 + Constants.Controllers.driver1Xbox.getRightTriggerAxis()),
  //       Constants.Controllers.stickDeadband);
    
  //   // Queue robot's trajectory
    
  //   trajectory = new Pose2d(xSpeed*Constants.Swerve.maxSpeed,ySpeed*Constants.Swerve.maxSpeed,new Rotation2d(rot * Constants.Swerve.maxAngularVelocity));
    
  //   // Field Relative
  //   isFieldRel = !Constants.Controllers.driver1Xbox.getRightBumperButton();

  //   // Controls for auto-aligning robot
  //   if (Constants.Controllers.driver1Xbox.getAButton()) {
  //     swerve.isAligning = true;
  //     swerve.AlignPose = new Pose2d(0.5,0.5,new Rotation2d(0));
  //   }

  //   if (Constants.Controllers.driver1Xbox.getYButton()) {
  //     swerve.isAligning = true;
  //     swerve.AlignPose = new Pose2d(-0.5,0.5,new Rotation2d(0));
  //   }

  //   if (Constants.Controllers.driver1Xbox.getXButton()) {
  //     //isLooking = true;
  //     coralAutoAim();
  //   }
    
  //   //Designate button to cancel aligning
  //   if (Constants.Controllers.driver1Xbox.getBButton()) {
  //     swerve.isAligning = false;
  //     //isLooking = false;
  //   }

  // }

  //private void Driver2Controls() {


 
     //Backup control for intake suck/spit
    //  if (Constants.Controllers.driver2.getRightTriggerAxis() > 0.1) { // Right Trigger Variable Spit
    //    scoring.runRollerOut(Constants.Controllers.driver2.getRightTriggerAxis() * 0.7);
    //  } else if (Constants.Controllers.driver2.getLeftTriggerAxis() > 0.1) { // Left Trigger Variable Suck
    //    scoring.runRollerIn(Constants.Controllers.driver2.getLeftTriggerAxis() * 0.7);
    //  } else if (Constants.Controllers.driver2.getBButton()) {
    //   scoring.runRollerOut(0.2);
    // } else if (Constants.Controllers.driver2.getXButton()) {
    //   scoring.runRollerIn(0.6);
    // } else {
    //   scoring.stopRoller();
    // }

     // Elevator control starts from d-pad down and goes clockwise, press leftbumper to reset back to 0 to recieve coral
     // this makes sense to me but tweak it if u want 
    // if (Constants.Controllers.driver2.getPOV() == 180) { // D-pad Down
    //   scoring.elevatorRun(1);
    //   scoring.wristRun(1);
    // } else if (Constants.Controllers.driver2.getPOV() == 270) { // D-pad Left
    //   scoring.elevatorRun(2);
    //   scoring.wristRun(2);
    // } else if (Constants.Controllers.driver2.getPOV() == 0) { // D-pad Up
    //   scoring.elevatorRun(3);
    //   scoring.wristRun(3);
    // } else if (Constants.Controllers.driver2.getPOV() == 90) { // D-pad Right
    //   scoring.elevatorRun(4);
    //   scoring.wristRun(4);
    // } else if (Constants.Controllers.driver2.getLeftBumperButton()) { // Left Bumper
    //   scoring.elevatorRun(0);
    //   scoring.wristRun(0);
    // }
     // Enables manual control of the elevator using the left stick y axis, you could also make it activate when the stick value is > 0.1 or < -0.1
    // if (Constants.Controllers.driver2.getBackButton()) {
    //   scoring.recalibratePosition();
    // }

    // if (Constants.Controllers.driver2.getYButtonPressed()) {
    //  scoring.toggleManualControl();
    // }


    // if (scoring.isManualControl()) {
    //   if(Math.abs(Constants.Controllers.driver2.getLeftY()) > 0.1)
    //   scoring.manualControlElevator(-Constants.Controllers.driver2.getLeftY()); // Left Stick Y Axis
    //   else {
    //   scoring.manualControlElevator(0);
    //   }
    //   if(Math.abs(Constants.Controllers.driver2.getRightY()) > 0.1)
    //   scoring.manualControlWrist(Constants.Controllers.driver2.getRightY()/2.0); // Right Stick Y Axis
    //   else {
    //   scoring.manualControlWrist(0);
    //   }
    // }
  }


