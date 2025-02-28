// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;

import static edu.wpi.first.units.Units.derive;


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
  //public Climber climber;
  public Scoring scoring;
  public Intake intake;
  public IntakeArm intakeArm;

  public boolean isAligning;

  public int POVPressTime;

  public Pose2d trajectory;
  public boolean isFieldRel;

  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-coral");
  public NetworkTableEntry coralx = table.getEntry("tx");
  public NetworkTableEntry coraly = table.getEntry("ty");

  public NetworkTable april = NetworkTableInstance.getDefault().getTable("limelight-scoring");
  public NetworkTableEntry targetPosCameraspace = april.getEntry("targetpose_cameraspace");


  public Pose2d AlignPose = null;

  public boolean isCoralReady = false;

  // 0 represents that the intake is ready
  public int coralPhase = 0;
  
  public boolean isLooking;
  
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
      swerve = Swerve.getInstance();
      //climber = Climber.getInstance();
      scoring = Scoring.getInstance();
      intake = Intake.getInstance();
      intakeArm = IntakeArm.getInstance();
  
      //robotContainer = new RobotContainer();
  
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
      CommandScheduler.getInstance().run();
      intake.intakePeriodic();
      scoring.scoringPeriodic();
      intakeArm.armPeriodic();
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
      trajectory = Pose2d.kZero;
  
      swerve.swerveOdometry.update(swerve.getPosGyroYaw(), swerve.getModulePositions());
  
      AlignPose = null;
  
      Driver1Controls();
  
      //Driver1ControlsXbox();
  
      Driver2Controls();
  
      try {
        if (isAligning){
          AlignRobot(AlignPose);
        }
      } catch (NullPointerException e) {
        System.err.println("No align pose was set!");
      }
  
    
    //if (isLooking){
    //  lookAtCoral();
    //}

    RobotTelemetry();

    //SmartDashboard.putNumber("Elevator Encoder",scoring.getElevatorEncoder());
    //SmartDashboard.putNumber("Angle Encoder", scoring.getAngleEncoder());

    swerve.drive(trajectory, isFieldRel, false);
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
  }

  // Move Robot to position and rotation compared to April Tag
  private void AlignRobot(Pose2d pose){

    double[] targetPoseData = targetPosCameraspace.getDoubleArray(new double[3]);

    Pose2d targetRelRobot = new Pose2d(targetPoseData[0], targetPoseData[1], new Rotation2d(targetPoseData[2]));
    Pose2d robotRelTarget = Pose2d.kZero.relativeTo(targetRelRobot);
    Pose2d offset = pose.relativeTo(robotRelTarget);

    double speed = -(1.0-1.0/Math.pow(5.0,offset.getTranslation().getDistance(Translation2d.kZero)))*Constants.Swerve.maxSpeed;
    double angularSpeed = -(1.0-1.0/Math.pow(5.0,offset.getRotation().getDegrees()/30.0))*Constants.Swerve.maxAngularVelocity;

    Translation2d velocity = offset.getTranslation().div(offset.getTranslation().getDistance(Translation2d.kZero)).times(speed);
    Rotation2d angularVelocity = offset.getRotation().div(Math.abs(offset.getRotation().getDegrees())).times(angularSpeed);
    
    if (offset.getTranslation().getDistance(Translation2d.kZero) > Constants.Vision.aligningTolerance) {
      trajectory = new Pose2d(velocity,angularVelocity);
      isFieldRel = false;
    } else {
      isAligning = false;
    }
  }

  /*private void lookAtCoral(){
    System.out.println(coralx.getDouble(0));
    if(coralx.getDouble(40) == 40) return;
    
    trajectory = new Pose2d(trajectory.getTranslation(), new Rotation2d(-Math.max(-1,Math.min(coralx.getDouble(0)-((coraly.getDouble(0)-21.0)*-0.5)/31.25,1)*Constants.Swerve.maxAngularVelocity)));
  }*/

  public boolean coralAutoAim() {
    boolean closeenough = false;
    double tx = coralx.getDouble(0);
    double tx_max = 31.25; // detemined empirically as the limelights horizontal field of view
    double ty = coralx.getDouble(0);
    double ty_max = 21; // detemined empirically as the limelights vertical field of view
    double error = 0.0;
    double kP = 0.6; // should be between 0 and 1, but can be greater than 1 to go even faster
    double kI = 0.1;
    double steering_adjust = 0.0; // between 0 and 1
    double acceptable_error_threshold = 7.0 / 360.0; // 15 degrees allowable
    double offset = -0.3; // in meters
    error = ((tx / tx_max) - (ty / ty_max * offset/Constants.Vision.limelightHeight*Math.tan(Math.toRadians(48))*tx_max/ty_max) + ty_max) * (31.25 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1 being 180 degrees away
    //error = (tx / tx_max) * (31.25 / 180) + (offset/180.0f); // scaling error between -1 and 1, with 0 being dead on, and 1 being 180 degrees away

    if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
      steering_adjust = (kP * error + kI * error);
      closeenough = false;
    } else {
        closeenough = true;
    }

    trajectory = new Pose2d(trajectory.getTranslation(), new Rotation2d(steering_adjust * Constants.Swerve.maxAngularVelocity));
    System.out.println("tx: " + tx);
    System.out.println("Note error: " + error);
    System.out.println("steering adjust: " + steering_adjust);
    return closeenough;
  }

  private void Driver1Controls() {
    // Back to robot centric while button seven is pushed
    if (Constants.Controllers.driver1.getRawButton(2)) {
      swerve.zeroHeading();
      System.out.println("Gyro reset");
    }

    // Swerve Control
    // If button 3 is pressed the swerve will be robot centric - not recommended for
    // daily driving
    // Else swerve will be field centric - recommended for daily driving

    double xSpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1)
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);
    double ySpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0)
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);
    double rot = -MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(3)
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);
    
    // Queue robot's trajectory
    
    trajectory = new Pose2d(xSpeed*Constants.Swerve.maxSpeed,ySpeed*Constants.Swerve.maxSpeed,new Rotation2d(rot * Constants.Swerve.maxAngularVelocity));
    
    isFieldRel = !Constants.Controllers.driver1.getRawButton(3);

    // Controls for auto-aligning robot
    if (Constants.Controllers.driver1.getRawButton(5)) {
      isAligning = true;
      AlignPose = new Pose2d(0.5,0.5,new Rotation2d(0));
    }

    if (Constants.Controllers.driver1.getRawButton(6)) {
      isAligning = true;
      AlignPose = new Pose2d(-0.5,0.5,new Rotation2d(0));
    }
    
    //Designate button to cancel everything
    if (Constants.Controllers.driver1.getRawButton(4)) {
      isAligning = false;
    }

  }

  private void Driver1ControlsXbox() {
    // Back to robot centric while button seven is pushed
    if (Constants.Controllers.driver1Xbox.getLeftBumperButton()) {
      swerve.zeroHeading();
      System.out.println("Gyro reset");
    }


    double xSpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1Xbox.getLeftY()
        * 0.5 * (1 + Constants.Controllers.driver1Xbox.getRightTriggerAxis()),
        Constants.Controllers.stickDeadband);
    double ySpeed = -MathUtil.applyDeadband(Constants.Controllers.driver1Xbox.getLeftX()
        * 0.5 * (1 + Constants.Controllers.driver1Xbox.getRightTriggerAxis()),
        Constants.Controllers.stickDeadband);
    double rot = -MathUtil.applyDeadband(Constants.Controllers.driver1Xbox.getRightX()
        * 0.5 * (1 + Constants.Controllers.driver1Xbox.getRightTriggerAxis()),
        Constants.Controllers.stickDeadband);
    
    // Queue robot's trajectory
    
    trajectory = new Pose2d(xSpeed*Constants.Swerve.maxSpeed,ySpeed*Constants.Swerve.maxSpeed,new Rotation2d(rot * Constants.Swerve.maxAngularVelocity));
    
    // Field Relative
    isFieldRel = !Constants.Controllers.driver1Xbox.getRightBumperButton();

    // Controls for auto-aligning robot
    if (Constants.Controllers.driver1Xbox.getAButton()) {
      isAligning = true;
      AlignPose = new Pose2d(0.5,0.5,new Rotation2d(0));
    }

    if (Constants.Controllers.driver1Xbox.getYButton()) {
      isAligning = true;
      AlignPose = new Pose2d(-0.5,0.5,new Rotation2d(0));
    }

    if (Constants.Controllers.driver1Xbox.getXButton()) {
      //isLooking = true;
      coralAutoAim();
    }
    
    //Designate button to cancel aligning
    if (Constants.Controllers.driver1Xbox.getBButton()) {
      isAligning = false;
      //isLooking = false;
    }

  }

  private void Driver2Controls() {
    if (Constants.Controllers.driver2.getAButton() /*&& !intake.pickingUp*/) {
      //intake.coralPhase0();
      intake.runIn();
    } else {
      intake.intakeStop();
    }
    if (Constants.Controllers.driver2.getRightBumperButtonPressed()) {
      scoring.setLevel(4);
    } else if (Constants.Controllers.driver2.getLeftBumperButtonPressed()) {
      scoring.setLevel(0);
    }
    if(Constants.Controllers.driver2.getPOV() != -1){
      POVPressTime++;
      if(POVPressTime == 1){
        if (Math.abs(Constants.Controllers.driver2.getPOV() - 180) > 150) {
          scoring.elevatorUp();
        } else if (Math.abs(Constants.Controllers.driver2.getPOV() - 180) < 30) {
          scoring.elevatorDown();
        }
      }
    }else{
      POVPressTime = 0;
    }
    /*if(Math.abs(Constants.Controllers.driver2.getLeftY()) > 0.1){
      ;
    }*/
    

    if (Constants.Controllers.driver2.getRightTriggerAxis() > 0.1) {
      intakeArm.setArmPos(Constants.IntakeArm.angleIn);
    } else if (Constants.Controllers.driver2.getLeftTriggerAxis() > 0.1) {
      intakeArm.setArmPos(Constants.IntakeArm.angleOut);
      intake.pickingUp = true;
    }

    /*if (Constants.Controllers.driver2.getYButton()) {
      isAligning = true;
      AlignPose = new Pose2d(-0.5,0.5,new Rotation2d(0));
    }

    if (Constants.Controllers.driver2.getXButton()) {
      //isLooking = true;
      coralAutoAim();
    }
    */
    //Designate button to cancel everything
    if (Constants.Controllers.driver2.getBButton()) {
      isAligning = false;
      //isLooking = false;
      intake.pickingUp = false;
    }

  }
}
