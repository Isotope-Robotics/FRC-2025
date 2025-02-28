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

  public boolean isPickingUp;

  public int POVPressTime;

  public Pose2d trajectory;
  public boolean isFieldRel;

  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-coral");
  public NetworkTableEntry coralx = table.getEntry("tx");
  public NetworkTableEntry coraly = table.getEntry("ty");

  public NetworkTable april = NetworkTableInstance.getDefault().getTable("limelight-april");
  public NetworkTableEntry robotPosTargetspace = april.getEntry("robotpose_targetspace");

  public Pose2d AlignPose = null;

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
      //intake.intakePeriodic();
      scoring.scoringPeriodic();
      intakeArm.intakeArmPeriodic();
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
    SmartDashboard.putNumber("Elevator Encoder", scoring.getElevatorEncoder());
    SmartDashboard.putNumber("Scoring Angle Encoder", scoring.getAngleEncoder());
    SmartDashboard.putNumber("IntakeArm ENcoder", intakeArm.getArmEncoder());

  }

  // Move Robot to position and rotation compared to April Tag
  private void AlignRobot(Pose2d pose){

    double[] targetPoseData = robotPosTargetspace.getDoubleArray(new double[3]);
  
    Pose2d robotRelTarget = new Pose2d(targetPoseData[0], targetPoseData[1], new Rotation2d(targetPoseData[2]));
    Pose2d offset = pose.relativeTo(robotRelTarget);

    double speed = Constants.PIDs.AlignLinearPID.calculate(offset.getTranslation().getDistance(Translation2d.kZero));
    double angularSpeed = Constants.PIDs.AlignRotPID.calculate(offset.getRotation().getDegrees());

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

  public void coralAutoAim() {
    double tx = coralx.getDouble(0);
    double ty = coraly.getDouble(0);
    double ty_max = 21; // detemined empirically as the limelights vertical field of view
    double offset = -0.3; // inverse slope of focal line
    if(coralx.getDouble(40) == 40) return;
    trajectory = new Pose2d(trajectory.getTranslation(), new Rotation2d(Constants.PIDs.AimingPID.calculate(tx - offset * (ty-ty_max), 0)));
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
    if (Constants.Controllers.driver1.getRawButton(2)) {
      isAligning = false;
      isPickingUp = false;
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
  //     isAligning = true;
  //     AlignPose = new Pose2d(0.5,0.5,new Rotation2d(0));
  //   }

  //   if (Constants.Controllers.driver1Xbox.getYButton()) {
  //     isAligning = true;
  //     AlignPose = new Pose2d(-0.5,0.5,new Rotation2d(0));
  //   }

  //   if (Constants.Controllers.driver1Xbox.getXButton()) {
  //     //isLooking = true;
  //     coralAutoAim();
  //   }
    
  //   //Designate button to cancel aligning
  //   if (Constants.Controllers.driver1Xbox.getBButton()) {
  //     isAligning = false;
  //     //isLooking = false;
  //   }

  // }

  private void Driver2Controls() {

    // Automatic intake control, intake runs and extends out to pick up coral, once it detects it in the intake it stops and goes back
    if (Constants.Controllers.driver2.getAButton() ) { // A Button Auto Intake
      if (!intake.getCoralDetector()) { // MAY HAVE TO REMOVE THE ! IF THE SENSOR IS WACK
        intakeArm.setArmPosOut();
        intake.runIn(1.0);
      } else {
        intakeArm.setArmPosIn();
        intake.intakeStop();
      }
     } else // little confusing but this is an else if 

      // Backup control for intakeArm in/out
     if (Constants.Controllers.driver2.getRightBumperButton()) { // Right Bumper Extend IntakeArm Out
       intakeArm.setArmPosOut();
     } else {
       intakeArm.setArmPosIn();
     }
 
     // Backup control for intake suck/spit
     if (Constants.Controllers.driver2.getRightTriggerAxis() > 0.1) { // Right Trigger Variable Spit
       intake.runOut(Constants.Controllers.driver2.getRightTriggerAxis());
     } else if (Constants.Controllers.driver2.getLeftTriggerAxis() > 0.1) { // Left Trigger Variable Suck
       intake.runIn(Constants.Controllers.driver2.getLeftTriggerAxis());
     }

     // Elevator control starts from d-pad down and goes clockwise, press leftbumper to reset back to 0 to recieve coral
     // this makes sense to me but tweak it if u want 
    if (Constants.Controllers.driver2.getPOV() == 180) { // D-pad Down
      scoring.elevatorRun(1);
      scoring.wristRun(1);
    } else if (Constants.Controllers.driver2.getPOV() == 270) { // D-pad Left
      scoring.elevatorRun(2);
      scoring.wristRun(2);
    } else if (Constants.Controllers.driver2.getPOV() == 0) { // D-pad Up
      scoring.elevatorRun(3);
      scoring.wristRun(3);
    } else if (Constants.Controllers.driver2.getPOV() == 90) { // D-pad Right
      scoring.elevatorRun(4);
      scoring.wristRun(4);
    } else if (Constants.Controllers.driver2.getLeftBumperButton()) { // Left Bumper
      scoring.elevatorRun(0);
      scoring.wristRun(0);
    }

    // Enables manual control of the elevator using the left stick y axis, you could also make it activate when the stick value is > 0.1 or < -0.1
    if (Constants.Controllers.driver2.getStartButton() || Constants.Controllers.driver2.getBackButton()) {
      scoring.toggleManualControl();
    }

    // I had some thoughts about adding right stick control for manual wrist control but I won't add it unless necessary
    if (scoring.isManualControl()) {
      scoring.manualControl(-Constants.Controllers.driver2.getRawAxis(1)); // Left Stick Y Axis
    }

    if (Constants.Controllers.driver2.getBButton()) {
      scoring.runRollerOut();
    } else if (Constants.Controllers.driver2.getBButton()) {
      scoring.runRollerIn();
    } else {
      scoring.stopRoller();
    }



    

  }

  // I'm not deleting these to keep the order for the movements, but they cannot be used

  //   public void coralPhase0() {
  //     intake.pickingUp = true;
  //   // scoring.elevatorReset();
  //     intakeArm.setArmPosOut();
  //     if (Constants.PIDs.intakeArmPID.atSetpoint()) {
  //         coralPhase1();
  //     } 
  // }

  // public void coralPhase1() {
  //     // vision trys to pick up coral
  //     intake.runIn();
  //     if (intake.coralDetector()) {
  //         intake.intakeStop();
  //         // wheel control goes back to driver
  //         coralPhase2();
  //     }
  // }

  // public void coralPhase2() {
  //     intakeArm.setArmPosIn();
  //     if (Constants.PIDs.intakeArmPID.atSetpoint()) {
  //         coralPhase3();
  //     }
  // }

  // public void coralPhase3() {
  //     // move the coral into the scoring mech
  //     intake.runIn();
  //     scoring.runRollerIn();
  //     if (!scoring.isScoringMecClear()) {
  //         intake.intakeStop();
  //         scoring.stopRoller();
  //     }
  //     intake.pickingUp = false;
  // }
}

