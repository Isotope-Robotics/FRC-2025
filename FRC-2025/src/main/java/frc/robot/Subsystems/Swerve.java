package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator estimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Field2d field = new Field2d();
    
    public boolean isAligning;
    public Pose2d AlignPose;

    public RobotConfig config;

    public Pose2d trajectory = Pose2d.kZero;;
    public boolean isFieldRel;

    public Vision aprilTagVision;

    private static Swerve m_Instance = null;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonId);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        Constants.PIDs.AlignXPID.setTolerance(0.05);
        Constants.PIDs.AlignYPID.setTolerance(0.05);
        Constants.PIDs.AlignRotPID.setTolerance(5);

        aprilTagVision = new Vision("limelight-april");

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        try {
            estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), aprilTagVision.getGlobalRobotPose());
        } catch (NullPointerException e) {
            estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), Pose2d.kZero);
        }
        

        // Robot Config pulled from PathPlanner GUI Setting Page
        
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
        }

        // Set up custom logging to add the current path to a field 2d widget
        
    

    public void AlignRobot(Pose2d pose){
        try {
            driveTo(pose.relativeTo(Pose2d.kZero.relativeTo(aprilTagVision.getGlobalTargetPose())));
        } catch (NullPointerException e) {
            System.err.println("no Coral to align to");
        }
    }

    public void AlignRobot(Pose2d pose, int id){
        try {
            driveTo(pose.relativeTo(Pose2d.kZero.relativeTo(aprilTagVision.getGlobalTargetPose(id))));
        } catch (Exception e) {
            System.err.println("Apriltag Id not valid.");
        }
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }


    public void driveTo(Pose2d pose){
        isAligning = true; 
        AlignPose = pose;
    }

    private void drivePeriodic(boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                isFieldRel ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        trajectory.getX(),
                        trajectory.getY(),
                        trajectory.getRotation().getDegrees(),
                        getHeading())
                        : new ChassisSpeeds(
                                trajectory.getX(),
                                trajectory.getY(),
                                trajectory.getRotation().getDegrees()));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current", mod.getDriveCurrent());

        }

        trajectory = Pose2d.kZero;
    }

    public void drive(Pose2d trajectory, boolean isFieldRel){
        this.trajectory = trajectory;
        this.isFieldRel = isFieldRel;
    };


    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getFreakyPose() {
        return new Pose2d(getPose().getTranslation(), new Rotation2d(-getHeading().getDegrees()));

    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getPosGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getPosGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getPosGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    // Returns Gyro as a Rotation2d
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
    }

    public Rotation2d getPosGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    // For Telementry Info, Returns as a Double Value
    public double getRealYaw() {
        return -gyro.getYaw().getValueAsDouble();
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    // Returns Instance Of Swerve
    public static Swerve getInstance() {
        if (m_Instance == null) {
            m_Instance = new Swerve();
        }
        return m_Instance;
    }

    public void swervePeriodic() {

        // swerveOdometry.update(getPosGyroYaw(), getModulePositions());

        try {
            Pose2d globalpose = aprilTagVision.getGlobalRobotPose();
            if(globalpose.getTranslation().getDistance(getPose().getTranslation()) < Constants.Swerve.maxSpeed/4.0){
                estimator.addVisionMeasurement(globalpose, Timer.getFPGATimestamp());
                setPose(estimator.getEstimatedPosition());
            }
            // System.out.println(globalpose.getTranslation());
        } catch (NullPointerException e) {

        }

        field.setRobotPose(getPose());

        drivePeriodic(false);

        for (SwerveModule mod : mSwerveMods) {

            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder",
                    mod.getCANCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle",
                    mod.getPosition().angle.getDegrees());
        }
    }

    public void swerveCurrents() {
        for (SwerveModule mod : mSwerveMods) {

            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current", mod.getAngleCurrent());

        }
    }

    // public boolean limelightNoteAim(boolean isFieldRel) {
    //     boolean closeenough = false;
    //     double tx = limelightNoteTable.getEntry("tx").getFloat(0);
    //     double tx_max = 30.0f; // detemined empirically as the limelights field of view
    //     double error = 0.0f;
    //     double kP = 0.6f; // should be between 0 and 1, but can be greater than 1 to go even faster
    //     double kD = 0.0f; // should be between 0 and 1
    //     double steering_adjust = 0.0f;
    //     double acceptable_error_threshold = 7.0f / 360.0f; // 15 degrees allowable
    //     error = (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1 being 180
    //                                            // degrees away
    //     if (limelightNoteLastError == 0.0f) {
    //         limelightNoteLastError = tx;
    //     }
    //     double error_derivative = error - limelightNoteLastError;
    //     limelightNoteLastError = tx; // setting limelightlasterror for next loop

    //     if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
    //         steering_adjust = -1 * (kP * error + kD * error_derivative);
    //         closeenough = false;
    //     } else {
    //         closeenough = true;
    //     }

    //     final double xSpeed = 0;
    //     final double ySpeed = 0;
    //     drive(new Pose2d(xSpeed*Constants.Swerve.maxSpeed,ySpeed*Constants.Swerve.maxSpeed,new Rotation2d(steering_adjust * Constants.Swerve.maxAngularVelocity)), isFieldRel, false);

    //     // System.out.println("Note error: " + error);
    //     return closeenough;
    // }

    // public void limelightAprilTagAim(boolean isFieldRel) {
    //     double currentGyro = gyro.getYaw().getValueAsDouble();
    //     double mappedAngle = 0.0f;
    //     double angy = ((currentGyro % 360.0f));
    //     if (currentGyro >= 0.0f) {
    //         if (angy > 180) {
    //             mappedAngle = angy - 360.0f;
    //         } else {
    //             mappedAngle = angy;
    //         }
    //     } else {
    //         if (Math.abs(angy) > 180.0f) {
    //             mappedAngle = angy + 360.0f;
    //         } else {
    //             mappedAngle = angy;
    //         }
    //     }
    //     double tx = limelightAprilTable.getEntry("tx").getFloat(700);
    //     // System.out.println("tx april: " + tx);
    //     double tx_max = 30.0f; // detemined empirically as the limelights field of view
    //     double error = 0.0f;
    //     double kP = 2.0f; // should be between 0 and 1, but can be greater than 1 to go even faster
    //     double kD = 0.0f; // should be between 0 and 1
    //     double steering_adjust = 0.0f;
    //     double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
    //     if (tx != 0.0f) { // use the limelight if it recognizes anything, and use the gyro otherwise
    //         error = -1.0f * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and
    //                                                        // 1
    //                                                        // being 180 degrees away
    //     } else {
    //         error = mappedAngle / 180.0f; // scaling error between -1 and 1, with 0 being dead on, and 1 being 180
    //                                       // degrees
    //                                       // away
    //     }
    //     if (limelightAprilTagLastError == 0.0f) {
    //         limelightAprilTagLastError = tx;
    //     }
    //     double error_derivative = error - limelightAprilTagLastError;
    //     limelightAprilTagLastError = tx; // setting limelightlasterror for next loop

    //     if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
    //         steering_adjust = (kP * error + kD * error_derivative);
    //     }

    //     final double xSpeed = 0;
    //     final double ySpeed = 0;
    //     drive(new Pose2d(xSpeed*Constants.Swerve.maxSpeed,ySpeed*Constants.Swerve.maxSpeed,new Rotation2d(steering_adjust * Constants.Swerve.maxAngularVelocity)), isFieldRel, false);
    //     // System.out.println("raw angle: " + currentGyro + ", mapped angle: " +
    //     // mappedAngle + ", april tag error: " + error);
    // }

    Rotation2d swr = new Rotation2d(45);
    Rotation2d swr2 = new Rotation2d(-45);

    SwerveModuleState sw = new SwerveModuleState(0.0, swr);
    SwerveModuleState sw2 = new SwerveModuleState(0.0, swr2);

    public void lock() {
        for (SwerveModule mod : mSwerveMods) {
            if (mod.moduleNumber == 1 || mod.moduleNumber == 3) {
                mod.setDesiredState(sw, false);
            } else {
                mod.setDesiredState(sw2, false);
            }
        }
    }
}