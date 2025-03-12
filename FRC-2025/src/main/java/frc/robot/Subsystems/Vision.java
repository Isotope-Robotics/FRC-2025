package frc.robot.Subsystems;

import java.util.NoSuchElementException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    public NetworkTable april;
    private NetworkTableEntry robotPosTargetSpace;
    private NetworkTableEntry targetPosRobotSpace;
    private NetworkTableEntry globalRobotPose;
    private NetworkTableEntry aprilTagIDEntry; 

    public Vision(String tableID){
        april = NetworkTableInstance.getDefault().getTable(tableID);
        robotPosTargetSpace = april.getEntry("botpose_targetspace");
        targetPosRobotSpace = april.getEntry("targetpose_robotspace");
        globalRobotPose = april.getEntry("botpose");
        aprilTagIDEntry = april.getEntry("tid");
    }

    public static Pose2d toPose2d(NetworkTableEntry networkTable) throws NoSuchElementException{
        double[] posedata = networkTable.getDoubleArray(new double[0]);
        if(posedata.length == 0) throw new NoSuchElementException("No target was found");
        return new Pose3d(
            posedata[0],
            posedata[1],
            posedata[2],
            new Rotation3d(
                posedata[3],
                posedata[4],
                posedata[5]
            )
        ).toPose2d();
    }

    public Pose2d getRobotPosTargetSpace() throws NoSuchElementException{
        return toPose2d(robotPosTargetSpace);
    }

    public Pose2d getTargetPosRobotSpace() throws NoSuchElementException{
        return toPose2d(targetPosRobotSpace);
    }

    public Pose2d getGlobalRobotPose() throws NoSuchElementException{
        return toPose2d(globalRobotPose);
    }

    public Pose2d getGlobalTargetPose(int id) throws NoSuchElementException{
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(id).get().toPose2d();
    }

    public Pose2d getGlobalTargetPose() throws NoSuchElementException{
        return getGlobalTargetPose((int)aprilTagIDEntry.getInteger(-1));
    }

}
