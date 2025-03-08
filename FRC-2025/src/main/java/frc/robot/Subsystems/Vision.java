package frc.robot.Subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    public NetworkTable april;
    private NetworkTableEntry robotPosTargetSpace = april.getEntry("botpose_targetspace");
    private NetworkTableEntry targetPosRobotSpace = april.getEntry("targetpose_botspace");
    private NetworkTableEntry globalRobotPose = april.getEntry("botpose");
    private NetworkTableEntry aprilTagIDEntry = april.getEntry("tid");

    public Vision(String tableID){
        april = NetworkTableInstance.getDefault().getTable(tableID);
    }

    public static Pose2d toPose(NetworkTableEntry networkTable) throws NullPointerException{
        double[] posedata = networkTable.getDoubleArray(new double[0]);
        if(posedata.length == 0) throw new NullPointerException("No target was found");
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

    public Pose2d getRobotPosTargetSpace() throws NullPointerException{
        return toPose(robotPosTargetSpace);
    }

    public Pose2d getTargetPosRobotSpace() throws NullPointerException{
        return toPose(targetPosRobotSpace);
    }

    public Pose2d getGlobalRobotPose() throws NullPointerException{
        return toPose(globalRobotPose);
    }

    // public Pose2d getGlobalTargetPose(int id) throws NullPointerException{
    //     AprilTagFields.k2025ReefscapeAndyMark.getTagPose(id)
    // }

    // public Pose2d getGlobalTargetPose() throws NullPointerException{
    //     return getGlobalTargetPose((int)aprilTagIDEntry.getInteger(-1));
    // }

}
