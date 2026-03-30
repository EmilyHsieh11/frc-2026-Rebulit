package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class LoggableRobotPose {
    public final Pose3d pose;
    public final double timestamp;

    public LoggableRobotPose(Pose3d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
