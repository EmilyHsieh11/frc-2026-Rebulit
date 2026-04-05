package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVision extends SubsystemBase{

    final AprilTagFieldLayout fieldTagLayout;
    private final PhotonCamera first_Camera;
    private int[] hubTargetIds;
    private boolean isRedAlliance = false;
    // private int Target_Tag_Id;
    private PhotonTrackedTarget bestTargetThisCycle = null;
    double timeOfLastTrackedHubTarget = 0;
    PhotonTrackedTarget lastTrackedHubTarget = new PhotonTrackedTarget(0, 0, 0, 0, -1, -1, 0, Transform3d.kZero, Transform3d.kZero, 0, new ArrayList<TargetCorner>(), new ArrayList<TargetCorner>());
    Pose3d hubTarget = Pose3d.kZero;
    double visionRobotToHubDistance = 0;
    // Pose2d visionRobotPose = Pose2d.kZero;
    Rotation2d hubHeading = Rotation2d.kZero;
    /* Use the current robot heading to keep track of where to target when aiming for the hub */
    Supplier<Pose2d> currentRobotPose;
    private LoggableRobotPose[] allPoses = new LoggableRobotPose[0];
    /** The PhotonPoseEstimator class filters or combines readings from all the AprilTags visible at a given timestamp on the field to produce a single robot in field pose, using the strategy set below.*/
    private final PhotonPoseEstimator photonPoseEstimator;


    public PhotonVision(Supplier<Pose2d> robotPoseSupplier) {
        fieldTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        first_Camera = new PhotonCamera("Steve");
        this.currentRobotPose = robotPoseSupplier;
        updateAllianceColor();

        photonPoseEstimator = new PhotonPoseEstimator(
            fieldTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // 優先使用多目標定位
            VisionConstants.ROBOT_TO_CAMERA_Steve
        );
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }


    // Alliance color && hub aprilTags
    private void updateAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            boolean currentIsRed = (alliance.get() == Alliance.Red);

            if (currentIsRed != this.isRedAlliance || hubTargetIds == null) {
                this.isRedAlliance = currentIsRed;
                if (isRedAlliance) {
                    hubTargetIds = VisionConstants.RedHubApriltagIds; 
                } else {
                    hubTargetIds = VisionConstants.BlueHubApriltagIds; 
                }
            }
        }
    }
    
    public boolean hasAnyTarget() {
        return first_Camera.getLatestResult().hasTargets();
    }

    public PhotonPipelineResult getLatestResult() {
        return first_Camera.getLatestResult();
    }
    
    /**
     * [機器人在場上的位置] -> [相機在機器人上的位置] -> [標籤在相機前方的位移] -> [Hub 在標籤後方的位移]
     * */

    // get best aprilTag target ID
    public void updateBestTargetTag() {
        var allResults = first_Camera.getAllUnreadResults(); 
        PhotonTrackedTarget bestTarget = null;
        for (var result : allResults){
            var allTargets = result.getTargets();
            for (PhotonTrackedTarget target : allTargets){
                if(Arrays.stream(hubTargetIds).anyMatch(x -> x == target.fiducialId)) {
                    if(bestTarget == null){
                        bestTarget = target;
                    }else if(target.poseAmbiguity < bestTarget.poseAmbiguity && target.poseAmbiguity > 0&& target.poseAmbiguity < 0.2){
                        bestTarget = target;
                    }
                }
            }
        }
        bestTargetThisCycle = bestTarget;
    }

    public void getVisionHubAbsolutePose() {
        if(bestTargetThisCycle != null) {
            lastTrackedHubTarget = bestTargetThisCycle;
            Transform3d cameraToTag = lastTrackedHubTarget.bestCameraToTarget;
            var tagToHub = isRedAlliance ? VisionConstants.RedHub.getHubPose(lastTrackedHubTarget.fiducialId) : VisionConstants.BlueHub.getHubPose(lastTrackedHubTarget.fiducialId);
            var robotPose = currentRobotPose.get();
            hubTarget = new Pose3d(robotPose).transformBy(VisionConstants.ROBOT_TO_CAMERA_Steve).transformBy(cameraToTag).transformBy(tagToHub);
        }
    }

    public Rotation2d getVisionHubHeading() {
        var robotPose = currentRobotPose.get();
        Translation2d robotToHubTranslation2d = hubTarget.getTranslation().toTranslation2d().minus(robotPose.getTranslation());
        Rotation2d hubHeading = robotToHubTranslation2d.getAngle();
        return hubHeading;

        // var robotToHub = hubTarget.relativeTo(new Pose3d(robotPose));
        // hubHeading =  robotPose.getRotation().plus(robotToHub.getTranslation().toTranslation2d().getAngle()
        // .plus(isRedAlliance ? Rotation2d.kZero : Rotation2d.k180deg));
    }

    public double getVisionRobotToHubDistance() {
        visionRobotToHubDistance = currentRobotPose.get().getTranslation().getDistance(hubTarget.getTranslation().toTranslation2d());
        return visionRobotToHubDistance;
    }

    private void updateVisionRobotPoseEstimates() {
        var allResults = first_Camera.getAllUnreadResults();
        ArrayList<LoggableRobotPose> estimates = new ArrayList<>();
        for (var result : allResults) {
            var estimate = photonPoseEstimator.update(result);
            if(estimate.isPresent()) {
                var val = estimate.get();
                estimates.add(new LoggableRobotPose(val.estimatedPose, val.timestampSeconds));
            }
        }
        /* And save them so we can feed them to the drivetrain */
        allPoses = estimates.toArray(new LoggableRobotPose[0]);
    }

    public LoggableRobotPose[] getVisionLatestEstimates() {
        return allPoses; // feed them to the drivetrain
    }










    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Steve_Camera_HasTarget",hasAnyTarget());
        SmartDashboard.putNumber("Vision_Robot_To_Hub_Distance", visionRobotToHubDistance);
        updateAllianceColor();
        updateBestTargetTag();
        updateVisionRobotPoseEstimates();
        if (bestTargetThisCycle != null && currentRobotPose != null) {
            getVisionHubAbsolutePose();
            getVisionHubHeading();
            getVisionRobotToHubDistance();
        }

    }


}
