package org.team1515.SwerveTest;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVisionWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    private String camName = "";
    private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));


    public PhotonVisionWrapper() {
        // Set up a test arena of two apriltags at the center of each driver station set
        final AprilTag tag23 =
                new AprilTag(
                    1,
                    new Pose3d(
                        new Pose2d(
                            5, 5,
                            Rotation2d.fromDegrees(180))));
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        atList.add(tag23);

        // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
        AprilTagFieldLayout atfl =
                new AprilTagFieldLayout(atList, 5, 5);

        // Forward Camera
        photonCamera = new PhotonCamera(
                camName); // Change the name of your camera here to whatever it is in the
        // PhotonVision UI.

        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(
                atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, robotToCam);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the
     *         field, and the time
     *         of the observation. Assumes a planar field and the robot is always
     *         firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
