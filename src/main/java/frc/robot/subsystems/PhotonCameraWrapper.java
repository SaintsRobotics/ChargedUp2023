package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {

    public PhotonCamera camera;
    private PhotonPoseEstimator m_photonPoseEstimator = null;

    public static AprilTagFieldLayout kAprilTagFieldLayout = null;

    public PhotonCameraWrapper() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        camera = new PhotonCamera(VisionConstants.kCameraName);

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
            // on the field.
            kAprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            // Create pose estimator // TODO test what happens if kAprilTagFieldLayout is null
            m_photonPoseEstimator = new PhotonPoseEstimator(
                    kAprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera,
                    VisionConstants.kCameraOffset);

        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
            // we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout, no vision available", e.getStackTrace());
            m_photonPoseEstimator = null;
        }
    }

    /**
     * Returns the estimated robot pose if it can
     * 
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
     *         targets used to create
     *         the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (m_photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }
}