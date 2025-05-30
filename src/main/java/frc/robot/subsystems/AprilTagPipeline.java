package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.AprilTagPipelineSettings;
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Manages connection to a single PhotonVision AprilTag Pipeline
 */
public class AprilTagPipeline extends SubsystemBase {

    private final AprilTagPipelineSettings settings;
    /** < Pipeline Settings */
    private final PhotonCamera camera;
    /** < Camera object */
    private final PhotonPoseEstimator pose_est;
    /** < Pose Estimator */

    /** < Most recent estimated pose */
    private double last_timestamp;
    /** < Timestamp of the most recent pose estimation */
    private double maxDistance;
    private final double ambiguity_threshold;

    // Shuffleboard
    private GenericEntry sb_PoseX;
    private GenericEntry sb_PoseY;
    private GenericEntry sb_PoseR;
    private GenericEntry sb_lastTimestamp;
    private GenericEntry sb_lastUpdatePeriod;
    private GenericEntry sb_aprilTagSeen;
    private boolean aprilTagSeen;

    //Advantage Scope
    private StructArrayPublisher<Pose3d> as_aprilTags;
    private StructPublisher<Pose3d> as_cameraPose; //Camera Pose Relative to Robot on the Field
    private StructPublisher<Pose2d> as_estimatedCameraPose;
    private Pose3d[] aprilTagList;
    private AprilTagFields field;
    private Pose2d last_pose;  //Do NOT Use for any estimates


    // Camera Simulation
    // TargetModel targetModel;
    // SimCameraProperties cameraProp;
    // PhotonCameraSim cameraSim;
    // VisionTargetSim visionTargetSim;

    // Test Values
    private double displayNum;
    private String resultsList;
    private boolean tagPresent;

    /**
     * Constructor
     * 
     * @param settings Pipeline settings
     */
    public AprilTagPipeline(AprilTagPipelineSettings settings, String cameraName, String name) {
        this.settings = settings;
        // TODO: Find non-depricated method to get April Tag Field Locations
        camera = new PhotonCamera(cameraName);
        pose_est = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(settings.field_layout),
                settings.pose_strategy,
                settings.robot_to_camera);

        last_pose = new Pose2d();
        last_timestamp = 0;
        maxDistance = settings.max_dist;
        ambiguity_threshold = settings.ambiguity_threshold;
        field = settings.field_layout;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("AprilTags")
                .getLayout(cameraName, BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_PoseX = layout.add("Pose X" + cameraName, 0).getEntry();
        sb_PoseY = layout.add("Pose Y" + cameraName, 0).getEntry();
        sb_PoseR = layout.add("Pose R" + cameraName, 0).getEntry();
        sb_lastTimestamp = layout.add("Last Timestamp" + cameraName, last_timestamp).getEntry();
        sb_lastUpdatePeriod = layout.add("Time Since Last Update" + cameraName, 0).getEntry();
        sb_aprilTagSeen = layout.add(cameraName + "April Tag Read", false).getEntry();

        //Advantage Scope
        as_aprilTags = NetworkTableInstance.getDefault()
            .getStructArrayTopic(cameraName, Pose3d.struct).publish();

        as_cameraPose = NetworkTableInstance.getDefault()
            .getStructTopic(cameraName + " pose", Pose3d.struct).publish();

        as_estimatedCameraPose = NetworkTableInstance.getDefault()
            .getStructTopic(cameraName + " Estimated Pose", Pose2d.struct).publish();

        // Vision Simulation
        // targetModel = TargetModel.kAprilTag16h5;
        // cameraProp = new SimCameraProperties();
        // cameraProp.setFPS(60);
        // cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        // cameraSim = new PhotonCameraSim(camera, cameraProp);
        // cameraSim.enableDrawWireframe(true);

        // Test Values
        // TODO Delete after testing
        displayNum = -1;
        resultsList = "";
        tagPresent = false;

        aprilTagList = new Pose3d[] {};
    }

    /**
     * Period method. Updates UI.
     */
    @Override
    public void periodic() {
        updatePose();
        updateUI();
    }

    /**
     * Updates camera pose estimation
     */
    private void updatePose() {
        List<PhotonTrackedTarget> visionEst = List.of();
        Drive drive = Drive.getInstance();
        var unreadResults = camera.getAllUnreadResults();
        aprilTagSeen = false;
        
        for (var change : unreadResults) {
            visionEst = change.getTargets();
            int iteration = 0;
            last_pose = new Pose2d();
            // Check if a pose was estimated
            if (!visionEst.isEmpty()) {
                double est_timestamp = change.getTimestampSeconds();
                aprilTagList = new Pose3d[change.getTargets().size()];
                // Get found tag count and average distance
                for (var tag : visionEst) {
                    var tag_pose3d = PhotonUtils.estimateFieldToRobotAprilTag(
                        tag.bestCameraToTarget, 
                        pose_est.getFieldTags().getTagPose(tag.fiducialId).get(), 
                        settings.robot_to_camera.inverse());

                        if (tag.getPoseAmbiguity() <= ambiguity_threshold) {
                            double dist = tag.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
                            if (dist <= maxDistance) {
                                Vector<N3> est_std = settings.single_tag_std;
                                est_std = est_std.times(1 + (dist * dist / 30));

                                drive.addVisionPose(tag_pose3d.toPose2d(), est_timestamp, est_std);
                                last_pose = tag_pose3d.toPose2d();

                                aprilTagList[iteration] = AprilTagFieldLayout.loadField(field).getTagPose(tag.getFiducialId()).get();
                                iteration++;
                                aprilTagSeen = true;
                            }
                        }

                }

                if (iteration == 0) aprilTagList = new Pose3d[] {};

            }

        }
    }

    public Pose3d getRobotRelativeCamPos(){
        return new Pose3d(Drive.getInstance().getEstimatedPos()).transformBy(settings.robot_to_camera);
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        sb_PoseX.setDouble(last_pose.getX());
        sb_PoseY.setDouble(last_pose.getY());
        sb_PoseR.setDouble(last_pose.getRotation().getDegrees());
        sb_lastTimestamp.setDouble(last_timestamp);
        sb_lastUpdatePeriod.setDouble(Timer.getFPGATimestamp() - last_timestamp);
        sb_aprilTagSeen.setBoolean(aprilTagSeen);

        //Advantage Scope
        as_aprilTags.set(aprilTagList);
        as_cameraPose.set(getRobotRelativeCamPos());
        as_estimatedCameraPose.set(last_pose);
    }
}