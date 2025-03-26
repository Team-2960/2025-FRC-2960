package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.AprilTagPipelineSettings;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

/**
 * Manages connection to a single PhotonVision AprilTag Pipeline
 */
public class AprilTagPipeline extends SubsystemBase {
    private final Drive drive;

    private final AprilTagPipelineSettings settings;    //< Pipeline Settings
    private final PhotonCamera camera;                  //< Camera object
    private final PhotonPoseEstimator pose_est;         //< Most recent estimated pse

    private final Distance maxDistance;                 //< Maximum accepted target distance 
    private final Dimensionless maxAmbiguityThreshold;     //< Maximum accepted target ambiguity threshold

    private MutTime lastTimestamp;  //< Timestamp of the most recent pose estimation */
    private MutTime timestamp;

    // Shuffleboard
    private GenericEntry sb_PoseX;
    private GenericEntry sb_PoseY;
    private GenericEntry sb_PoseR;
    private GenericEntry sb_lastTimestamp;
    private GenericEntry sb_lastUpdatePeriod;

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

    /**
     * Constructor
     * 
     * @param settings Pipeline settings
     */
    public AprilTagPipeline(AprilTagPipelineSettings settings, String cameraName, String name, Drive drive) {
        this.settings = settings;
        // TODO: Find non-depricated method to get April Tag Field Locations
        camera = new PhotonCamera(cameraName);
        pose_est = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(settings.field_layout),
                settings.pose_strategy,
                settings.robot_to_camera);

        this.drive = drive;

        last_pose = new Pose2d();
        lastTimestamp = Seconds.mutable(0);
        timestamp = Seconds.mutable(0);
        maxDistance = Meters.mutable(settings.max_dist);
        maxAmbiguityThreshold = Value.mutable(settings.ambiguity_threshold);
        field = settings.field_layout;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("AprilTags")
                .getLayout(cameraName, BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_PoseX = layout.add("Pose X" + cameraName, 0).getEntry();
        sb_PoseY = layout.add("Pose Y" + cameraName, 0).getEntry();
        sb_PoseR = layout.add("Pose R" + cameraName, 0).getEntry();
        sb_lastTimestamp = layout.add("Last Timestamp" + cameraName, lastTimestamp.toShortString()).getEntry();
        sb_lastUpdatePeriod = layout.add("Time Since Last Update" + cameraName, 0).getEntry();

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
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        var unreadResults = camera.getAllUnreadResults();
        
        for (var change : unreadResults) {
            visionEst = pose_est.update(change);
            int iteration = 0;
            last_pose = new Pose2d();
            // Check if a pose was estimated
            if (visionEst.isPresent()) {
                timestamp.mut_replace(change.getTimestampSeconds(), Seconds);
                aprilTagList = new Pose3d[change.getTargets().size()];

                // Get found tag count and average distance
                for (var tag : change.getTargets()) {
                    var tag_pose3d = pose_est.getFieldTags().getTagPose(tag.getFiducialId());

                    if (!tag_pose3d.isEmpty()) {
                        Pose3d estPose = PhotonUtils.estimateFieldToRobotAprilTag(tag.getBestCameraToTarget(),
                                tag_pose3d.get(), settings.robot_to_camera.inverse());
                        if (tag.getPoseAmbiguity() <= maxAmbiguityThreshold.in(Value)) {
                            double targetDist = tag.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
                            if (targetDist <= maxDistance.in(Meters)) {
                                Vector<N3> est_std = settings.single_tag_std;
                                est_std = est_std.times(1 + (targetDist * targetDist / 30));

                                drive.addVisionPose(estPose.toPose2d(), timestamp, est_std);
                                last_pose = estPose.toPose2d();

                                aprilTagList[iteration] = AprilTagFieldLayout.loadField(field).getTagPose(tag.getFiducialId()).get();
                                iteration++;
                            }
                        }

                    }

                }

                if (iteration == 0) aprilTagList = new Pose3d[] {};

            }

        }
    }

    public Pose3d getRobotRelativeCamPos(){
        return new Pose3d(drive.getPose()).transformBy(settings.robot_to_camera);
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        sb_PoseX.setDouble(last_pose.getX());
        sb_PoseY.setDouble(last_pose.getY());
        sb_PoseR.setDouble(last_pose.getRotation().getDegrees());
        sb_lastTimestamp.setString(lastTimestamp.toShortString());
        sb_lastUpdatePeriod.setString(lastTimestamp.mut_times(-1).mut_plus(Timer.getFPGATimestamp(), Seconds).toShortString());

        //Advantage Scope
        as_aprilTags.set(aprilTagList);
        as_cameraPose.set(getRobotRelativeCamPos());
        as_estimatedCameraPose.set(last_pose);
    }
}