package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.AprilTagPipelineSettings;
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Manages connection to a single PhotonVision AprilTag Pipeline
 */
public class AprilTagPipeline extends SubsystemBase {

    private final AprilTagPipelineSettings settings;                /**< Pipeline Settings */
    private final PhotonCamera camera;              /**< Camera object */
    private final PhotonPoseEstimator pose_est;     /**< Pose Estimator */

    private Pose2d last_pose;                       /**< Most recent estimated pose */
    private double last_timestamp;                  /**< Timestamp of the most recent pose estimation */

    // Shuffleboard 
    private GenericEntry sb_PoseX;
    private GenericEntry sb_PoseY;
    private GenericEntry sb_PoseR;
    private GenericEntry sb_lastTimestamp;
    private GenericEntry sb_lastUpdatePeriod;

    //Camera Simulation
    // VisionSystemSim visionSim;
    // TargetModel targetModel;
    // SimCameraProperties cameraProp;
    // PhotonCameraSim cameraSim;
    //VisionTargetSim visionTargetSim;

    //Test Values
    private double displayNum;
    private String resultsList;
    private boolean tagPresent;

    /**
     * Constructor
     * @param   settings    Pipeline settings
     */
    public AprilTagPipeline(AprilTagPipelineSettings settings, String cameraName, String name) {
        this.settings = settings;
       // TODO: Find non-depricated method to get April Tag Field Locations
        camera = new PhotonCamera(cameraName);
        pose_est = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(settings.field_layout), 
            settings.pose_strategy, 
            settings.robot_to_camera
        );

        last_pose = new Pose2d();
        last_timestamp = 0;
        

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("AprilTags" + camera)
                .getLayout(name, BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_PoseX = layout.add("Pose X" + cameraName, 0).getEntry();
        sb_PoseY = layout.add("Pose Y" + cameraName, 0).getEntry();
        sb_PoseR = layout.add("Pose R" + cameraName, 0).getEntry();
        sb_lastTimestamp = layout.add("Last Timestamp" + cameraName, last_timestamp).getEntry();
        sb_lastUpdatePeriod = layout.add("Time Since Last Update" + cameraName, 0).getEntry();

        //Vision Simulation
        // visionSim = new VisionSystemSim("main");
        // visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));
        // targetModel = TargetModel.kAprilTag16h5;
        // cameraProp = new SimCameraProperties();
        // cameraProp.setFPS(60);
        // cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        // cameraSim = new PhotonCameraSim(camera, cameraProp);
        // //visionTargetSim = new VisionTargetSim(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(17).get(), targetModel);
        // visionSim.addCamera(cameraSim, settings.robot_to_camera);
        // cameraSim.enableDrawWireframe(true);

        //Test Values
        //TODO Delete after testing
        displayNum = -1;
        resultsList = "";
        tagPresent = false;
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
        Drive drive = Drive.getInstance();
        var unreadResults = camera.getAllUnreadResults();
        // visionSim.update(new Pose2d(17.526/2, 8.05/2, new Rotation2d()));
        
        for(var change : unreadResults) {
            // Get Estimated Position
            visionEst = pose_est.update(change);

            // Check if a pose was estimated
            if(visionEst.isPresent()) {
                
                Pose2d est_pose = visionEst.get().estimatedPose.toPose2d();
                double est_timestamp = visionEst.get().timestampSeconds;

                double avg_dist = 0;
                int tag_count = 0;

                // Get found tag count and average distance
                for(var tag : visionEst.get().targetsUsed) {
                    var tag_pose3d = pose_est.getFieldTags().getTagPose(tag.getFiducialId());
                    
                    if(!tag_pose3d.isEmpty()) {
                        Pose2d tag_pose = tag_pose3d.get().toPose2d();
                        
                        avg_dist += tag_pose.getTranslation().getDistance(est_pose.getTranslation());
                        tag_count++;
                    }
                }

                // Check if any targets were found
                if(tag_count > 0) {
                    Vector<N3> est_std = settings.single_tag_std;

                    // Calculate average target distance
                    avg_dist /= tag_count;

                    // Decrease standard deviation if multiple tags are found
                    if(tag_count > 1) est_std = settings.multi_tag_std;

                    // Increase standard deviation based on average distance and ignore single 
                    // target results over the settings.max_dist
                    if(tag_count > 1 || avg_dist < settings.max_dist) {
                        // TODO Add average distance scaler to settings
                        est_std = est_std.times(1 + (avg_dist * avg_dist / 30));
                        
                        last_pose = est_pose;
                        last_timestamp = est_timestamp;
                        drive.addVisionPose(est_pose, est_timestamp, est_std);
                    }
                    
                }
            }
        }
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
    }
}