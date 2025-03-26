package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConst;
import frc.robot.Util.AprilTagPipelineSettings;

public class Cameras extends SubsystemBase{
    private AprilTagPipelineSettings frontPipeline;
    private AprilTagPipelineSettings rightPipeline;
    private AprilTagPipelineSettings leftPipeline;
    private edu.wpi.first.math.Vector<N3> singleStds;
    private edu.wpi.first.math.Vector<N3> multiStds;

    public AprilTagPipeline frontCamera;
    public AprilTagPipeline rightCamera;
    public AprilTagPipeline leftCamera;
    private Field2d cameraField;

    //Simulation
    // private VisionSystemSim visionSim;

    public Cameras(Drive drive) {

        singleStds = VecBuilder.fill(1, 1, 16);
        multiStds = VecBuilder.fill(0.5, 0.5, 1);

        frontPipeline = new AprilTagPipelineSettings(
            AprilTagFields.k2025ReefscapeWelded,
            CameraConst.robotToFrontCamera,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            3, 
            VecBuilder.fill(3, 3, 8), 
            multiStds,
            .2
        );
        
        rightPipeline = new AprilTagPipelineSettings(
            AprilTagFields.k2025ReefscapeWelded,
            CameraConst.robotToRightRearCamera,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            3, 
            singleStds , 
            multiStds,
            .2
        );

        leftPipeline = new AprilTagPipelineSettings(
            AprilTagFields.k2025ReefscapeWelded,
            CameraConst.robotToLeftRearCamera,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            3, 
            singleStds , 
            multiStds,
            .2
        );
        
        frontCamera = new AprilTagPipeline(frontPipeline, "Camera01", "AprilTagPipeline", drive);
        rightCamera = new AprilTagPipeline(rightPipeline, "Camera02", "AprilTagPipeline", drive);
        leftCamera = new AprilTagPipeline(leftPipeline, "Camera03", "AprilTagPipeline", drive);

        var layout = Shuffleboard.getTab("AprilTags")
                .getLayout("Camera Update", BuiltInLayouts.kList)
                .withSize(1, 4);

        cameraField = new Field2d();
        layout.add(cameraField).withWidget("Field");

        //Simulation
        // visionSim = new VisionSystemSim("main");
        // visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

        // visionSim.addCamera(frontCamera.cameraSim, frontPipeline.robot_to_camera);
        // visionSim.addCamera(leftCamera.cameraSim, leftPipeline.robot_to_camera);
        // visionSim.addCamera(rightCamera.cameraSim, rightPipeline.robot_to_camera);
        
    }

    @Override
    public void simulationPeriodic(){
        // visionSim.update(Drive.getInstance().getEstimatedPos());
    }  
}
