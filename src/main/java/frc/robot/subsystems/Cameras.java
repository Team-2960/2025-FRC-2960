package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    
    private static Cameras cameras = null;

    public Cameras(){

        singleStds = VecBuilder.fill(1, 1, 16);
        multiStds = VecBuilder.fill(0.5, 0.5, 1);
        frontPipeline = new AprilTagPipelineSettings(AprilTagFields.k2025ReefscapeWelded,
            new Transform3d(
                14 /*in  */ * .0254, 
                0.125 * 0.0254, 
                8.875 * .0254, 
                new Rotation3d(
                    Math.toRadians(0), 
                    Math.toRadians(-10), 
                    Math.toRadians(0))),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            3, 
            VecBuilder.fill(.5, .5, 16), 
            multiStds,
            .2
        );
        
        rightPipeline = new AprilTagPipelineSettings(AprilTagFields.k2025ReefscapeWelded,
            new Transform3d(
                13.1 /*in  */ * .0254, 
                -12.614 * 0.0254, 
                11.068 * .0254, 
                new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(0), 
                    Math.toRadians(65))),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            3, 
            singleStds , 
            multiStds,
            .2
        );

        leftPipeline = new AprilTagPipelineSettings(AprilTagFields.k2025ReefscapeWelded,
            new Transform3d(
                0.876 /*in  */ * .0254, 
                12.357 * 0.0254, 
                11.068 * .0254, 
                new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(0), 
                    Math.toRadians(0))),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            3, 
            singleStds , 
            multiStds,
            .2
        );
        
        frontCamera = new AprilTagPipeline(frontPipeline, "Camera01", "AprilTagPipeline");
        rightCamera = new AprilTagPipeline(rightPipeline, "Camera02", "AprilTagPipeline");
        leftCamera = new AprilTagPipeline(leftPipeline, "Camera03", "AprilTagPipeline");

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

    public void updateUI(){
    }

    @Override
    public void periodic(){
        // visionSim.update(Drive.getInstance().getEstimatedPos());
    }

    public static Cameras getInstance(){
        if (cameras == null){
            cameras = new Cameras();
        }
        return cameras;
    }   
}
