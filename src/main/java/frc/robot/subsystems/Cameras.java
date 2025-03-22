package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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

    private AprilTagPipeline frontCamera;
    private AprilTagPipeline rightCamera;
    private AprilTagPipeline leftCamera;
    private Field2d cameraField;
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
            VecBuilder.fill(3, 3, 8), 
            multiStds,
            .2
        );
        
        rightPipeline = new AprilTagPipelineSettings(AprilTagFields.k2025ReefscapeWelded,
            new Transform3d(
                -13.944 /*in  */ * .0254, 
                13.944 * 0.0254, 
                8.791 * .0254, 
                new Rotation3d(
                    Math.toRadians(14.6),
                    Math.toRadians(14.6), 
                    Math.toRadians(135))),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            3, 
            singleStds , 
            multiStds,
            .2
        );

        leftPipeline = new AprilTagPipelineSettings(AprilTagFields.k2025ReefscapeWelded,
            new Transform3d(
                -13.944 /*in  */ * .0254, 
                -13.944 * 0.0254, 
                8.791 * .0254, 
                new Rotation3d(
                    Math.toRadians(-14.6),
                    Math.toRadians(14.6), 
                    Math.toRadians(-135))),
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
    }

    public void updateUI(){
        cameraField.getObject("left Camera Est").setPose(leftCamera.getEstCameraPos());
        cameraField.getObject("right Camera Est").setPose(rightCamera.getEstCameraPos());
        cameraField.getObject("front Camera Est").setPose(frontCamera.getEstCameraPos());
    }

    @Override
    public void periodic(){
    }

    public static Cameras getInstance(){
        if (cameras == null){
            cameras = new Cameras();
        }
        return cameras;
    }   
}
