package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
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
    private static Cameras cameras = null;

    public Cameras(){
        singleStds = VecBuilder.fill(4, 4, 8);
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
            3.5, 
            VecBuilder.fill(3, 3, 8), 
            multiStds,
            0
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
            4, 
            singleStds , 
            multiStds,
            0
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
            4, 
            singleStds , 
            multiStds,
            0
        );
        
        frontCamera = new AprilTagPipeline(frontPipeline, "Camera01", "AprilTagPipeline");
        rightCamera = new AprilTagPipeline(rightPipeline, "Camera02", "AprilTagPipeline");
        leftCamera = new AprilTagPipeline(leftPipeline, "Camera03", "AprilTagPipeline");
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
