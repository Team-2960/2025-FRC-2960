package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConst;
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
