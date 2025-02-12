package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Vector;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.AprilTagPipelineSettings;

public class Cameras extends SubsystemBase{
    private AprilTagPipelineSettings pipeline01;
    private AprilTagPipelineSettings pipeline02;
    private AprilTagPipelineSettings pipeline03;
    private edu.wpi.first.math.Vector<N3> singleStds;
    private edu.wpi.first.math.Vector<N3> multiStds;

    private AprilTagPipeline camera01;
    private AprilTagPipeline camera02;
    private AprilTagPipeline camera03;
    private static Cameras cameras = null;

    public Cameras(){
        singleStds = VecBuilder.fill(1, 1, 1);
        multiStds = VecBuilder.fill(1, 1, 1);
        pipeline01 = new AprilTagPipelineSettings(AprilTagFields.k2025Reefscape,
            new Transform3d(-Constants.robotLength/2, 0, 0.254, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(180))),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 4, singleStds , multiStds);
        
        pipeline02 = new AprilTagPipelineSettings(AprilTagFields.k2025Reefscape,
            new Transform3d(Constants.robotLength/2, -Constants.robotWidth/2, 0.254, new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-45))),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 4, singleStds , multiStds);

        pipeline03 = new AprilTagPipelineSettings(AprilTagFields.k2025Reefscape,
            new Transform3d(Constants.robotLength/2, Constants.robotWidth/2, 0.254, new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(45))),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 4, singleStds , multiStds);
        
        camera01 = new AprilTagPipeline(pipeline01, "Camera01", "AprilTagPipeline");
        camera02 = new AprilTagPipeline(pipeline02, "Camera02", "AprilTagPipeline");
        camera03 = new AprilTagPipeline(pipeline03, "Camera03", "AprilTagPipeline");
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
