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
    private AprilTagPipelineSettings aprilTagPipeline;
    private edu.wpi.first.math.Vector<N3> singleStds;
    private edu.wpi.first.math.Vector<N3> multiStds;

    private AprilTagPipeline camera03;
    private static Cameras cameras = null;

    public Cameras(){
        singleStds = VecBuilder.fill(1, 1, 1);
        multiStds = VecBuilder.fill(1, 1, 1);
        aprilTagPipeline = new AprilTagPipelineSettings(AprilTagFields.k2025Reefscape,
            new Transform3d(Constants.robotLength/2, Constants.robotWidth/2, 0.254, new Rotation3d(0, 60, 45)),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 4, singleStds , multiStds);
        //aprilTagPipeline = new AprilTagPipelineSettings(new Transform3d(Constants.robotLength/2, Constants.robotWidth/2, 0.254, new Rotation3d(0, 60, 45)));
        
        camera03 = new AprilTagPipeline(aprilTagPipeline, "Camera03", "AprilTagPipeline");
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
