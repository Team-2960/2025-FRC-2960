package frc.robot.Util;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class FieldLayout {
    private static FieldLayout fieldLayout = null;
    public enum StageFace {
        AMP,
        SOURCE,
        FAR
    }

    public enum ReefFace {
        CENTER,
        ZERO,
        SIXTY,
        ONETWENTY,
        ONEEIGHTY,
        TWOFOURTY,
        THREEHUNDRED,
    }

    public enum AlgaeType {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum NoteType {
        NEAR_STAGE,
        NEAR_SPEAKER,
        NEAR_AMP,
        FAR_SOURCE2,
        FAR_SOURCE1,
        FAR_MID,
        FAR_AMP1,
        FAR_AMP2
    }

    public static final Pose2d bProcessor = new Pose2d(5.973, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d bHPRight = new Pose2d(0.838, 0.657, Rotation2d.fromDegrees(54));
    public static final Pose2d bHPLeft = new Pose2d(0.838, 7.395, Rotation2d.fromDegrees(-54));
    
    public static final Pose2d bReefCenter = new Pose2d(4.475, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d bReef0 = new Pose2d(3.643, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d bReef60 = new Pose2d(4.059, 3.306, Rotation2d.fromDegrees(60));
    public static final Pose2d bReef120 = new Pose2d(4.891, 3.306, Rotation2d.fromDegrees(120));
    public static final Pose2d bReef180 = new Pose2d(5.662, 4.026, Rotation2d.fromDegrees(180));
    public static final Pose2d bReef240 = new Pose2d(5.096, 5.054, Rotation2d.fromDegrees(-60));
    public static final Pose2d bReef300 = new Pose2d(4.059, 4.746, Rotation2d.fromDegrees(-120));

    public static final Pose2d bAlgaeLeft = new Pose2d(1.205, 2.197, Rotation2d.fromDegrees(0));
    public static final Pose2d bAlgaeMiddle = new Pose2d(1.205, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d bAlgaeRight = new Pose2d(0.838, 5.855, Rotation2d.fromDegrees(0));

    public static final Pose2d rProcessor = new Pose2d(11.54, 8.051, Rotation2d.fromDegrees(0));
    public static final Pose2d rAlgaeLeft = new Pose2d(16.315, 2.197, Rotation2d.fromDegrees(0));
    public static final Pose2d rAlgaeMiddle = new Pose2d(16.315, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d rAlgaeRight = new Pose2d(16.315, 5.855, Rotation2d.fromDegrees(0));
    
    public static final Pose2d rHPRight = new Pose2d(16.681, 7.395, Rotation2d.fromDegrees(54));
    public static final Pose2d rHPLeft = new Pose2d(16.681, 0.657, Rotation2d.fromDegrees(-54));
    
    public static final Pose2d rReefCenter = new Pose2d(13.045, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d rReef0 = new Pose2d(13.876, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d rReef60 = new Pose2d(13.46, 4.746, Rotation2d.fromDegrees(60));
    public static final Pose2d rReef120 = new Pose2d(12.629, 4.746, Rotation2d.fromDegrees(120));
    public static final Pose2d rReef180 = new Pose2d(12.213, 4.026, Rotation2d.fromDegrees(180));
    public static final Pose2d rReef240 = new Pose2d(12.629, 3.306, Rotation2d.fromDegrees(-120));
    public static final Pose2d rReef300 = new Pose2d(13.461, 3.306, Rotation2d.fromDegrees(-60));
      
    public static final Pose2d bCageProcessor = new Pose2d(8.76, 0.792, Rotation2d.fromDegrees(0));
    public static final Pose2d bCageMiddle = new Pose2d(8.76, 1.883, Rotation2d.fromDegrees(0));
    public static final Pose2d bCageTable = new Pose2d(8.76, 2.973, Rotation2d.fromDegrees(0));

    public static final Pose2d rCageProcessor = new Pose2d(8.76, 5.078, Rotation2d.fromDegrees(0));
    public static final Pose2d rCageMiddle = new Pose2d(8.76, 6.169, Rotation2d.fromDegrees(0));
    public static final Pose2d rCageTable = new Pose2d(8.76, 7.26, Rotation2d.fromDegrees(0));

    public static final List<Pose2d> bReefFaces = new ArrayList<>();
    public static final List<Pose2d> rReefFaces = new ArrayList<>();

    public FieldLayout(){
        //Fill Blue Reef Faces ArrayList with reef face poses
        bReefFaces.add(bReefCenter);
        bReefFaces.add(bReef0);
        bReefFaces.add(bReef60);
        bReefFaces.add(bReef120);
        bReefFaces.add(bReef180);
        bReefFaces.add(bReef240);
        bReefFaces.add(bReef300);

        //Fill Red Reef Faces ArrayList with reef face poses
        rReefFaces.add(rReefCenter);
        rReefFaces.add(rReef0);
        rReefFaces.add(rReef60);
        rReefFaces.add(rReef120);
        rReefFaces.add(rReef180);
        rReefFaces.add(rReef240);
        rReefFaces.add(rReef300);
    }

    public static final Map<ReefFace, Pose2d> bReef = Map.of(
            ReefFace.CENTER, bReefCenter,
            ReefFace.ZERO, bReef0,
            ReefFace.SIXTY, bReef60,
            ReefFace.ONETWENTY, bReef120,
            ReefFace.ONEEIGHTY, bReef180,
            ReefFace.TWOFOURTY, bReef240,
            ReefFace.THREEHUNDRED, bReef300
        );
        
    public static final Map<ReefFace, Pose2d> rReef = Map.of(
            ReefFace.CENTER, rReefCenter,
            ReefFace.ZERO, rReef0,
            ReefFace.SIXTY, rReef60,
            ReefFace.ONETWENTY, rReef120,
            ReefFace.ONEEIGHTY, rReef180,
            ReefFace.TWOFOURTY, rReef240,
            ReefFace.THREEHUNDRED, rReef300
        );

    public static final Map<AlgaeType, Pose2d> bAlgaeType = Map.of(
            AlgaeType.LEFT, bAlgaeLeft,
            AlgaeType.MIDDLE, bAlgaeMiddle,
            AlgaeType.RIGHT, bAlgaeRight
        );
    
    public static final Map<AlgaeType, Pose2d> rAlgaeType = Map.of(
            AlgaeType.LEFT, bAlgaeLeft,
            AlgaeType.MIDDLE, bAlgaeMiddle,
            AlgaeType.RIGHT, bAlgaeRight
        );

    public static final double rAutoLineX = 6.137 + Constants.fieldCenterOffset.getX(); // Meters
    public static final double bAutoLineX = -6.137 + Constants.fieldCenterOffset.getX(); // Meters

    public static final double rWingLineX = 2.338 + Constants.fieldCenterOffset.getX(); // Meters
    public static final double bWingLineX = -2.338 + Constants.fieldCenterOffset.getX(); // Meters

    /**
     * Gets the pose of the amp for the current alliance
     * 
     * @return pose of the amp for the current alliance
     */
    public static Pose2d getAlgaeType(AlgaeType type) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rAlgaeType.get(type);
        } else {
            return rAlgaeType.get(type);
        }
    }

    /**
     * Gets the pose of the stage for the current alliance
     * 
     * @param face face to retrieve
     * @return pose of the stage for the current alliance
     */
    public static Pose2d getReef(ReefFace face) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rReef.get(face);
        } else {
            return bReef.get(face);
        }
    }

    public static List<Pose2d> getReefList(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rReefFaces;
        } else {
            return bReefFaces;
        }
    }

    /**
     * Gets the pose of the nearest reef face for the current alliance
     * 
     * @param pos current position
     * @return pose of the nearest reef face for the current alliance
     */
    public static Pose2d getNearestReefFace(Pose2d pose) {
        
        List<Pose2d> reeflist = bReefFaces;
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            reeflist = rReefFaces;
        } else {
            reeflist = bReefFaces;
        }

        return pose.nearest(reeflist);
    }

    public static Rotation2d getReefFaceZone(Pose2d pose){
        Rotation2d rotOffset = new Rotation2d();
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            rotOffset = Rotation2d.fromDegrees(180);
        }else{
            rotOffset = Rotation2d.fromDegrees(0);
        }

        Pose2d reef = getReef(ReefFace.CENTER); 
        Transform2d calcPose = pose.minus(reef);
        Rotation2d calcRotation = new Rotation2d(calcPose.getX(), calcPose.getY()).rotateBy(rotOffset);
        Rotation2d targetAngle = new Rotation2d();
        

        if (calcRotation.getDegrees() >= -30 && calcRotation.getDegrees() < 30){
            targetAngle = getReef(ReefFace.ZERO).getRotation();
        } else if (calcRotation.getDegrees() >= 30 && calcRotation.getDegrees() < 90){
            targetAngle = getReef(ReefFace.SIXTY).getRotation();
        } else if (calcRotation.getDegrees() >= 90 && calcRotation.getDegrees() <= 150){
            targetAngle = getReef(ReefFace.ONETWENTY).getRotation();
        } else if (calcRotation.getDegrees() >= 150 && calcRotation.getDegrees() <= 180){
            targetAngle = getReef(ReefFace.ONEEIGHTY).getRotation();
        }else if(calcRotation.getDegrees() >= -180 && calcRotation.getDegrees() < -150){
            targetAngle = getReef(ReefFace.ONEEIGHTY).getRotation();
        } else if (calcRotation.getDegrees() >= -150 && calcRotation.getDegrees() < -90){
            targetAngle = getReef(ReefFace.TWOFOURTY).getRotation();
        } else if (calcRotation.getDegrees() >= -90 && calcRotation.getDegrees() < -30){
            targetAngle = getReef(ReefFace.THREEHUNDRED).getRotation();
        }
        return targetAngle;
    }

    public static Translation2d getNoteOffset(AlgaeType algaeType, double x, double y){
        var algaePos = getAlgaeType(algaeType);
        return new Translation2d(algaePos.getX() + x, algaePos.getY() + y);
    }

    /**
     * Gets the x position of the autoline for the current alliance
     * 
     * @return x position of the autoline for the current alliance
     */
    public static double getAutoLineX() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rAutoLineX;
        } else {
            return bAutoLineX;
        }
    }

    /**
     * Gets the x position of the wing line for the current alliance
     * 
     * @return x position of the wing line for the current alliance
     */
    public static double getWingLineX() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rWingLineX;
        } else {
            return bWingLineX;
        }
    }

    /**
     * Gets the x position that will ensure the robot is clear of the auto zone line
     * 
     * @return x position that will ensure the robot is clear of the auto zone line
     */
    public static double getAutoClearX() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rAutoLineX - Constants.robotDiag - Constants.autoClearance;
        } else {
            return bAutoLineX + Constants.robotDiag + Constants.autoClearance;
        }
    }

    /**
     * Gets the forward angle for the robot for the current alliance
     * 
     * @return forward angle for the robot for the current alliance
     */
    public static Rotation2d getForwardAngle() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Rotation2d.fromDegrees(180);
        } else {
            return Rotation2d.fromDegrees(0);
        }
    }

    public static FieldLayout getInstance(){
        if (fieldLayout == null){
            fieldLayout = new FieldLayout();
        }
        return fieldLayout;
    }

}