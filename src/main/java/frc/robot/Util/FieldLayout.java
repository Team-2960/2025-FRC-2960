package frc.robot.Util;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class FieldLayout {
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

    public enum BranchID {
        A, B, C, D, E, F, G, H, I, J, K, L
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
    public static final Pose2d bReef240 = new Pose2d(5.096, 5.054, Rotation2d.fromDegrees(-120));
    public static final Pose2d bReef300 = new Pose2d(4.059, 4.746, Rotation2d.fromDegrees(-60));

    public static final Pose2d bBranchA = new Pose2d(3.643, 4.191, Rotation2d.fromDegrees(0.000));
    public static final Pose2d bBranchB = new Pose2d(3.643, 3.861, Rotation2d.fromDegrees(0.000));
    public static final Pose2d bBranchC = new Pose2d(3.917, 3.388, Rotation2d.fromDegrees(60.000));
    public static final Pose2d bBranchD = new Pose2d(4.202, 3.223, Rotation2d.fromDegrees(60.000));
    public static final Pose2d bBranchE = new Pose2d(4.748, 3.223, Rotation2d.fromDegrees(120.000));
    public static final Pose2d bBranchF = new Pose2d(5.033, 3.388, Rotation2d.fromDegrees(120.000));
    public static final Pose2d bBranchG = new Pose2d(5.307, 3.861, Rotation2d.fromDegrees(180.000));
    public static final Pose2d bBranchH = new Pose2d(5.307, 4.191, Rotation2d.fromDegrees(180.000));
    public static final Pose2d bBranchI = new Pose2d(5.033, 4.664, Rotation2d.fromDegrees(240.000));
    public static final Pose2d bBranchJ = new Pose2d(4.748, 4.828, Rotation2d.fromDegrees(240.000));
    public static final Pose2d bBranchK = new Pose2d(4.202, 4.828, Rotation2d.fromDegrees(300.000));
    public static final Pose2d bBranchL = new Pose2d(4.748, 4.664, Rotation2d.fromDegrees(300.000));

    public static final Pose2d bAlgaeLeft = new Pose2d(1.205, 2.197, Rotation2d.fromDegrees(0));
    public static final Pose2d bAlgaeMiddle = new Pose2d(1.205, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d bAlgaeRight = new Pose2d(0.838, 5.855, Rotation2d.fromDegrees(0));

    public static final Pose2d bCageProcessor = new Pose2d(8.76, 0.792, Rotation2d.fromDegrees(0));
    public static final Pose2d bCageMiddle = new Pose2d(8.76, 1.883, Rotation2d.fromDegrees(0));
    public static final Pose2d bCageTable = new Pose2d(8.76, 2.973, Rotation2d.fromDegrees(0));

    public static final List<Pose2d> bReefFaces = Arrays.asList(
        bReefCenter,
        bReef0,
        bReef60,
        bReef120,
        bReef180,
        bReef240,
        bReef300
    );

    public static final Map<ReefFace, Pose2d> bReef = Map.of(
        ReefFace.CENTER, bReefCenter,
        ReefFace.ZERO, bReef0,
        ReefFace.SIXTY, bReef60,
        ReefFace.ONETWENTY, bReef120,
        ReefFace.ONEEIGHTY, bReef180,
        ReefFace.TWOFOURTY, bReef240,
        ReefFace.THREEHUNDRED, bReef300
    );

    public static final List<Pose2d> bBranchList = Arrays.asList(
        bBranchA,
        bBranchB,
        bBranchC,
        bBranchD,
        bBranchE,
        bBranchF,
        bBranchG,
        bBranchH,
        bBranchI,
        bBranchJ,
        bBranchK,
        bBranchL
    );

    public static final Map<AlgaeType, Pose2d> bAlgaeType = Map.of(
            AlgaeType.LEFT, bAlgaeLeft,
            AlgaeType.MIDDLE, bAlgaeMiddle,
            AlgaeType.RIGHT, bAlgaeRight);

    public static final double bAutoLineX = -6.137 + Constants.fieldCenterOffset.getX(); // Meters

    public static final Pose2d rProcessor = new Pose2d(11.54, 8.051, Rotation2d.fromDegrees(0));

    public static final Pose2d rHPRight = new Pose2d(16.681, 7.395, Rotation2d.fromDegrees(54));
    public static final Pose2d rHPLeft = new Pose2d(16.681, 0.657, Rotation2d.fromDegrees(-54));

    public static final Pose2d rReefCenter = new Pose2d(13.045, 4.026, Rotation2d.fromDegrees(0));

    public static final Pose2d rReef0 = new Pose2d(13.876, 4.026, Rotation2d.fromDegrees(180));
    public static final Pose2d rReef60 = new Pose2d(13.46, 4.746, Rotation2d.fromDegrees(-60));
    public static final Pose2d rReef120 = new Pose2d(12.629, 4.746, Rotation2d.fromDegrees(-120));
    public static final Pose2d rReef180 = new Pose2d(12.213, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d rReef240 = new Pose2d(12.629, 3.306, Rotation2d.fromDegrees(60));
    public static final Pose2d rReef300 = new Pose2d(13.461, 3.306, Rotation2d.fromDegrees(120));
    
    public static final Pose2d rBranchA = new Pose2d(13.876, 3.861, Rotation2d.fromDegrees(180.000));
    public static final Pose2d rBranchB = new Pose2d(13.876, 4.191, Rotation2d.fromDegrees(180.000));
    public static final Pose2d rBranchC = new Pose2d(13.603, 4.664, Rotation2d.fromDegrees(240.000));
    public static final Pose2d rBranchD = new Pose2d(13.318, 4.828, Rotation2d.fromDegrees(240.000));
    public static final Pose2d rBranchE = new Pose2d(12.771, 4.828, Rotation2d.fromDegrees(300.000));
    public static final Pose2d rBranchF = new Pose2d(12.486, 4.664, Rotation2d.fromDegrees(300.000));
    public static final Pose2d rBranchG = new Pose2d(12.213, 4.191, Rotation2d.fromDegrees(0.000));
    public static final Pose2d rBranchH = new Pose2d(12.213, 3.861, Rotation2d.fromDegrees(0.000));
    public static final Pose2d rBranchI = new Pose2d(12.486, 3.388, Rotation2d.fromDegrees(60.000));
    public static final Pose2d rBranchJ = new Pose2d(12.771, 3.223, Rotation2d.fromDegrees(60.000));
    public static final Pose2d rBranchK = new Pose2d(13.318, 3.223, Rotation2d.fromDegrees(120.000));
    public static final Pose2d rBranchL = new Pose2d(13.603, 3.388, Rotation2d.fromDegrees(120.000));

    public static final Pose2d rAlgaeLeft = new Pose2d(16.315, 2.197, Rotation2d.fromDegrees(0));
    public static final Pose2d rAlgaeMiddle = new Pose2d(16.315, 4.026, Rotation2d.fromDegrees(0));
    public static final Pose2d rAlgaeRight = new Pose2d(16.315, 5.855, Rotation2d.fromDegrees(0));

    public static final Pose2d rCageProcessor = new Pose2d(8.76, 5.078, Rotation2d.fromDegrees(0));
    public static final Pose2d rCageMiddle = new Pose2d(8.76, 6.169, Rotation2d.fromDegrees(0));
    public static final Pose2d rCageTable = new Pose2d(8.76, 7.26, Rotation2d.fromDegrees(0));

    public static final List<Pose2d> rReefFaces = Arrays.asList(
        rReefCenter,
        rReef0,
        rReef60,
        rReef120,
        rReef180,
        rReef240,
        rReef300
    );

    public static final Map<ReefFace, Pose2d> rReef = Map.of(
            ReefFace.CENTER, rReefCenter,
            ReefFace.ZERO, rReef0,
            ReefFace.SIXTY, rReef60,
            ReefFace.ONETWENTY, rReef120,
            ReefFace.ONEEIGHTY, rReef180,
            ReefFace.TWOFOURTY, rReef240,
            ReefFace.THREEHUNDRED, rReef300);

    public static final List<Pose2d> rBranchList = Arrays.asList(
        rBranchA,
        rBranchB,
        rBranchC,
        rBranchD,
        rBranchE,
        rBranchF,
        rBranchG,
        rBranchH,
        rBranchI,
        rBranchJ,
        rBranchK,
        rBranchL
    );

    public static final Map<AlgaeType, Pose2d> rAlgaeType = Map.of(
            AlgaeType.LEFT, bAlgaeLeft,
            AlgaeType.MIDDLE, bAlgaeMiddle,
            AlgaeType.RIGHT, bAlgaeRight);

    public static final double rAutoLineX = 6.137 + Constants.fieldCenterOffset.getX(); // Meters

    /**
     * Checks if the current alliance is red. defaults to blue if no alliance is set
     * 
     * @return true if current alliance is red. false if current alliance is blue or
     *         not set
     */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    /**
     * Gets the pose of the amp for the current alliance
     * 
     * @return pose of the amp for the current alliance
     */
    public static Pose2d getAlgaeType(AlgaeType type) {
        return isRedAlliance() ? rAlgaeType.get(type) : bAlgaeType.get(type);
    }

    /**
     * Gets the pose of the stage for the current alliance
     * 
     * @param face face to retrieve
     * @return pose of the stage for the current alliance
     */
    public static Pose2d getReef(ReefFace face) {
        return isRedAlliance() ? rReef.get(face) : bReef.get(face);
    }

    /**
     * Get a list of all the reef face poses
     * 
     * @return list if all reef face poses
     */
    public static List<Pose2d> getReefList() {
        return isRedAlliance() ? rReefFaces : bReefFaces;
    }

    /**
     * Gets the pose of the nearest reef face for the current alliance
     * 
     * @param pos current position
     * @return pose of the nearest reef face for the current alliance
     */
    public static Pose2d getNearestReefFace(Pose2d pose) {
        return pose.nearest(getReefList());
    }

    public static Rotation2d getReefFaceZone(Pose2d pose){
        Pose2d reef = getReef(ReefFace.CENTER); 
        Transform2d calcPose = pose.minus(reef);
        Rotation2d calcRotation = calcPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        Rotation2d targetAngle = new Rotation2d();

        double angle = calcRotation.getDegrees();

        if (angle >= -180 && angle < -150) {
            targetAngle = Rotation2d.fromDegrees(180);
        } else if (angle >= -150 && angle < -90) {
            targetAngle = Rotation2d.fromDegrees(-120);
        } else if (angle >= -90 && angle < -30) {
            targetAngle = Rotation2d.fromDegrees(-60);
        } else if (angle >= -30 && angle < 30) {
            targetAngle = Rotation2d.fromDegrees(0);
        } else if (angle >= 30 && angle < 90) {
            targetAngle = Rotation2d.fromDegrees(60);
        } else if (angle >= 90 && angle <= 150) {
            targetAngle = Rotation2d.fromDegrees(120);
        } else if (angle >= 150 && angle <= 180) {
            targetAngle = Rotation2d.fromDegrees(180);
        }

        return targetAngle;
    }

    /**
     * Gets the pose for scoring on a given branch
     * @param branch    id of the branch to find
     * @return          pose for scoring on the selected branch
     */
    public static Pose2d getBranch(BranchID branch) {
        Pose2d result = new Pose2d();
        if(isRedAlliance()) {
            switch(branch) {
				case A: result = rBranchA;
						break;
				case B: result = rBranchB;
						break;
				case C: result = rBranchC;
						break;
				case D: result = rBranchD;
						break;
				case E: result = rBranchE;
						break;
				case F: result = rBranchF;
						break;
				case G: result = rBranchG;
						break;
				case H: result = rBranchH;
						break;
				case I: result = rBranchI;
						break;
				case J: result = rBranchJ;
						break;
				case K: result = rBranchK;
						break;
				case L: result = rBranchL;
						break;
            }
        } else {
            switch(branch) {
                case A: result = bBranchA;
						break;
				case B: result = bBranchB;
						break;
				case C: result = bBranchC;
						break;
				case D: result = bBranchD;
						break;
				case E: result = bBranchE;
						break;
				case F: result = bBranchF;
						break;
				case G: result = bBranchG;
						break;
				case H: result = bBranchH;
						break;
				case I: result = bBranchI;
						break;
				case J: result = bBranchJ;
						break;
				case K: result = bBranchK;
						break;
				case L: result = bBranchL;
						break;
            }
        }

        return result;
    }

    /**
     * Gets the scoring pose for the nearest branch to a given pose
     * @param pose  selected pose
     * @return  scoring pose for the nearest branch to the given pose
     */
    public static Pose2d getNearestBranch(Pose2d pose) {
        return pose.nearest(isRedAlliance() ? rBranchList : bBranchList);
    }

    /**
     * Gets the field position of an algae with an offset applied
     * 
     * @param algaeType desired algae
     * @param offset    offset to apply
     * @return translation to the target algae with offset applied
     */
    public static Translation2d getAlgaeOffset(AlgaeType algaeType, Translation2d offset) {
        var algaePos = getAlgaeType(algaeType);

        return algaePos.getTranslation().plus(offset);
    }

    /**
     * Gets the x position of the autoline for the current alliance
     * 
     * @return x position of the autoline for the current alliance
     */
    public static double getAutoLineX() {
        return isRedAlliance() ? rAutoLineX : bAutoLineX;
    }

    /**
     * Gets the x position that will ensure the robot is clear of the auto zone line
     * 
     * @return x position that will ensure the robot is clear of the auto zone line
     */
    public static double getAutoClearX() {
        double distance = 0;

        if (isRedAlliance()) {
            distance = rAutoLineX - Constants.robotDiag - Constants.autoClearance;
        } else {
            distance = bAutoLineX + Constants.robotDiag + Constants.autoClearance;
        }

        return distance;
    }

    /**
     * Gets the forward angle for the robot for the current alliance
     * 
     * @return forward angle for the robot for the current alliance
     */
    public static Rotation2d getForwardAngle() {
        return isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
    }

}