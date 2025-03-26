package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Util.*;

public class Constants {
    public static final Transform2d fieldCenterOffset = new Transform2d(8.270875, 4.105275, new Rotation2d(0.0));

    // Robot constants
    // TODO Convert constants to units library for clarity
    public static final double updatePeriod = 0.02;//seconds
    
    public static final double robotWidth = 29.5 * .0254;   // Meters 
    public static final double robotLength = 29.5 * .0254;  // Meters 
    public static final double wheelInset = 1.75 * .0254;   // Meters
    public static final double robotDiag = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)); // Meters
    public static final double bumperThick = 3.25 * .0254;
    public static final double fullWidth = robotWidth + (bumperThick * 2.0);
    public static final double fullLength = robotLength + (bumperThick * 2.0);

    public static final int revTBEncCountPerRev = 4096;

    public static final ModuleConfig moduleConfig = new ModuleConfig(0.0381, 5.450, 1, new DCMotor(12, 4.69, 257, 1.5, 106.33, 1), 257, 1);

    // CAN IDs
    public static final int elevatorMotor = 11;

    public static final int coralMotor = 13;

    public static final int algaeAngleMotor = 3;
    public static final int algaeRollerMotor = 4;

    public static final int climberMotor = 14;

    public static final int armMotor = 12;

    public static final int frontLeftDriveM = 9;
    public static final int frontLeftAngleM = 10;
    public static final int frontRightDriveM = 7;
    public static final int frontRightAngleM = 8;

    public static final int backLeftDriveM = 1;
    public static final int backLeftAngleM = 2;
    public static final int backRightDriveM = 5;
    public static final int backRightAngleM = 6;

    // Digital Input Ports
    public static final int coralPresentPE = 9;

    // Auton
    public static final double autoClearance = .25; // Meters
    public static final double autonRampDownSpeed = 0.5;  
    public static final double minSpeed = 2;              // m/s
    //TODO Change MOI
    public static final RobotConfig robotConfig = new RobotConfig(52.16, 6.883, moduleConfig, robotWidth);

    //Preset Auton Positions
    public static final Pose2d redSourceSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d redCenter = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d redAmpSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueSourceSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueCenter = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueAmpSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static final Translation2d rightBranchOffset = new Translation2d(-fullLength/2.0 + 0.05, -.1109);
    public static final Translation2d leftBranchOffset = new Translation2d(-fullLength/2.0 + 0.05, -0.441);
    public static final Translation2d centerOffset = new Translation2d(-fullLength/2.0 + 0.05, 0);



    // Drive
    public static final double driveGearRatio = 5.08;
    public static final double wheelCirc = 3 * .0254 * Math.PI; // Meters
    public static final double driveRatio =  Constants.wheelCirc / Constants.driveGearRatio;   // Meters

    public static PIDParam drivePID = new PIDParam(1, 0.0, 0.0);  //.5
    public static FFParam driveFF = FFParam.simpleMotor(0.08, 2.5, 0.0);

    public static PIDParam driveAngPID = new PIDParam(0.05, 0.0, 0.001);
    public static PIDConstants driveAngPIDConstants = new PIDConstants(0.05, 0.0, 0.001);
    public static FFParam driveAngFF = FFParam.simpleMotor(0.1, 0.1, 0);


    public static final double maxSpeed = 4.5;
    public static final double maxAutoSpeed = 4.0;
    public static final double maxAngularSpeed = 1.5 * 2 * Math.PI;
    public static final double maxAutoAngularSpeed = 0.5 * 2 * Math.PI;

    public static final double swerveAngleRampRate = 7; 
    public static final Rotation2d swerveAngleRampDist = Rotation2d.fromDegrees(30);

    public static final double maxSwerveAngularSpeed = Math.PI * 4;     //Rad per second
    public static final double maxSwerveAngularAccel = Math.PI * 10;    //Rad per second ^ 2

    public static final Rotation2d driveAngleRampDistance = Rotation2d.fromDegrees(10);
    public static final PIDParam angleAlignPID = new PIDParam(4, 0, 0.3);
    public static final TrapezoidProfile.Constraints trapConstraints = new Constraints(4.5, 11);
    public static final double trapezoidTime = 0.1;
    public static final double alignRampDistance = 1.2;    //Meters

    public static final double alignLinearTolerance = 0.005;  //Meters
    public static final Rotation2d alignRotTolerance = Rotation2d.fromDegrees(2); //Degrees


    // Arm
    public static final PIDParam armPID = new PIDParam(0.002, 0.0, 0.0);
    public static final FFParam armFF = FFParam.arm(0.01 * 2, 0.02948 * 1.0, 0.025, 0.0);

    public static final Rotation2d armRampDownDist = Rotation2d.fromDegrees(20);

    public static final double maxArmSpeed = 90;   // Degrees / s
    public static final double maxArmAutoSpeed = 90;  //Degrees /s

    //TODO Change to real limits
    public static final Rotation2d armTopLim = Rotation2d.fromDegrees(95);
    public static final Rotation2d armBotLim = Rotation2d.fromDegrees(0);

    public static final Rotation2d armIntakeAngle = Rotation2d.fromDegrees(87);
    public static final Rotation2d armTravelAngle = Rotation2d.fromDegrees(75);
    public static final Rotation2d armL1CoralScoreAngle = Rotation2d.fromDegrees(73);
    public static final Rotation2d armCoralScoreAngle = Rotation2d.fromDegrees(60);
    public static final Rotation2d armCoralL4Angle = Rotation2d.fromDegrees(60);
    public static final Rotation2d armAlgaeRemoveAngle = Rotation2d.fromDegrees(25);

    //Elevator
    public static PIDParam elevatorPIDS = new PIDParam(0.0, 0.0, 0.0); //0.02132 0.0017692
    public static FFParam elevatorFFS = FFParam.elevator(0.22643, 4.4162, 0.33045, 0.32248);
    
    public static final double elevatorGearRatio = 1.0/12.0 * 2.0;    // rot. out / rot. in
    public static final double elevatorOutputDiam = 1.751;  // in.

    public static final double elevatorScale = elevatorGearRatio * elevatorOutputDiam * Math.PI;           // in. / rot.
    public static final double maxElevatorAutoSpeed = 2;    // in. / s
    public static final double elevatorRampDownDist = 10;    // in.
    public static final double elevatorDefTol = .5;         // in.
    public static final double elevatorPosTol = .5;         // in.

    public static final double elevatorTopLim = 57.5;         //in.
    public static final double elevatorBotLim = 0.25;        //in.

    public static final double elevIntakePos = 0;       // in.
    public static final double elevL1Pos = 0;           // in.
    public static final double elevL2Pos = 14;           // in.
    public static final double elevL3Pos = 29.5;           // in.
    public static final double elevL4Pos = 55.5;           // in.
    public static final double elevLowAlgaePos = 17.7;     // in.
    public static final double elevHighAlgaePos = 36;    // in.

    //End Effector
    public static final double coralEjectVolt = -12;
    public static final double coralIntakeVolt = -3;
    public static final double algaeRemovalVolt = -6;
    public static final double coralEjectTime = -1;
    public static final double coralReverseVolt = 2;

    //Algae Roller
    public static final PIDParam algaeAnglePID = new PIDParam(0.0, 0.0, 0.0);
    public static final FFParam algaeAngleFF = FFParam.arm(0.15488, 5.0985, 0.64833, 0.48409);

    
    public static final double maxAlgaeAutoSpeed = 1 * Math.PI;  //radians /s
    public static final Rotation2d algaeRampDownDist = Rotation2d.fromDegrees(30);

    public static final double algaeEjectVolt = -3;
    public static final double algaeIntakeVolt = 3;

    public static final Rotation2d algaeTopLim = Rotation2d.fromDegrees(88);
    public static final Rotation2d algaeBotLim = Rotation2d.fromDegrees(10);

    // Climber
    public static final double winchDiam = 1.5; // in.
    public static final double winchCircum = Math.PI * winchDiam; // in.

    public static final double winchMaxExtension = 88;   // in.
    public static final double winchMinLimit = 1.5; //in
    public static final double winchRatchedDelay = .25;  // seconds

    public static final double climberExtDist = 0.325; //rotations
    public static final double climberRetDist = 0.127; //rotations

    public static final double climberExtVolt = -6;      // voltage
    public static final double climberRetVolt = 12;     // voltage
    public static final double climberResetVolt = 6;    // voltage

    // Cameras
    public static final Transform3d robotToFrontCamera = new Transform3d(
        new Translation3d(-robotLength/2+.040, 0, .206), 
        new Rotation3d(36 * Math.PI / 180, 0, Math.PI)
    );  
    
    public static final Transform3d robotToLeftRearCamera = new Transform3d(
        new Translation3d(-robotLength/2+.040, 0, .206), 
        new Rotation3d(36 * Math.PI / 180, 0, Math.PI)
    );  
    
    public static final Transform3d robotToRightRearCamera = new Transform3d(
        new Translation3d(-robotLength/2+.040, 0, .206), 
        new Rotation3d(36 * Math.PI / 180, 0, Math.PI)
    );  

    //LEDs
    public static final int frontLEDCount = 30;
    public static final int leftLEDCount = 30;
    public static final int rightLEDCount = 30;
}
