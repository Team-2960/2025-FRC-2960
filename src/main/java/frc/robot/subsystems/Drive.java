package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;
import frc.robot.subsystems.Drive.LinearDriveCommands.DriveRateCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.AngleAlignCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.PointAlignCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.ReefAlignCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.RotationRateCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class Drive extends SubsystemBase {
    public enum AngleControlMode {
        AngleRate,
        Angle,
        LookAtPoint
    }

    private static Drive drive = null; // Statically initialized instance

    private final Translation2d frontLeftLocation;
    private final Translation2d frontRightLocation;
    private final Translation2d backLeftLocation;
    private final Translation2d backRightLocation;

    private final Swerve frontLeft;
    private final Swerve frontRight;
    private final Swerve backLeft;
    private final Swerve backRight;

    private final AHRS navx;

    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final SwerveDriveKinematics kinematics;

    private double rSpeed;
    private Translation2d targetPoint = new Translation2d();
    private AngleControlMode angleMode = AngleControlMode.AngleRate;
    private boolean fieldRelative = false;
    private ChassisSpeeds chassisSpeeds;
    private PIDController angleAlignPID;
    private PIDController driveAlignPID;

    private boolean isLinearManualDrive = true;
    private boolean isRotManualDrive = true;

    // Shuffleboard
    private GenericEntry sb_posEstX;
    private GenericEntry sb_posEstY;
    private GenericEntry sb_posEstR;

    private GenericEntry sb_speedX;
    private GenericEntry sb_speedY;
    private GenericEntry sb_speedR;
    private GenericEntry sb_robotTargetAngle;
    private GenericEntry sb_speedTargetR;
    private GenericEntry sb_linearCommand;
    private GenericEntry sb_driveCommand;
    private GenericEntry sb_rotationCommand;
    private GenericEntry sb_batteryVoltage;

    private ComplexWidget sb_field2d;
    private Pose2d nearestReefFace;

    private Field2d field2d;
    private FieldObject2d fieldTargetPoint;

    // PathPlanner
    public RobotConfig config;
    public AutoBuilder autoBuilder;

    // AdvantageScope
    private StructArrayPublisher<SwerveModuleState> as_swerveModules;
    private StructPublisher<Pose2d> as_currentPose;

    PathConstraints pathConstraints;

    /**
     * Subsystem to handle linear motion commands
     */
    public class LinearDriveCommands extends SubsystemBase {
        /**
         * Command for rate based control
         */

        public class DriveRateCommand extends Command {
            double xSpeed;      /**< Desired X axis rate */
            double ySpeed;      /**< Desired Y axis rate */

            /**
             * Constructor
             * @param xSpeed    Desired X axis rate
             * @param ySpeed    Desired Y axis rate
             */
            public DriveRateCommand(double xSpeed, double ySpeed) {
                this.xSpeed = xSpeed;
                this.ySpeed = ySpeed;
                addRequirements(LinearDriveCommands.this);
            }


            /**
             * Sets the desired speeds
             * @param xSpeed    Desired X axis rate
             * @param ySpeed    Desired Y axis rate
             */
            public void setSpeeds(double xSpeed, double ySpeed) {
                this.xSpeed = xSpeed;
                this.ySpeed = ySpeed;
            }

            /**
             * Updates X and Y axis rates
             */
            @Override
            public void execute() {
                updateKinematics(xSpeed, ySpeed);
            }

            //HACK this is a temporary solution to finish the drive rate command
            //You get updates from the OperatorInterface using the setLinearManualDrive method
            @Override
            public boolean isFinished(){
                return !isLinearManualDrive;
            }

        }

        
        public class LinearGoToReefCommand extends Command{
            Translation2d offset;

            public LinearGoToReefCommand(Translation2d offset){
                this.offset = offset;
                addRequirements(LinearDriveCommands.this);
            }

            public void setOffset(Translation2d offset){
                this.offset = offset;
            }

            @Override
            public void execute(){
                linearGoToReef(offset);
            }

            @Override
            public boolean isFinished(){
                return PhotonUtils.getDistanceToPose(getEstimatedPos(), getReefFace(offset)) <= Constants.alignLinearTolerance;
            }   
        }

        // public class AutonLinearGoToReefCommand extends Command{
        //     Translation2d offset;

        //     public AutonLinearGoToReefCommand(Translation2d offset){
        //         this.offset = offset;
        //         addRequirements(LinearDriveCommands.this);
        //     }

        //     public void setOffset(Translation2d offset){
        //         this.offset = offset;
        //     }


        //     @Override
        //     public void initialize(){
        //         PPHolonomicDriveController.overrideXFeedback(() -> 
        //             drive.getCalcToPoint(
        //                 drive.calcGoToReef(
        //                     new Pose2d(Constants.rightBranchOffset, 
        //                     Rotation2d.fromDegrees(0))).getTranslation()).get(1));
                
        //         PPHolonomicDriveController.overrideYFeedback(() -> 
        //             drive.getCalcToPoint(
        //                 drive.calcGoToReef(
        //                     new Pose2d(Constants.rightBranchOffset, 
        //                     Rotation2d.fromDegrees(0))).getTranslation()).get(2));
        //     }

        //     @Override
        //     public boolean isFinished(){
        //         return PhotonUtils.getDistanceToPose(getEstimatedPos(), getReefFace(offset)) <= Constants.alignLinearTolerance;
        //     }   

        //     @Override
        //     public void end(boolean interrupt){
        //         PPHolonomicDriveController.clearXYFeedbackOverride();
        //     }
        // }

        public class LinearDoNothingCommand extends Command{

            public LinearDoNothingCommand(){
                addRequirements(LinearDriveCommands.this);
            }

            @Override
            public void execute(){
                updateKinematics(0, 0);
            }
        }

        private final DriveRateCommand driveRateCommand;    /**< Internal instance of Drive Rate command */
        private final LinearGoToReefCommand linearGoToReefCommand;
        private final LinearDoNothingCommand doNothingCommand;


        /**
         * Constructor
         */
        public LinearDriveCommands() {
            driveRateCommand = new DriveRateCommand(0, 0);
            linearGoToReefCommand = new LinearGoToReefCommand(new Translation2d());
            doNothingCommand = new LinearDoNothingCommand();
            setDefaultCommand(doNothingCommand);
        }
    }

    /**
     * Subsystem for controlling rotation of the robot
     */
    public class RotationDriveCommands extends SubsystemBase {
        /**
         * Command for controlling the angle rate
         */
        public class RotationRateCommand extends Command {
            double rSpeed;  /**< Target angle rate */

            /**
             * Constructor
             * @param rSpeed    Target angle rate
             */
            public RotationRateCommand(double rSpeed) {
                this.rSpeed = rSpeed;
                addRequirements(RotationDriveCommands.this);
            }


            /**
             * Set the target angle rate
             * @param rSpeed    Target angle rate
             */
            public void setRotationRate(double rSpeed) {
                this.rSpeed = rSpeed;
            }

            /**
             * Update the robots target angle rate
             */
            @Override
            public void execute() {
                setAngleRate(rSpeed);
            }

            //HACK this is a temporary solution to finish the drive rate command
            //You get updates from the OperatorInterface using the setRotManualDrive method
            @Override
            public boolean isFinished(){
                return !isRotManualDrive;
            }
        }

        /**
         * Command to align to an angle relative to the field
         */
        public class AngleAlignCommand extends Command {
            Rotation2d angle;   /**< Target angle */

            /**
             * Constructor
             * @param angle     Target angle
             */
            public AngleAlignCommand(Rotation2d angle) {
                this.angle = angle;
                addRequirements(RotationDriveCommands.this);
            }

            /**
             * Set the target angle
             * @param angle     Target angle
             */
            public void setAngle(Rotation2d angle) {
                this.angle = angle;
            }

            /**
             * Updates the target angle
             */
            @Override
            public void execute() {
                calcRateToAngle(angle);
            }
        }

        /**
         * Command for aligning the robot to a point on the field
         */
        public class PointAlignCommand extends Command {
            Translation2d point;        /**< Target point */
            Rotation2d rotationOffset;  /**< Offset rotation */

            /**
             * Constructor
             * @param point             Target point
             * @param rotationOffset    Offset rotation
             */
            public PointAlignCommand(Translation2d point, Rotation2d rotationOffset) {
                this.point = point;
                this.rotationOffset = rotationOffset;
                addRequirements(RotationDriveCommands.this);
            }

            /**
             * Set the target point
             * @param point             Target point
             */
            public void setPoint(Translation2d point) {
                this.point = point;
            }

            /**
             * Set the offset rotation
             * @param rotationOffset    Offset rotation
             */
            public void setOffset(Rotation2d rotationOffset) {
                this.rotationOffset = rotationOffset;
            }
            
            /**
             * Update the robot's angle
             */
            @Override
            public void execute() {
                calcRateToPoint(targetPoint, rotationOffset);
            }
        }

        /**
         * Command to align to the center of the reef
         */
        public class ReefAlignCommand extends Command {
            Rotation2d offset;  /**< Offset angle */

            /**
             * Constructor
             * @param offset    Offset angle
             */
            public ReefAlignCommand(Rotation2d offset) {
                this.offset = offset;
                addRequirements(RotationDriveCommands.this);
            }

            /**
             * Set the offset angle
             * @param offset    Offset angle
             */
            public void setOffset(Rotation2d offset) {
                this.offset = offset;
            }

            /**
             * Update the robot's angle
             */
            @Override
            public void execute() {
                reefAngleCalc(offset);
            }

            @Override
            public boolean isFinished(){
                return getReefFace(new Translation2d()).getRotation().minus(getEstimatedPos().getRotation()).getDegrees() <= 2;
            }

            
        }

        public class RotGoToReefCommand extends Command{
            Rotation2d offset;

            public RotGoToReefCommand(Rotation2d offset){
                this.offset = offset;
                addRequirements(RotationDriveCommands.this);
            }

            public void setOffset(Rotation2d offset){
                this.offset = offset;
            }

            @Override
            public void execute(){
                rotGoToReef(offset);
            }

            @Override
            public boolean isFinished(){
                return Math.abs(getReefFace(new Translation2d()).getRotation().minus(getEstimatedPos().getRotation()).getDegrees()) <= 2;
            }
        }

        // public class AutonRotGoToReefCommand extends Command{
        //     Rotation2d offset;

        //     public AutonRotGoToReefCommand(Rotation2d offset){
        //         this.offset = offset;
        //         addRequirements(RotationDriveCommands.this);
        //     }

        //     public void setOffset(Rotation2d offset){
        //         this.offset = offset;
        //     }

        //     @Override
        //     public void initialize(){
        //         PPHolonomicDriveController.overrideRotationFeedback(() -> 
        //             getCalcRateToAngle(
        //                 calcGoToReef(new Pose2d(new Translation2d(), offset)).getRotation())
        //         );
        //     }

        //     @Override
        //     public boolean isFinished(){
        //         return getReefFace(new Translation2d()).getRotation().minus(getEstimatedPos().getRotation()).getDegrees() <= 2;
        //     }

        //     @Override
        //     public void end(boolean interrupt){
        //         PPHolonomicDriveController.clearRotationFeedbackOverride();
        //     }
        // }

        public class RotDoNothingCommand extends Command{
            public RotDoNothingCommand(){
                addRequirements(RotationDriveCommands.this);
            }

            @Override
            public void execute(){
                setAngleRate(0);
            }
        }

        private final RotationRateCommand rotationRateCommand;  /**< Internal rate control command instance */
        private final AngleAlignCommand angleAlignCommand;      /**< Internal angle align command instance */
        private final PointAlignCommand pointAlignCommand;      /**< Internal point align command instance */
        private final ReefAlignCommand reefAlignCommand;        /**< Internal reef align command instance */
        private final RotGoToReefCommand rotGoToReefCommand;
        private final RotDoNothingCommand rotDoNothingCommand;

        /**
         * Constructor
         */
        public RotationDriveCommands() {
            angleAlignCommand = new AngleAlignCommand(new Rotation2d());
            pointAlignCommand = new PointAlignCommand(new Translation2d(0, 0), new Rotation2d());
            rotationRateCommand = new RotationRateCommand(0);
            reefAlignCommand = new ReefAlignCommand(new Rotation2d());
            rotGoToReefCommand = new RotGoToReefCommand(new Rotation2d());
            rotDoNothingCommand = new RotDoNothingCommand();
            setDefaultCommand(rotDoNothingCommand);
        }
    }

    public class AutonReefAlign extends Command{
        Pose2d offset;

        public AutonReefAlign(Pose2d offset){
            this.offset = offset;
            addRequirements(Drive.this);
        }

        public void setOffset(Pose2d offset){
            this.offset = offset;
        }

        @Override
        public void execute(){
            autonCalcGoToReef(offset);
        }

        @Override
        public boolean isFinished(){
            //TODO Uncomment this part as soon as you're done testing
            return Math.abs(getReefFace(new Translation2d()).getRotation().minus(getEstimatedPos().getRotation()).getDegrees()) <= Constants.alignRotTolerance.getDegrees()
              && PhotonUtils.getDistanceToPose(getEstimatedPos(), getReefFace(offset.getTranslation())) <= Constants.alignLinearTolerance;
        }
    }

    public class PresetPoseCommand extends Command{
        Pose2d pose;

        public PresetPoseCommand(Pose2d pose){
            this.pose = pose;
        }

        public void resetPose(Pose2d pose){
            this.pose = pose;
        }

        @Override
        public void initialize(){
            presetPosition(pose);
        }

        @Override 
        public boolean isFinished(){
            return true;
        }
    }

    // Command classes
    public final LinearDriveCommands linearDriveCommands;      /**< Linear motion control subsytem */
    public final RotationDriveCommands rotationDriveCommands;  /**< Rotation motion control subsystem */
    public final PresetPoseCommand presetPoseCommand;

    /**
     * Constructor
     */
    private Drive() {
        // Set swerve drive positions
        frontLeftLocation = new Translation2d((Constants.robotLength / 2 - Constants.wheelInset),
                (Constants.robotWidth / 2 - Constants.wheelInset));
        frontRightLocation = new Translation2d((Constants.robotLength / 2 - Constants.wheelInset),
                -(Constants.robotWidth / 2 - Constants.wheelInset));
        backLeftLocation = new Translation2d(-(Constants.robotLength / 2 - Constants.wheelInset),
                (Constants.robotWidth / 2 - Constants.wheelInset));
        backRightLocation = new Translation2d(-(Constants.robotLength / 2 - Constants.wheelInset),
                -(Constants.robotWidth / 2 - Constants.wheelInset));

        // Initialize Swerve Kinematics
        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        // Create swerve drive module objects
        frontLeft = new Swerve(Constants.frontLeftDriveM, Constants.frontLeftAngleM, "FrontLeft",
                Rotation2d.fromDegrees(0), true, true);
        frontRight = new Swerve(Constants.frontRightDriveM, Constants.frontRightAngleM, "FrontRight",
                Rotation2d.fromDegrees(0), false, true);
        backLeft = new Swerve(Constants.backLeftDriveM, Constants.backLeftAngleM, "BackLeft", Rotation2d.fromDegrees(0),
                true, true);
        backRight = new Swerve(Constants.backRightDriveM, Constants.backRightAngleM, "BackRight",
                Rotation2d.fromDegrees(0), false, true);

        // Initialize NavX
        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
        field2d = new Field2d();
        field2d.getObject("fieldTargetPoint").setPose(targetPoint.getX(), targetPoint.getY(),
                Rotation2d.fromDegrees(0));
        nearestReefFace = new Pose2d();
        field2d.getObject("nearestReefFace").setPose(nearestReefFace);

        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d());

        rSpeed = 0;

        angleAlignPID = new PIDController(Constants.angleAlignPID.kP, Constants.angleAlignPID.kI,
                Constants.angleAlignPID.kD);

        angleAlignPID.enableContinuousInput(-Math.PI, Math.PI);

        driveAlignPID = new PIDController(3, 0, 0);


        // Initialize pose estimation
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        // Setup PathPlanner
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Make instance of Command Classes
        linearDriveCommands = new LinearDriveCommands();
        rotationDriveCommands = new RotationDriveCommands();

        presetPoseCommand = new PresetPoseCommand(new Pose2d());

        autoBuilder = new AutoBuilder();
        AutoBuilder.configure(
                this::getEstimatedPos,
                this::presetPosition,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> pathPlannerKinematics(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(7, 0, 0),
                        new PIDConstants(7, 0, 0)),
                config,
                this::isRedAlliance,
                // linearDriveCommands,
                // rotationDriveCommands,
                this
        );

        // Call method to initialize shuffleboard
        shuffleBoardInit();


        pathConstraints = PathConstraints.unlimitedConstraints(12);
    }

    /**
     * Initialize suffleboard
     */
    public void shuffleBoardInit() {
        //Advantage Scope
        as_swerveModules = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

        as_currentPose = NetworkTableInstance.getDefault()
            .getStructTopic("Current Pose2d", Pose2d.struct).publish();

        // Setup Shuffleboard
        var pose_layout = Shuffleboard.getTab("Drive")
                .getLayout("Drive Pose", BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_posEstX = pose_layout.add("Pose X", swerveDrivePoseEstimator.getEstimatedPosition().getX()).getEntry();
        sb_posEstY = pose_layout.add("Pose Y", swerveDrivePoseEstimator.getEstimatedPosition().getY()).getEntry();
        sb_posEstR = pose_layout
                .add("Pose R", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees()).getEntry();

        sb_speedX = pose_layout.add("Speed X", 0).getEntry();
        sb_speedY = pose_layout.add("Speed Y", 0).getEntry();
        sb_speedR = pose_layout.add("Speed R", 0).getEntry();
        sb_robotTargetAngle = pose_layout.add("Robot Target Angle", 0).getEntry();

        sb_linearCommand = pose_layout.add("Linear Current Command", "---").getEntry();
        sb_rotationCommand = pose_layout.add("Rotation Current Command", "---").getEntry();
        sb_driveCommand = pose_layout.add("Drive Current Command", "---").getEntry();

        sb_speedTargetR = pose_layout.add("Target Speed R", 0).getEntry();

        sb_field2d = Shuffleboard.getTab("Drive").add(field2d).withWidget("Field");
        sb_batteryVoltage = Shuffleboard.getTab("Drive").add("Battery voltage", 0).getEntry();

    }

    /*
     * Sets field relative mode
     * 
     * @param enable true to enable field relative mode, false to disable. (Default:
     * true)
     */
    public void setfieldRelative(boolean enable) {
        this.fieldRelative = enable;
    }

    /**
     * Sets the drivetrain linear speeds in x and y parameters. Direction of
     * the axis used determine by field relative setting.
     * 
     * @param xSpeed speed of along the x-axis
     * @param ySpeed speed of along the y-axis
     */
    public void setSpeed(double xSpeed, double ySpeed) {
        updateKinematics(xSpeed, ySpeed);
    }

    /**
     * Sets the drivetrain linear speeds by setting speed and a heading. Direction
     * of the axis used determine by field relative setting.
     * 
     * @param speed   linear speed of the robot
     * @param heading heading of the robot
     */
    public void setVector(double speed, Rotation2d heading) {
        double xSpeed = Math.cos(heading.getRadians()) * speed;
        double ySpeed = Math.sin(heading.getRadians()) * speed;

        setSpeed(xSpeed, ySpeed);
    }

    /**
     * Sets the angular rate of the robot and sets the robot to AngleRate
     * angle control mode.
     * 
     * @param rSpeed angle rate for the robot
     */
    public void setAngleRate(double rSpeed) {
        this.rSpeed = rSpeed;
    }

    /**
     * Gets the robots estimiated pose
     * 
     * @return estimated robot pose
     */
    public Pose2d getEstimatedPos() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public double getDriveRate(){
        return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
    }

    /**
     * Gets the measured chassis speeds
     * @return  measured chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });
    }

    /**
     * Updates the robot swerve kinematics
     */
    private void updateKinematics(double xSpeed, double ySpeed) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            Pose2d robot_pose = getEstimatedPos();
            Rotation2d fieldAngle = robot_pose.getRotation();
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, fieldAngle);
        } else {
            // TODO Replace deprecated method
            speeds = chassisSpeeds;
        }

        speeds = ChassisSpeeds.discretize(speeds, Constants.updatePeriod);

        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

        sb_speedTargetR.setDouble(rSpeed);
        sb_speedX.setDouble(chassisSpeeds.vxMetersPerSecond);
        sb_speedY.setDouble(chassisSpeeds.vyMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Updates the chassis speeds from path planner
     * @param chassisSpeeds
     */
    private void pathPlannerKinematics(ChassisSpeeds chassisSpeeds) {
        // updateKinematics(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        // setAngleRate(chassisSpeeds.omegaRadiansPerSecond);
        this.chassisSpeeds = chassisSpeeds;
    }


    /**
     * Calculates the angle rate to move to the target angle from the current angle
     * @param targetAngle   target angle
     */
    private void calcRateToAngle(Rotation2d targetAngle) {
        // Get current angle position
        Pose2d pose = getEstimatedPos();
        Rotation2d currentAngle = pose.getRotation();

        calcRateToAngle(targetAngle, currentAngle);
    }

    private double getCalcRateToAngle(Rotation2d targetAngle){
        // Get current angle position
        Rotation2d currentAngle = getEstimatedPos().getRotation();

        double speed = angleAlignPID.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        return speed;
    }

    /**
     * Calculates the angle rate for moving to a target angle from a given angle
     * @param targetAngle   target angle
     * @param currentAngle  current angle
     */
    private void calcRateToAngle(Rotation2d targetAngle, Rotation2d currentAngle) {
        double speed = angleAlignPID.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        this.rSpeed = speed;
    }

    /**
     * Calculates the angle rate to look at a target point
     * 
     * @param point  target point
     * @param offset target orientation offset
     */
    private void calcRateToPoint(Translation2d point, Rotation2d offset) {
        Pose2d pose = getEstimatedPos();
        Translation2d targetOffset = targetPoint.minus(pose.getTranslation());
        Rotation2d targetAngle = targetOffset.getAngle().plus(offset);

        calcRateToAngle(targetAngle, pose.getRotation());
    }

    /**
     * Updates the robot swerve odometry
     */
    private void update_odometry() {
        swerveDrivePoseEstimator.update(navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }


    public void calcToPoint(Translation2d point){
        Pose2d currentPose = getEstimatedPos();

        // Calculate trapezoidal profile
        double maxDriveRate = Constants.maxAutoSpeed;

        //Gets the 2D distance between the target point and current point
        double linearError = point.getDistance(currentPose.getTranslation());

        //Variable that gets a new point from subtracting the targetpoint by the current point
        Translation2d calcPoint = point.minus(currentPose.getTranslation());

        //Calculates the angle Error between the two points using Rotation2d (unit circle)
        Rotation2d angleError = calcPoint.getAngle();

        //Trapezoidal Profiling
        double targetSpeed = maxDriveRate * (linearError > 0 ? 1 : -1);
        double rampDownSpeed = linearError / Constants.alignRampDistance * maxDriveRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed)) targetSpeed = rampDownSpeed;
        
        //Calculates the X and Y Speeds by multiplying the distance between the two points by the angle components
        double xSpeed = angleError.getCos() * targetSpeed;
        double ySpeed = angleError.getSin() * targetSpeed;
        
        updateKinematics(xSpeed, ySpeed);
    }

    public Vector<N2> getCalcToPoint(Translation2d point){
        Pose2d currentPose = getEstimatedPos();

        double maxDriveRate = Constants.maxAutoSpeed;

        //Gets the 2D distance between the target point and current point
        double linearError = point.getDistance(currentPose.getTranslation());

        //Variable that gets a new point from subtracting the targetpoint by the current point
        Translation2d calcPoint = point.minus(currentPose.getTranslation());

        //Calculates the angle Error between the two points using Rotation2d (unit circle)
        Rotation2d angleError = calcPoint.getAngle();

        //Trapezoidal Profiling
        double targetSpeed = maxDriveRate * (linearError > 0 ? 1 : -1);
        double rampDownSpeed = linearError / Constants.alignRampDistance * maxDriveRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed)) targetSpeed = rampDownSpeed;
        
        //Calculates the X and Y Speeds by multiplying the distance between the two points by the angle components
        double xSpeed = angleError.getCos() * targetSpeed;
        double ySpeed = angleError.getSin() * targetSpeed;

        SmartDashboard.putNumber("Align Linear Error", linearError);
        SmartDashboard.putNumber("Align Rotation Error", Math.abs(getReefFace(new Translation2d()).getRotation().minus(getEstimatedPos().getRotation()).getDegrees()));

        return VecBuilder.fill(xSpeed, ySpeed);

        
    }

    // 
    /**
     * Calculates the angle the robot should go at to align with the reef.
     * @param offset    offset angle
     */
    public void reefAngleCalc(Rotation2d offset) {
        Rotation2d targetAngle = FieldLayout.getReefFaceZone(getEstimatedPos());
        calcRateToAngle(targetAngle.plus(offset));
    }

    public void linearGoToReef(Translation2d offset){
        Rotation2d rotOffset = Rotation2d.fromDegrees(0);
        if (isRedAlliance()) {
            offset = new Translation2d(-offset.getX(), -offset.getY());
            rotOffset = Rotation2d.fromDegrees(180);
        }

        Rotation2d reefFaceRotation = FieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = FieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX() + offset.getX(), zeroFace.getY() + offset.getY())
                .rotateAround(FieldLayout.getReef(ReefFace.CENTER).getTranslation(),
                        reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation.rotateBy(rotOffset));
        this.nearestReefFace = finalReefFace;

        calcToPoint(finalReefFace.getTranslation());

    }

    public Pose2d getReefFace(Translation2d offset){
        Rotation2d rotOffset = Rotation2d.fromDegrees(0);
        if (isRedAlliance()) {
            offset = new Translation2d(-offset.getX(), -offset.getY());
            rotOffset = Rotation2d.fromDegrees(180);
        }

        Rotation2d reefFaceRotation = FieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = FieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX() + offset.getX(), zeroFace.getY() + offset.getY())
                .rotateAround(FieldLayout.getReef(ReefFace.CENTER).getTranslation(),
                        reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation.rotateBy(rotOffset));

        return finalReefFace;
    }

    public void rotGoToReef(Rotation2d offset){
        Rotation2d rotOffset = Rotation2d.fromDegrees(0);
        if (isRedAlliance()) {
            rotOffset = Rotation2d.fromDegrees(180);
        }
        Rotation2d reefFaceRotation = FieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = FieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX(), zeroFace.getY())
                .rotateAround(FieldLayout.getReef(ReefFace.CENTER).getTranslation(),
                        reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation.rotateBy(rotOffset));
        calcRateToAngle(finalReefFace.getRotation().plus(offset));
    }

    public Pose2d calcGoToReef(Pose2d offset){
        Rotation2d rotOffset = Rotation2d.fromDegrees(0);
        if (isRedAlliance()) {
            offset = new Pose2d(-offset.getX(), -offset.getY(), offset.getRotation());
            rotOffset = Rotation2d.fromDegrees(180);
        }

        Rotation2d reefFaceRotation = FieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = FieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX() + offset.getX(), zeroFace.getY() + offset.getY())
                .rotateAround(FieldLayout.getReef(ReefFace.CENTER).getTranslation(),
                        reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation.rotateBy(rotOffset));
        
        return finalReefFace;
    }

    public void autonCalcGoToReef(Pose2d offset){
        Rotation2d rotOffset = Rotation2d.fromDegrees(0);
        if (isRedAlliance()) {
            offset = new Pose2d(-offset.getX(), -offset.getY(), offset.getRotation());
            rotOffset = Rotation2d.fromDegrees(180);
        }

        Rotation2d reefFaceRotation = FieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = FieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX() + offset.getX(), zeroFace.getY() + offset.getY())
                .rotateAround(FieldLayout.getReef(ReefFace.CENTER).getTranslation(),
                        reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation.rotateBy(rotOffset));

        Vector<N2> linearRate = getCalcToPoint(finalReefFace.getTranslation());
        double angleRate = getCalcRateToAngle(finalReefFace.getRotation().plus(offset.getRotation()));
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(linearRate.get(0), linearRate.get(1), angleRate), getEstimatedPos().getRotation());
        pathPlannerKinematics(speeds);
        this.nearestReefFace = finalReefFace;
    }

    /**
     * Checks if the robot is on the red alliance
     * @return  true if the current alliance is red, false if blue or not set
     */
    public boolean isRedAlliance() {
        // TODO Move to a dedicated PathPlanner class
        var alliance = DriverStation.getAlliance();
        boolean is_red = false;
        if (alliance.isPresent()) {
            is_red = alliance.get() == DriverStation.Alliance.Red;
        } 

        return is_red;
    }

    public void setLinearManualDrive(boolean isLinearManualDrive){
        this.isLinearManualDrive = isLinearManualDrive;
    }

    public void setRotManualDrive(boolean isRotManualDrive){
        this.isRotManualDrive = isRotManualDrive;
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI() {
        Pose2d pose = getEstimatedPos();
        sb_posEstX.setDouble(pose.getX());
        sb_posEstY.setDouble(pose.getY());
        sb_posEstR.setDouble(pose.getRotation().getDegrees());

        sb_speedR.setDouble(rSpeed);
        field2d.setRobotPose(getEstimatedPos());
        field2d.getObject("fieldTargetPoint").setPose(targetPoint.getX(), targetPoint.getY(),
                Rotation2d.fromDegrees(0));
        field2d.getObject("nearestReefFace").setPose(nearestReefFace);

        sb_batteryVoltage.setDouble(RobotController.getBatteryVoltage());

        var currentCommand = Drive.getInstance().rotationDriveCommands.getCurrentCommand();
        String curCommandName = "null";

        if (currentCommand != null)
            curCommandName = currentCommand.getName();
        sb_rotationCommand.setString(curCommandName);

        currentCommand = Drive.getInstance().linearDriveCommands.getCurrentCommand();
        curCommandName = "null";

        if (currentCommand != null)
            curCommandName = currentCommand.getName();
        sb_linearCommand.setString(curCommandName);

        currentCommand = Drive.getInstance().getCurrentCommand();
        curCommandName = "null";

        if (currentCommand != null)
            curCommandName = currentCommand.getName();
        sb_driveCommand.setString(curCommandName);

        as_swerveModules.set(new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });

        as_currentPose.set(getEstimatedPos());
    }

    /**
     * Adds a new vision pose to the pose estimator
     * @param estPose       estimated pose
     * @param timeStamp     timestamp of the estimated pose
     * @param estStd        estimated pose standard deviation values
     */
    public void addVisionPose(Pose2d estPose, double timeStamp, Vector<N3> estStd) {
        Pose2d poseResult = new Pose2d(estPose.getX(), estPose.getY(), getEstimatedPos().getRotation());
        swerveDrivePoseEstimator.addVisionMeasurement(poseResult, timeStamp, estStd);
    }

    /**
     * Resets the pose estimators current position
     * @param new_pose  new robot position
     */
    public void presetPosition(Pose2d new_pose) {
        swerveDrivePoseEstimator.resetPosition(
                navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                new_pose
        );
    }

    /**
     * Gets the current Auton command from Shuffleboard
     * @return  current auton command from Shuffleboard
     */
    public Command followPath(String pathName) {
        // TODO Move to a dedicated PathPlanner class
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public PathPlannerPath getPath (String pathName){
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return path;
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }
    
    /**
     * Sets the target linear rate
     * @param xSpeed    Target X rate
     * @param ySpeed    Target Y rate
     */
    public void setDriveRate(double xSpeed, double ySpeed) {
        DriveRateCommand driveRate = linearDriveCommands.driveRateCommand;
        driveRate.setSpeeds(xSpeed, ySpeed);
        
        if (linearDriveCommands.getCurrentCommand() != driveRate) driveRate.schedule();
    }

    /**
     * Sets the target angle rate
     * @param rSpeed    Target angle rate
     */
    public void setRotationRate(double rSpeed) {
        RotationRateCommand rotationRate = rotationDriveCommands.rotationRateCommand;
        rotationRate.setRotationRate(rSpeed);

        if (rotationDriveCommands.getCurrentCommand() != rotationRate)
            rotationRate.schedule();
    }

    /**
     * Sets the robot to align to an angle
     * @param targetAngle   target angle
     */
    public void setAngleAlign(Rotation2d targetAngle) {
        AngleAlignCommand rotationCommand = rotationDriveCommands.angleAlignCommand;
        rotationCommand.setAngle(targetAngle);
        if (rotationDriveCommands.getCurrentCommand() != rotationCommand)
            rotationCommand.schedule();
    }

    /**
     * Sets the robot to align to a point
     * @param targetPoint       target point
     * @param rotationOffset    offset angle
     */
    public void setPointAlign(Translation2d targetPoint, Rotation2d rotationOffset) {
        PointAlignCommand pointAlign = rotationDriveCommands.pointAlignCommand;

        pointAlign.setPoint(targetPoint);
        pointAlign.setOffset(rotationOffset);

        if (rotationDriveCommands.getCurrentCommand() != pointAlign)
            pointAlign.schedule();
    }


    // /**
    //  * Sets the robot to follow a PathPlanner path command
    //  * @param path  PathPlanner path command
    //  */
    // public void followPath(Command path) {
    //     if (getCurrentCommand() != path) {
    //         linearDriveCommands.getCurrentCommand().cancel();
    //         rotationDriveCommands.getCurrentCommand().cancel();
    //         path.addRequirements(linearDriveCommands, rotationDriveCommands);
    //         path.schedule();
    //     }
    // }

    /**
     * Aligns to the reef
     * @param offset    offset angle
     */
    public void setReefAlign(Rotation2d offset) {
        ReefAlignCommand reefAlignCommand = rotationDriveCommands.reefAlignCommand;
        reefAlignCommand.setOffset(offset);
        if (rotationDriveCommands.getCurrentCommand() != reefAlignCommand)
            reefAlignCommand.schedule();
    }

    public void setRotGoToReef(Rotation2d offset){
        frc.robot.subsystems.Drive.RotationDriveCommands.RotGoToReefCommand rotGoToReefCommand = rotationDriveCommands.rotGoToReefCommand;
        rotGoToReefCommand.setOffset(offset);
        if(rotGoToReefCommand != rotationDriveCommands.getCurrentCommand()) rotGoToReefCommand.schedule();
    }


    public void setPresetPose(Pose2d pose){
        PresetPoseCommand presetPoseCommand = new PresetPoseCommand(pose);
        presetPoseCommand.schedule();
    }

    // /**
    //  * Generate a PathPlanner path on the floy
    //  */
    // public void pathOnTheFly(Pose2d targetPose, IdealStartingState idealStartingState, GoalEndState goalEndState) {
    //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(getEstimatedPos(), targetPose);
        
    //     PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, idealStartingState, goalEndState);
    //     followPath(AutoBuilder.followPath(path));
    // }

    

    /**
     * Periodic update method
     */
    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) {
            fieldRelative = false;
        } else if (DriverStation.isTeleop()) {
            fieldRelative = true;
        }
        update_odometry();
        updateUI();
    }

    /**
     * Static Initializer
     * 
     * @return Common object of Drive
     */
    public static Drive getInstance() {
        if (drive == null) {
            drive = new Drive();
        }

        return drive;
    }

}