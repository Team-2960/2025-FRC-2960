package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;
import frc.robot.subsystems.Drive.LinearDriveCommands.DriveRateCommand;
import frc.robot.subsystems.Drive.LinearDriveCommands.GoToPointCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.AngleAlignCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.PointAlignCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.ReefAlignCommand;
import frc.robot.subsystems.Drive.RotationDriveCommands.RotationRateCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.struct.Rotation2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Field;
import java.nio.channels.FileLock;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.opencv.core.Point;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
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

    // Shuffleboard
    private GenericEntry sb_posEstX;
    private GenericEntry sb_posEstY;
    private GenericEntry sb_posEstR;

    private GenericEntry sb_speedX;
    private GenericEntry sb_speedY;
    private GenericEntry sb_speedR;
    private GenericEntry sb_robotTargetAngle;
    private GenericEntry sb_speedTargetR;

    private ComplexWidget sb_field2d;
    private Pose2d nearestReefFace;
    

    private boolean targetSeen;
    private boolean ignoreCamera;

    private Field2d field2d;
    private FieldObject2d fieldTargetPoint;

    // PathPlanner
    public RobotConfig config;
    public AutoBuilder autoBuilder;

    // AdvantageScope
    private SwerveModuleState[] swerveModuleStates;
    private StructPublisher<Pose2d> odometryPose;
    private StructArrayPublisher<Pose2d> arrayPose;
    private StructArrayPublisher<SwerveModuleState> swerveModules;

    PathConstraints pathConstraints;


    

    public class LinearDriveCommands extends SubsystemBase{
        private final DriveRateCommand driveRateCommand;
        private final GoToPointCommand goToPointCommand;
        private static LinearDriveCommands linearCommands = null;

        public LinearDriveCommands(){
            driveRateCommand = new DriveRateCommand(0, 0);
            goToPointCommand = new GoToPointCommand(new Translation2d(0, 0));
            setDefaultCommand(driveRateCommand);
        }

        public class DriveRateCommand extends Command{
            double xSpeed;
            double ySpeed;
        
            public DriveRateCommand(double xSpeed, double ySpeed){
                this.xSpeed = xSpeed;
                this.ySpeed = ySpeed;
                addRequirements(LinearDriveCommands.this);
            } 
    
            public void setSpeeds(double xSpeed, double ySpeed){
                this.xSpeed = xSpeed;
                this.ySpeed = ySpeed;
            }

            @Override
            public void execute(){
                updateKinematics(xSpeed, ySpeed);
            }
        }

        public class GoToPointCommand extends Command{
            Translation2d point;
            
            public GoToPointCommand(Translation2d point){
                this.point = point;
                addRequirements(LinearDriveCommands.this);
            }

            public void setPoint(Translation2d point){
                this.point = point;
            }

            @Override
            public void execute(){
                calcToPoint(point);
            }

        }

        public class LinearGoToReefCommand extends Command{
            Pose2d offset;
    
            public LinearGoToReefCommand(Pose2d offset){
                this.offset = offset;
                addRequirements(LinearDriveCommands.this);
            }
    
            public void goToReef(Pose2d offset){
                this.offset = offset;
            }
    
            @Override
            public void initialize(){
                Drive.this.linearGoToReef(offset);
            }
        }

        public static LinearDriveCommands getInstance() {
            if (linearCommands == null) {
                linearCommands = drive.new LinearDriveCommands();
            }
    
            return linearCommands;
        }
        
    }

    public class RotationDriveCommands extends SubsystemBase{
        private final RotationRateCommand rotationRateCommand;
        private final AngleAlignCommand angleAlignCommand;
        private final PointAlignCommand pointAlignCommand;
        private final ReefAlignCommand reefAlignCommand;
        private static RotationDriveCommands rotationCommands = null;

        public RotationDriveCommands(){
            angleAlignCommand = new AngleAlignCommand(new Rotation2d());
            pointAlignCommand = new PointAlignCommand(new Translation2d(0, 0), new Rotation2d());
            rotationRateCommand = new RotationRateCommand(0);
            reefAlignCommand = new ReefAlignCommand(new Rotation2d());
            setDefaultCommand(rotationRateCommand);
        }

        public class RotationRateCommand extends Command{
            double rSpeed;

            public RotationRateCommand(double rSpeed){
                this.rSpeed = rSpeed;
                addRequirements(RotationDriveCommands.this);
            }

            public void setRotationRate(double rSpeed){
                this.rSpeed = rSpeed;
            }

            @Override
            public void execute(){
                setAngleRate(rSpeed);
            }
        }

        public class AngleAlignCommand extends Command{
            Rotation2d angle;

            public AngleAlignCommand(Rotation2d angle){
                this.angle = angle;
                addRequirements(RotationDriveCommands.this);
            }

            public void setAngle(Rotation2d angle){
                this.angle = angle;
            }

            @Override
            public void execute(){
                calcRateToAngle(angle);
            }
        }

        public class PointAlignCommand extends Command{
            Translation2d point;
            Rotation2d rotationOffset;

            public PointAlignCommand(Translation2d point, Rotation2d rotationOffset){
                this.point = point;
                this.rotationOffset = rotationOffset;
                addRequirements(RotationDriveCommands.this);
            }

            public void setPoint(Translation2d point, Rotation2d rotationOffset){
                this.point = point;
                this.rotationOffset = rotationOffset;
            }

            @Override
            public void execute(){
                calcRateToPoint(targetPoint, rotationOffset);
            }
        }

        public class ReefAlignCommand extends Command{
            Rotation2d offset;

            public ReefAlignCommand(Rotation2d offset){
                this.offset = offset;
                addRequirements(RotationDriveCommands.this);
            }

            public void alignReef(Rotation2d offset){
                this.offset = offset;
            }

            @Override
            public void execute(){
                reefAngleCalc(offset);
            }
        }

        public class RotGoToReefCommand extends Command{
            Pose2d offset;
    
            public RotGoToReefCommand(Pose2d offset){
                this.offset = offset;
                addRequirements(RotationDriveCommands.this);
            }
    
            public void goToReef(Pose2d offset){
                this.offset = offset;
            }
    
            @Override
            public void initialize(){
                Drive.this.rotGoToReef(offset);
            }
        }

        public static RotationDriveCommands getInstance() {
            if (rotationCommands == null) {
                rotationCommands = drive.new RotationDriveCommands();
            }
    
            return rotationCommands;
        }
    }


    //Command classes
    private final LinearDriveCommands linearDriveCommands;
    private final RotationDriveCommands rotationDriveCommands;

    

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
                Rotation2d.fromDegrees(0), true);
        frontRight = new Swerve(Constants.frontRightDriveM, Constants.frontRightAngleM, "FrontRight",
                Rotation2d.fromDegrees(0), false);
        backLeft = new Swerve(Constants.backLeftDriveM, Constants.backLeftAngleM, "BackLeft", Rotation2d.fromDegrees(0),
                true);
        backRight = new Swerve(Constants.backRightDriveM, Constants.backRightAngleM, "BackRight",
                Rotation2d.fromDegrees(0), false);

        // Initialize NavX
        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
        targetSeen = false;
        field2d = new Field2d();
        field2d.getObject("fieldTargetPoint").setPose(targetPoint.getX(), targetPoint.getY(), Rotation2d.fromDegrees(0));
        nearestReefFace = new Pose2d();
        field2d.getObject("nearestReefFace").setPose(nearestReefFace);

        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d());

        rSpeed = 0;

        angleAlignPID = new PIDController(Constants.angleAlignPID.kP, Constants.angleAlignPID.kI,
                Constants.angleAlignPID.kD);
        
        angleAlignPID.enableContinuousInput(-Math.PI, Math.PI);

        driveAlignPID = new PIDController(2, 0, 0);

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
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e){
            e.printStackTrace();
        }

        autoBuilder = new AutoBuilder();
        AutoBuilder.configure(
                this::getEstimatedPos,
                this::presetPosition,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> pathPlannerKinematics(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(1, 0, 0), 
                        new PIDConstants(1, 0, 0)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this

        );

        //Call method to initialize shuffleboard
        shuffleBoardInit();

        //Make instance of Command Classes
        linearDriveCommands = new LinearDriveCommands();
        rotationDriveCommands = new RotationDriveCommands();


        pathConstraints = PathConstraints.unlimitedConstraints(12);
    }

    public void shuffleBoardInit(){
        odometryPose = NetworkTableInstance.getDefault()
                .getStructTopic("Odometry Pose", Pose2d.struct).publish();
        arrayPose = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Pose Array", Pose2d.struct).publish();
        swerveModules = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();

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
        
        sb_speedTargetR = pose_layout.add("Target Speed R", 0).getEntry();

        sb_field2d = Shuffleboard.getTab("Drive").add(field2d).withWidget("Field");
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

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });
    }

    /**
     * Sets a new vision pose update
     * 
     * @param pose      estimated pose from the vision
     * @param timestamp timestamp of when the pose was captured
     */
    public void setVisionPose(Pose2d pose, double timeStamp) {
        // TODO Adjust standard deviations based on distance from target
        if (!ignoreCamera) {
            swerveDrivePoseEstimator.addVisionMeasurement(pose, timeStamp);
            targetSeen = true;
        }
    }

    public boolean getTargetSeen() {
        return targetSeen;
    }

    /**
     * Updates the robot swerve kinematics
     */
    private void updateKinematics(double xSpeed, double ySpeed) {
        ChassisSpeeds speeds;

        // double xSpeed = this.xSpeed;
        // double ySpeed = this.ySpeed;

        double autonRSpeed = chassisSpeeds.omegaRadiansPerSecond;
        /*if (targetSeen && fieldRelative) {
            xSpeed *= -1;
            ySpeed *= -1;
        }*/
        
        if (fieldRelative) {
            Pose2d robot_pose = getEstimatedPos();
            Rotation2d fieldAngle = robot_pose.getRotation();
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, fieldAngle);
        } else {
            PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
            speeds = chassisSpeeds;
        }

        speeds = ChassisSpeeds.discretize(speeds, Constants.updatePeriod);

        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

        sb_speedTargetR.setDouble(rSpeed);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /*
     * private void pathPlannerKinematics(ChassisSpeeds chassisSpeeds){
     * this.xSpeed = chassisSpeeds.vxMetersPerSecond;
     * this.ySpeed = chassisSpeeds.vyMetersPerSecond;
     * ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds,
     * Constants.updatePeriod);
     * SwerveModuleState[] targetStates =
     * kinematics.toSwerveModuleStates(targetSpeeds);
     * SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,
     * Constants.maxSpeed);
     * frontLeft.setDesiredState(targetStates[0]);
     * frontRight.setDesiredState(targetStates[1]);
     * backLeft.setDesiredState(targetStates[2]);
     * backRight.setDesiredState(targetStates[3]);
     * }
     */
    private Optional<Rotation2d> getRotationTargetOverride(){
    // Some condition that should decide if we want to override rotation
        if(angleMode == AngleControlMode.Angle || angleMode == AngleControlMode.LookAtPoint) {
        // Return an optional containing the rotation override (this should be a field relative rotation)
            return Optional.of(new Rotation2d(rSpeed));
        } else {
        // return an empty optional when we don't want to override the path's rotation
            return Optional.empty();
        }
    }
    private void pathPlannerKinematics(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    private void calcRateToAngle(Rotation2d targetAngle) { 
        // Get current angle position
        Pose2d pose = getEstimatedPos();
        Rotation2d currentAngle = pose.getRotation();

        calcRateToAngle(targetAngle, currentAngle);
    }

    private void calcRateToAngle(Rotation2d targetAngle, Rotation2d currentAngle) {
        double speed = angleAlignPID.calculate(currentAngle.getRadians(), targetAngle.getRadians());
        
        this.rSpeed = speed;
    }

    /**
     * Calculates the angle rate to look at a target point
     * @param   point   target point
     * @param   offset  target orientation offset
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
        Transform2d distance = currentPose.minus(new Pose2d(point, Rotation2d.fromDegrees(0)));
        double xSpeed = driveAlignPID.calculate(distance.getX());
        double ySpeed = driveAlignPID.calculate(distance.getY());
        SmartDashboard.putNumber("xSpeed PID", xSpeed);
        SmartDashboard.putNumber("ySpeed PID", ySpeed);

        updateKinematics(xSpeed, ySpeed);
    }

    //Calculates the angle the robot should go at to align with the reef.
    public void reefAngleCalc(Rotation2d offset){
        Rotation2d targetAngle = FieldLayout.getReefFaceZone(getEstimatedPos());
        calcRateToAngle(targetAngle.plus(offset));
    }

    public void goToReef(Pose2d offset){
        if (getAlliance() == DriverStation.Alliance.Red){
            offset = new Pose2d(-offset.getX(), -offset.getY(), offset.getRotation());
        }
        FieldLayout fieldLayout = FieldLayout.getInstance();
        //Pose2d nearestReefFace = fieldLayout.getNearestReefFace(getEstimatedPos());
        Rotation2d reefFaceRotation = fieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = fieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX() + offset.getX(), zeroFace.getY() + offset.getY())
            .rotateAround(fieldLayout.getReef(ReefFace.CENTER).getTranslation(), 
                reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation);
        this.nearestReefFace = finalReefFace;

        setGoToPoint(finalReefFace.getTranslation());
        setAngleAlign(finalReefFace.getRotation().plus(offset.getRotation()));
    }

    public void linearGoToReef(Pose2d offset){
        if (getAlliance() == DriverStation.Alliance.Red){
            offset = new Pose2d(-offset.getX(), -offset.getY(), offset.getRotation());
        }
        FieldLayout fieldLayout = FieldLayout.getInstance();
        //Pose2d nearestReefFace = fieldLayout.getNearestReefFace(getEstimatedPos());
        Rotation2d reefFaceRotation = fieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = fieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX() + offset.getX(), zeroFace.getY() + offset.getY())
            .rotateAround(fieldLayout.getReef(ReefFace.CENTER).getTranslation(), 
                reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation);
        this.nearestReefFace = finalReefFace;

        setGoToPoint(finalReefFace.getTranslation());
    }

    public void rotGoToReef(Pose2d offset){
        if (getAlliance() == DriverStation.Alliance.Red){
            offset = new Pose2d(-offset.getX(), -offset.getY(), offset.getRotation());
        }
        FieldLayout fieldLayout = FieldLayout.getInstance();
        //Pose2d nearestReefFace = fieldLayout.getNearestReefFace(getEstimatedPos());
        Rotation2d reefFaceRotation = fieldLayout.getReefFaceZone(getEstimatedPos());
        Pose2d zeroFace = fieldLayout.getReef(ReefFace.ZERO);
        Translation2d poseOffset = new Translation2d(zeroFace.getX() + offset.getX(), zeroFace.getY() + offset.getY())
            .rotateAround(fieldLayout.getReef(ReefFace.CENTER).getTranslation(), 
                reefFaceRotation);
        Pose2d finalReefFace = new Pose2d(poseOffset, reefFaceRotation);
        this.nearestReefFace = finalReefFace;

        setGoToPoint(finalReefFace.getTranslation());
        setAngleAlign(finalReefFace.getRotation().plus(offset.getRotation()));
    }

    public Alliance getAlliance(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
            if (alliance.get() == DriverStation.Alliance.Red){
                return DriverStation.Alliance.Red;
            }else{
                return DriverStation.Alliance.Blue;
            }
        }else{
            return DriverStation.Alliance.Blue;
        }
    }


    private void updateUI() {
        Pose2d pose = getEstimatedPos();
        sb_posEstX.setDouble(pose.getX());
        sb_posEstY.setDouble(pose.getY());
        sb_posEstR.setDouble(pose.getRotation().getDegrees());

        sb_speedR.setDouble(rSpeed);
        field2d.setRobotPose(getEstimatedPos());
        field2d.getObject("fieldTargetPoint").setPose(targetPoint.getX(), targetPoint.getY(), Rotation2d.fromDegrees(0));
        field2d.getObject("nearestReefFace").setPose(nearestReefFace);
        var rotationCommand = Drive.getInstance().rotationDriveCommands.getCurrentCommand();
        String curRotCommand = "null";
        if (rotationCommand != null) curRotCommand = rotationCommand.getName();
        SmartDashboard.putString("Rotation Command", curRotCommand);

        var linearCommand = linearDriveCommands.getCurrentCommand();
        String curLinearCommand = "null";
        if (linearCommand != null) curLinearCommand = linearCommand.getName();
        
        SmartDashboard.putString("Linear Command", curLinearCommand);

        var curCommand = getCurrentCommand();
        String curCommandName = "null";
        if (curCommand != null) curCommandName = curCommand.getName();
        
        SmartDashboard.putString("Drive Command", curCommandName);
    }

    private void updateScope() {
        swerveModules.set(new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });
    }
    
    //estPos is the estimated pose from the cameras. timeStamp is the time stamp that the April Tag was detected. 
    //estStd is the estimated Standard Deviation from the Camera/ for the camera/
    public void addVisionPose(Pose2d estPose, double timeStamp, Vector<N3> estStd){
        Pose2d poseResult = new Pose2d(estPose.getX(), estPose.getY(), getEstimatedPos().getRotation());
        swerveDrivePoseEstimator.addVisionMeasurement(poseResult, timeStamp, estStd);
    }

    public void ignoreCamera(boolean ignore) {
        this.ignoreCamera = ignore;
    }

    public void presetPosition(Pose2d pose2d) {
        swerveDrivePoseEstimator.resetPosition(
                navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose2d);
        targetSeen = false;
    }

    public Command getAutonomousCommand() {
        try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    
            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
      }

    public void setDriveRate(double xSpeed, double ySpeed){
        DriveRateCommand driveRate = linearDriveCommands.driveRateCommand;
        driveRate.setSpeeds(xSpeed, ySpeed);
        if (linearDriveCommands.getCurrentCommand() != driveRate) driveRate.schedule();
    }

    public void setRotationRate(double rSpeed){
        RotationRateCommand rotationRate = rotationDriveCommands.rotationRateCommand;
        rotationRate.setRotationRate(rSpeed);
        if (rotationDriveCommands.getCurrentCommand() != rotationRate) rotationRate.schedule();
    }

    public void setAngleAlign(Rotation2d targetAngle){
        AngleAlignCommand rotationCommand = rotationDriveCommands.angleAlignCommand;
        rotationCommand.setAngle(targetAngle);
        if (rotationDriveCommands.getCurrentCommand() != rotationCommand) rotationCommand.schedule();
    }

    public void setPointAlign(Translation2d targetPoint, Rotation2d rotationOffset){
        PointAlignCommand pointAlign = rotationDriveCommands.pointAlignCommand;
        pointAlign.setPoint(targetPoint, rotationOffset);
        if (rotationDriveCommands.getCurrentCommand() != pointAlign) pointAlign.schedule();
    }

    public void setGoToPoint(Translation2d targetPoint){
        GoToPointCommand goToPointCommand = linearDriveCommands.goToPointCommand;
        goToPointCommand.setPoint(targetPoint);
        if (linearDriveCommands.getCurrentCommand() != goToPointCommand) goToPointCommand.schedule();
    }


    public void followPath(Command path){
        if (getCurrentCommand() != path){
            linearDriveCommands.getCurrentCommand().cancel();
            rotationDriveCommands.getCurrentCommand().cancel();
            path.addRequirements(LinearDriveCommands.getInstance(), RotationDriveCommands.getInstance());
            path.schedule();
        }
    }

    public void setReefAlign(Rotation2d offset){
        ReefAlignCommand reefAlignCommand = rotationDriveCommands.reefAlignCommand;
        reefAlignCommand.alignReef(offset);
        if (rotationDriveCommands.getCurrentCommand() != reefAlignCommand) reefAlignCommand.schedule();
    }

    public void pathOnTheFly(){
        //this.wayPoints = poseList;
        //this.storeWaypoints = PathPlannerPath.waypointsFromPoses(getEstimatedPos(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        Command pathCommand = AutoBuilder.pathfindToPose(new Pose2d(), pathConstraints);
        followPath(pathCommand);
    }



    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) {
            fieldRelative = false;
        } else if (DriverStation.isTeleop()) {
            fieldRelative = true;
        }
        update_odometry();
        updateUI();
        updateScope();
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
