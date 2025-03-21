package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.FieldLayout;
import frc.robot.Util.Limits;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class Drive extends SubsystemBase {
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

    private Translation2d targetPoint = new Translation2d();
    private boolean fieldRelative = false;
    private PIDController angleAlignPID;

    private MutDistance mut_dist;
    private MutAngle mut_angle;
    private MutLinearVelocity mut_linVel;
    private MutAngularVelocity mut_angVel;

    private MutAngularVelocity mut_targetAV;

    // Shuffleboard
    private GenericEntry sb_currentCommand;

    private GenericEntry sb_posEstX;
    private GenericEntry sb_posEstY;
    private GenericEntry sb_posEstR;

    private GenericEntry sb_xTarget;
    private GenericEntry sb_yTarget;
    private GenericEntry sb_rTarget;

    private GenericEntry sb_speedX;
    private GenericEntry sb_speedY;
    private GenericEntry sb_speedR;
    
    private GenericEntry sb_speedXTarget;
    private GenericEntry sb_speedYTarget;
    private GenericEntry sb_speedRTarget;

    private ComplexWidget sb_field2d;
    private Pose2d nearestBranch;

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

    /**
     * Command for controlling the robot via rate
     */
    public class RateControlCommand extends Command {
        private Supplier<LinearVelocity> xRate;     /**< X rate supplier */
        private Supplier<LinearVelocity> yRate;     /**< Y rate supplier */
        private Supplier<AngularVelocity> rRate;    /**< angle rate supplier */

        /**
         * Constructor
         * @param xRate     X rate supplier
         * @param yRate     Y rate supplier
         * @param rRate     angle rate suppler
         */
        public RateControlCommand(Supplier<LinearVelocity> xRate, Supplier<LinearVelocity> yRate, Supplier<AngularVelocity> rRate) {
            this.xRate = xRate;
            this.yRate = yRate;
            this.rRate = rRate;
            addRequirements(Drive.this);
        }

        /**
         * Updates the chassis speeds
         */
        @Override
        public void execute() {
            ChassisSpeeds speeds = new ChassisSpeeds(
                xRate.get().in(MetersPerSecond),
                yRate.get().in(MetersPerSecond),
                rRate.get().in(RadiansPerSecond)
            );

            updateKinematics(speeds);
        }
    }

    /**
     * Command for keeping the robot at a specific angle
     */
    public class AngleAlignCommand extends Command {
        private Supplier<LinearVelocity> xRate;     /**< X rate supplier */
        private Supplier<LinearVelocity> yRate;     /**< Y rate supplier */
        private Rotation2d target;                  /**< Target angle */
        private Rotation2d tolerance = null;        /**< Target angle tolerance */

        /**
         * Constructor
         * @param xRate     X rate supplier
         * @param yRate     Y rate supplier
         * @param target    Target Angle
         */
        public AngleAlignCommand(Supplier<LinearVelocity> xRate, Supplier<LinearVelocity> yRate, Rotation2d target) {
            this.xRate = xRate;
            this.yRate = yRate;
            this.target = target;

            addRequirements(Drive.this);
        }

        /**
         * Constructor
         * @param xRate     X rate supplier
         * @param yRate     Y rate supplier
         * @param target    Target Angle
         * @param tolerance Target angle tolerance
         */
        public AngleAlignCommand(Supplier<LinearVelocity> xRate, Supplier<LinearVelocity> yRate, Rotation2d target, Rotation2d tolerance) {
            this(xRate, yRate, target);
            this.tolerance = tolerance;
        }

        /**
         * Calculates the anglular velocity to align to the target angle and updates the chassis speeds
         */
        @Override
        public void execute() {
            AngularVelocity rRate = calcRateToAngle(target);

            ChassisSpeeds speeds = new ChassisSpeeds(
                xRate.get().in(MetersPerSecond),
                yRate.get().in(MetersPerSecond),
                rRate.in(RadiansPerSecond)
            );

            pathPlannerKinematics(speeds);

            // Update UI
            sb_rTarget.setString(mut_angle.mut_replace(target.getDegrees(), Degrees).toShortString());
        }

        /**
         * Finishs the command when the robot is within tolerance of the target 
         * command if a tolerance is set.
         * @return true if at the target angle, false otherwise
         */
        @Override
        public boolean isFinished() {
            boolean result = tolerance != null;

            if(result) result = Limits.inTol(target, tolerance, getPose().getRotation());

            return result;
        }

        /**
         * Clear target angle
         * @param   interrupted     true if command was interrupted
         */
        @Override
        public void end(boolean interruped) {
            sb_rTarget.setString("---");
        }

        /**
         * Sets the target angle
         * @param target    Target angle
         */
        public void setTarget(Rotation2d target) {
            this.target = target;
        }
    }

    /**
     * Command to keep the robot oriented toward a point
     */
    public class AlignToPointCommand extends AngleAlignCommand {
        private Translation2d target;               /**< Target point */
        private Rotation2d offset;                  /**< Angle offset */

        /**
         * Constructor
         * @param xRate     X rate supplier
         * @param yRate     Y rate supplier
         * @param target    Target Angle
         * @param offset    Target angle offset
         */
        public AlignToPointCommand(Supplier<LinearVelocity> xRate, Supplier<LinearVelocity> yRate, Translation2d target, Rotation2d offset) {
            super(xRate, yRate, getPose().getTranslation().minus(target).getAngle());

            this.target = target;
            this.offset = offset;
        }

        /**
         * Constructor
         * @param xRate     X rate supplier
         * @param yRate     Y rate supplier
         * @param target    Target Angle
         * @param offset    Target angle offset
         * @param tolerance Target angle tolerance
         */
        public AlignToPointCommand(Supplier<LinearVelocity> xRate, Supplier<LinearVelocity> yRate, Translation2d target, Rotation2d offset, Rotation2d tolerance) {
            super(xRate, yRate, getPose().getTranslation().minus(target).getAngle().plus(offset), tolerance);

            this.target = target;
            this.offset = offset;
        }

        /**
         * Sets the initial target angle
         */
        @Override
        public void initialize() {
            setTarget(getPose().getTranslation().minus(target).getAngle().plus(offset));
        }

        /**
         * Calcualtes the anglular velocity to align to the target point and updates the chassis speeds
         */
        @Override
        public void execute() {
            setTarget(getPose().getTranslation().minus(target).getAngle().plus(offset));
            super.execute();
        }

        /**
         * Sets the target point
         * @param target    target point
         */
        public void setTarget(Translation2d target) {
            this.target = target;
        }

        /**
         * Sets the offset angle
         * @param offset
         */
        public void setOffset(Rotation2d offset) {
            this.offset = offset;
        }
    }

    /**
     * Command to move to a pose on the field
     */
    public class GotoPoseCommand extends Command {
        private Pose2d target;                  /**< Target Pose */
        private Transform2d offset;             /**< Offset tranform from the robot origin */
        private Distance linearTol = null;      /**< Linear tolerance */
        private Rotation2d angleTol = null;     /**< Angle tolerance */

        /**
         * Constructor
         * @param target    target pose
         * @param offset    offset transform from the robot origin
         */
        public GotoPoseCommand(Pose2d target, Transform2d offset) {
            this.target = target;
            this.offset = offset;

            addRequirements(Drive.this);
        }

        /**
         * Constructor
         * @param target    target pose
         * @param offset    offset transform from the robot origin
         */
        public GotoPoseCommand(Pose2d target, Transform2d offset, Distance linearTol, Rotation2d angleTol) {
            this(target, offset);

            this.linearTol = linearTol;
            this.angleTol = angleTol;

            addRequirements(Drive.this);
        }


        /**
         * Updates the goto pose controller
         */
        @Override
        public void execute() {
            ChassisSpeeds speeds = calcGotoPose(target, offset);
            updateKinematics(speeds);

            // Update UI
            sb_xTarget.setString(mut_dist.mut_replace(target.getX(), Meters).toShortString());
            sb_yTarget.setString(mut_dist.mut_replace(target.getY(), Meters).toShortString());
            sb_rTarget.setString(mut_angle.mut_replace(target.getRotation().getDegrees(), Degrees).toShortString());
        }

        /**
         * Check if the robot is within tolerance of the target pose
         * @return  true if robot is in tolerance of the target pose
         */
        @Override
        public boolean isFinished() {
            boolean result = linearTol != null && angleTol != null;

            if(result) {
                Pose2d currentPose = getPose();

                result = Limits.inTol(target.getRotation(), angleTol, currentPose.getRotation());
                result &= Limits.inTol(target.getTranslation(), linearTol, currentPose.getTranslation());
            }

            return result;
        }

        /**
         * Clear UI
         * @param   interrupted     true if command was interrupted
         */
        @Override
        public void end(boolean interrupted) {
            // Update UI
            sb_xTarget.setString("---");
            sb_yTarget.setString("---");
            sb_rTarget.setString("---");
        }

        /**
         * Sets the target point
         * @param target    target point
         */
        public void setTarget(Pose2d target) {
            this.target = target;
        }

        /**
         * Sets the offset angle
         * @param offset
         */
        public void setOffset(Transform2d offset) {
            this.offset = offset;
        }
    }

    /**
     * Command to align to the nearest reef face
     */
    public class AlignToReefFaceCommand extends AngleAlignCommand {
        private Rotation2d offset;

        /**
         * Constructor
         * @param xRate     X rate supplier
         * @param yRate     Y rate supplier
         */
        public AlignToReefFaceCommand(Supplier<LinearVelocity> xRate, Supplier<LinearVelocity> yRate, Rotation2d offset) {
            super(xRate, yRate, new Rotation2d());
            this.offset = offset;
        }

        /**
         * Sets the initial target angle
         */
        @Override
        public void initialize() {
            setTarget(FieldLayout.getReefFaceZone(getPose()).plus(offset));
        }

        /**
         * Update the target angle and executes motion to target
         */
        @Override
        public void execute() {
            setTarget(FieldLayout.getReefFaceZone(getPose()).plus(offset));
            super.execute();
        }
    }

    /**
     * Command to move to the nearest branch
     */
    public class GotoNearestBranchCommand extends GotoPoseCommand {

        /**
         * Constructor
         * @param offset    offset transform from the robot origin
         */
        public GotoNearestBranchCommand(Transform2d offset) {
            super(new Pose2d(), offset);
        }

        /**
         * Sets the initial target pose
         */
        public void initialize() {
            setTarget(FieldLayout.getNearestBranch(getPose()));
        }

        /**
         * Update the target pose and executes motion to target
         */
        @Override
        public void execute() {
            setTarget(FieldLayout.getNearestBranch(getPose()));
            super.execute();
        }
    }

    /**
     * Sets the current robot pose
     */
    public class PresetPoseCommand extends Command{
        private Pose2d pose;

        /**
         * Constructor
         * @param pose new robot pose
         */
        public PresetPoseCommand(Pose2d pose){
            this.pose = pose;
        }

        /**
         * Sets the current robot pose
         */
        @Override
        public void initialize(){
            presetPosition(pose);
        }

        /**
         * Ends the command after the first cycle
         */
        @Override
        public boolean isFinished(){
            return true;
        }
    }

    /**
     * Sets if the robot is in field relative mode
     */
    public class FieldRelativeCommand extends Command {
        private boolean isFieldRelative;

        /**
         * Constructor
         * @param isFieldRelative   true to set the robot to Field Relative, false otherwise.
         */
        public FieldRelativeCommand(boolean isFieldRelative) {
            this.isFieldRelative = isFieldRelative;
        }

        /**
         * Sets the robot field relative setting
         */
        @Override
        public void initialize() {
            setfieldRelative(isFieldRelative);
        }

        /**
         * Ends the command after the first cycle
         */
        @Override
        public boolean isFinished(){
            return true;
        }
    }

    /**
     * Constructor
     */
    private Drive() {
        // Set swerve drive positions
        // TODO Move to constants
        frontLeftLocation = new Translation2d(
            (Constants.robotLength / 2 - Constants.wheelInset),
            (Constants.robotWidth / 2 - Constants.wheelInset)
        );

        frontRightLocation = new Translation2d(
            (Constants.robotLength / 2 - Constants.wheelInset),
            -(Constants.robotWidth / 2 - Constants.wheelInset)
        );
        
        backLeftLocation = new Translation2d(
            -(Constants.robotLength / 2 - Constants.wheelInset),
            (Constants.robotWidth / 2 - Constants.wheelInset)
        );
        
        backRightLocation = new Translation2d(
            -(Constants.robotLength / 2 - Constants.wheelInset),
            -(Constants.robotWidth / 2 - Constants.wheelInset)
        );

        // Initialize Swerve Kinematics
        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        
        // Create swerve drive module objects
        frontLeft = new Swerve(
            Constants.frontLeftDriveM, 
            Constants.frontLeftAngleM, 
            "FrontLeft",
            Rotation2d.fromDegrees(0), 
            true, 
            true
        );

        frontRight = new Swerve(
            Constants.frontRightDriveM, 
            Constants.frontRightAngleM, 
            "FrontRight",
            Rotation2d.fromDegrees(0), 
            false, 
            true
        );

        backLeft = new Swerve(
            Constants.backLeftDriveM, 
            Constants.backLeftAngleM, 
            "BackLeft", 
            Rotation2d.fromDegrees(0),
            true, 
            true
        );
        
        backRight = new Swerve(
            Constants.backRightDriveM, 
            Constants.backRightAngleM, 
            "BackRight",
            Rotation2d.fromDegrees(0), 
            false, 
            true
        );

        // Initialize NavX
        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();

        // Initialize angle align PID
        angleAlignPID = new PIDController(
            Constants.angleAlignPID.kP, 
            Constants.angleAlignPID.kI,
            Constants.angleAlignPID.kD
        );

        angleAlignPID.enableContinuousInput(-Math.PI, Math.PI);

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
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );

        // Setup Mutable units
        mut_dist = Meters.mutable(0);
        mut_angle = Degrees.mutable(0);
        mut_linVel = MetersPerSecond.mutable(0);
        mut_angVel = DegreesPerSecond.mutable(0);

        mut_targetAV = RadiansPerSecond.mutable(0);

        // Setup PathPlanner

        // TODO move PathPlanner initialization to robot container
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        autoBuilder = new AutoBuilder();
        AutoBuilder.configure(
                this::getPose,
                this::presetPosition,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> pathPlannerKinematics(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0, 0),
                        new PIDConstants(5, 0, 0)),
                config,
                FieldLayout::isRedAlliance,
                this

        );

        pathConstraints = PathConstraints.unlimitedConstraints(12);

        // Initialize Shuffleboard
        shuffleBoardInit();

        // Initialize Advantage Scope
        initAdvantageScope();
    }

    /**
     * Initialize suffleboard
     */
    private void shuffleBoardInit() {
        // Setup Layout
        var pose_layout = Shuffleboard.getTab("Drive")
                .getLayout("Drive Pose", BuiltInLayouts.kList)
                .withSize(1, 4);

        // Setup Current Pose display
        Pose2d estPose = getPose();
        sb_posEstX = pose_layout.add(
            "Pose X", 
            mut_dist.mut_replace(estPose.getX(), Meters).toShortString()
        ).getEntry();
        
        sb_posEstY = pose_layout.add(
            "Pose Y", 
            mut_dist.mut_replace(estPose.getY(), Meters).toShortString()
        ).getEntry();

        sb_posEstR = pose_layout.add(
            "Pose R", 
            mut_angle.mut_replace(estPose.getRotation().getDegrees(), Degrees).toShortString()
        ).getEntry();

        // Setup Target Pose display
        sb_xTarget = pose_layout.add("Target X", "---").getEntry();
        sb_yTarget = pose_layout.add("Target Y", "---").getEntry();
        sb_rTarget = pose_layout.add("Target R", "---").getEntry();

        // Setup Current Speed display
        sb_speedX = pose_layout.add(
            "Speed X", 
            mut_linVel.mut_replace(0, MetersPerSecond).toShortString()
        ).getEntry();

        sb_speedY = pose_layout.add(
            "Speed Y", 
            mut_linVel.mut_replace(0, MetersPerSecond).toShortString()
        ).getEntry();

        sb_speedR = pose_layout.add(
            "Speed R", 
            mut_angVel.mut_replace(0, RadiansPerSecond).toShortString()
        ).getEntry();

        // Setup Target Speed Display
        sb_speedXTarget = pose_layout.add(
            "Target Speed X", 
            mut_linVel.mut_replace(0, MetersPerSecond).toShortString()
        ).getEntry();

        sb_speedYTarget = pose_layout.add(
            "Target Speed Y", 
            mut_linVel.mut_replace(0, MetersPerSecond).toShortString()
        ).getEntry();

        sb_speedRTarget = pose_layout.add(
            "Target Speed R", 
            mut_angVel.mut_replace(0, RadiansPerSecond).toShortString()
        ).getEntry();

        // Setup 2d Field Widget
        field2d = new Field2d();
        field2d.getObject("fieldTargetPoint").setPose(targetPoint.getX(), targetPoint.getY(),
                Rotation2d.fromDegrees(0));
        nearestBranch = new Pose2d();
        field2d.getObject("nearestReefFace").setPose(nearestBranch);
        sb_field2d = Shuffleboard.getTab("Drive").add(field2d).withWidget("Field");

        
    }

    /**
     * Initialize AdvantageScope
     */
    private void initAdvantageScope() {
        odometryPose = NetworkTableInstance.getDefault()
                .getStructTopic("Odometry Pose", Pose2d.struct).publish();
        arrayPose = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Pose Array", Pose2d.struct).publish();
        swerveModules = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Swerve States", SwerveModuleState.struct).publish();
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
     * Gets the robots estimiated pose
     * 
     * @return estimated robot pose
     */
    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
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

    private void updateKinematics(ChassisSpeeds speeds) {
        updateKinematics(speeds, fieldRelative);
    }

    /**
     * Updates the robot swerve kinematics
     */
    private void updateKinematics(ChassisSpeeds speeds, boolean isFieldRelative) {
        if (!isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
        }

        speeds = ChassisSpeeds.discretize(speeds, Constants.updatePeriod);

        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);

        // Update target speed ui
        sb_speedXTarget.setString(mut_linVel.mut_replace(speeds.vxMetersPerSecond, MetersPerSecond).toShortString());
        sb_speedYTarget.setString(mut_linVel.mut_replace(speeds.vyMetersPerSecond, MetersPerSecond).toShortString());
        sb_speedRTarget.setString(mut_angVel.mut_replace(speeds.omegaRadiansPerSecond, RadiansPerSecond).toShortString());
    }

    /**
     * Updates the chassis speeds from path planner
     * @param chassisSpeeds
     */
    private void pathPlannerKinematics(ChassisSpeeds chassisSpeeds) {
        updateKinematics(chassisSpeeds, false);
    }

    /**
     * Calculates the angle rate to move to the target angle from the current angle
     * @param targetAngle   target angle
     */
    private AngularVelocity calcRateToAngle(Rotation2d targetAngle) {
        // Get current angle position
        Pose2d pose = getPose();
        Rotation2d currentAngle = pose.getRotation();

        return calcRateToAngle(targetAngle, currentAngle);
    }

    /**
     * Calculates the angle rate for moving to a target angle from a given angle
     * @param targetAngle   target angle
     * @param currentAngle  current angle
     */
    private AngularVelocity calcRateToAngle(Rotation2d targetAngle, Rotation2d currentAngle) {
        return mut_targetAV.mut_replace(
            angleAlignPID.calculate(currentAngle.getRadians(), targetAngle.getRadians()), 
            RadiansPerSecond
        );
    }

    /**
     * Calculates the lnear rate to move to a target point
     * @param point     target point
     * @param offset    offset from robot center
     */
    public ChassisSpeeds calcGotoPose(Pose2d target, Transform2d offset){
        Pose2d currentPose = getPose().transformBy(offset);
        double error = currentPose.getTranslation().getDistance(target.getTranslation());
        Rotation2d angle = currentPose.getTranslation().minus(target.getTranslation()).getAngle();

        double maxDriveRate = Constants.maxSpeed;
        double targetSpeed = maxDriveRate * (error > 0 ? 1 : -1);
        double rampDownSpeed = error / 1 * maxDriveRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed)) targetSpeed = rampDownSpeed;

        double xRate = targetSpeed * angle.getCos();
        double yRate = targetSpeed * angle.getSin();
        double rRate = calcRateToAngle(target.getRotation()).in(RadiansPerSecond);

        return new ChassisSpeeds(xRate, yRate, rRate);
    }

    /**
     * Updates the robot swerve odometry
     */
    private void updateOdometry() {
        swerveDrivePoseEstimator.update(navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI() {
        // Update Current Command
        String curCommandName = "null";
        var currentCommand = Drive.getInstance().getCurrentCommand();

        if (currentCommand != null)
            curCommandName = currentCommand.getName();
        
        sb_currentCommand.setString(curCommandName);

        // Update current speeds
        ChassisSpeeds currentSpeeds = getChassisSpeeds();

        sb_speedX.setDouble(currentSpeeds.vxMetersPerSecond);
        sb_speedY.setDouble(currentSpeeds.vyMetersPerSecond);
        sb_speedR.setDouble(currentSpeeds.omegaRadiansPerSecond);

        // Update robot pose
        Pose2d pose = getPose();
        sb_posEstX.setDouble(pose.getX());
        sb_posEstY.setDouble(pose.getY());
        sb_posEstR.setDouble(pose.getRotation().getDegrees());

        field2d.setRobotPose(pose);
        field2d.getObject("fieldTargetPoint").setPose(targetPoint.getX(), targetPoint.getY(),
                Rotation2d.fromDegrees(0));
        field2d.getObject("nearestReefFace").setPose(nearestBranch);
    }

    /**
     * Updates advantage scope
     */
    private void updateScope() {
        swerveModules.set(new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        });
    }

    /**
     * Adds a new vision pose to the pose estimator
     * @param estPose       estimated pose
     * @param timeStamp     timestamp of the estimated pose
     * @param estStd        estimated pose standard deviation values
     */
    public void addVisionPose(Pose2d estPose, double timeStamp, Vector<N3> estStd) {
        Pose2d poseResult = new Pose2d(estPose.getX(), estPose.getY(), getPose().getRotation());
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
    public Command getAutonomousCommand() {
        // TODO Move to a dedicated PathPlanner class
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    /**
     * Sets the robot to follow a PathPlanner path command
     * @param path  PathPlanner path command
     */
    public void followPath(Command path) {
        if (getCurrentCommand() != path) {
            path.addRequirements(this);
            path.schedule();
        }
    }

    public void setPresetPose(Pose2d pose){
        PresetPoseCommand presetPoseCommand = new PresetPoseCommand(pose);
        presetPoseCommand.schedule();
    }

    /**
     * Generate a PathPlanner path on the floy
     */
    public void pathOnTheFly() {
        // this.wayPoints = poseList;
        // this.storeWaypoints = PathPlannerPath.waypointsFromPoses(getPose(),
        // new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        Command pathCommand = AutoBuilder.pathfindToPose(new Pose2d(), pathConstraints);
        followPath(pathCommand);
    }

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
        updateOdometry();
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