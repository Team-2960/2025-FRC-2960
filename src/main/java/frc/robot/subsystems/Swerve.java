package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Rotation2dStruct;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.AngleUnit.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Swerve extends SubsystemBase {

    private final SparkFlex mDrive;

    private final SparkMax mAngle;

    private final SparkAbsoluteEncoder encAngle;
    private final RelativeEncoder encDrive;

    private final PIDController drivePIDcontroller;
    private final SimpleMotorFeedforward driveFeedforward;

    private final PIDController anglePIDController;
    private final SimpleMotorFeedforward angleFeedforward;

    private SwerveModuleState desiredState;

    private GenericEntry sb_angleSetPoint;
    private GenericEntry sb_angleCurrent;
    private GenericEntry sb_angleVolt;
    private GenericEntry sb_angleRate;
    private GenericEntry sb_angleError;
    private GenericEntry sb_driveSetPoint;
    private GenericEntry sb_driveCurrent;
    private GenericEntry sb_driveVolt;

    private Rotation2d swerveAngleOffset;

    public Swerve(int driveMotorID, int angleMotorID, String swerveName, Rotation2d swerveOffset, boolean invertDrive, boolean invertAngle) {
        // Initialize Drive Motor
        mDrive = new SparkFlex(driveMotorID, MotorType.kBrushless);

        SparkFlexConfig drive_config = new SparkFlexConfig();
        drive_config
            .inverted(invertDrive)
            .smartCurrentLimit(Constants.driveCurrentLimit);
        mDrive.configure(drive_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        swerveAngleOffset = swerveOffset;

        // Initialize Angle Motor
        mAngle = new SparkMax(angleMotorID, MotorType.kBrushless);
        
        SparkMaxConfig angle_config = new SparkMaxConfig();
        angle_config.inverted(invertAngle);
        angle_config.apply(new AbsoluteEncoderConfig().inverted(true));
        mAngle.configure(angle_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize drive encoder
        encDrive = mDrive.getEncoder();

        // Initialize Angle Sensor
        encAngle = mAngle.getAbsoluteEncoder();
    

        // Initialize Drive rate controllers
        drivePIDcontroller = new PIDController(Constants.drivePID.kP, Constants.drivePID.kI,
                Constants.drivePID.kD);
        driveFeedforward = new SimpleMotorFeedforward(Constants.driveFF.kS, Constants.driveFF.kV,
                Constants.driveFF.kA);

        // Initialize angle position controllers
        anglePIDController = new PIDController(Constants.driveAngPID.kP, Constants.driveAngPID.kI,
                Constants.driveAngPID.kD);
        angleFeedforward = new SimpleMotorFeedforward(Constants.driveAngFF.kS, Constants.driveAngFF.kV,
                Constants.driveAngFF.kA);

        // Initialize desired state
        desiredState = new SwerveModuleState();

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Drive")
                .getLayout(swerveName + " Swerve", BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_angleSetPoint = layout.add("Angle Desired", 0).getEntry();
        sb_angleCurrent = layout.add("Angle Current", 0).getEntry();
        sb_angleVolt = layout.add("Angle Voltage", 0).getEntry();
        sb_angleRate = layout.add("Angle Rate", 0).getEntry();
        sb_angleError = layout.add("Angle Error", 0).getEntry();

        sb_driveSetPoint = layout.add("Drive Desired", 0).getEntry();
        sb_driveCurrent = layout.add("Drive Current", 0).getEntry();
        sb_driveVolt = layout.add("Drive Voltage", 0).getEntry();
    }

    
    /**
     * Get current swerve module angle in Angle unit.
     * @return returns Angle unit of module absolute encoder pos
     */
    public Angle getAnglePos(){
        return Rotations.of(encAngle.getPosition());
    }

    /**
     * Gets the current swerve module angle
     * 
     * @return current swerve module angle as Rotation2d
     */
    public Rotation2d getAngleRotation2dPos() {
        return Rotation2d.fromRotations(encAngle.getPosition());
    }

    /**
     * Get swerve module angle rate
     * @return Angular Velocity unit of absolute encoder on module
     */
    public AngularVelocity getAngleRate(){
        return RotationsPerSecond.of(encAngle.getVelocity()/60);
    }

    //TODO retune values for the correct rate. Used to return angle rate in RPM inste

    /**
     * Gets the current swerve module drive distance
     * 
     * @return current swerve module drive distance
     */
    public Distance getDrivePos() {
        return Meters.of(encDrive.getPosition() * Constants.driveRatio);
    }

    /**
     * Gets the current swerve module drive speed
     * 
     * @return current swerve module drive speed
     */
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(encDrive.getVelocity()/60 * Constants.driveRatio);
    }

    /**
     * Gets the current swerve module positions
     * 
     * @return current swerve module positions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos().in(Meters), getAngleRotation2dPos());
    }

    /**
     * Gets the current swerve module state
     * 
     * @return current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngleRotation2dPos());
    }

    /**
     * Sets the desired module state
     * 
     * @param desiredState desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }



    /**
     * Subsystem period update
     */
    @Override
    public void periodic() {
        desiredState.optimize(getAngleRotation2dPos());

        updateDrive(desiredState);
        updateAngle(desiredState);
        updateUI();
    }

    /**
     * Updates the drive rate controllers
     */
    private void updateDrive(SwerveModuleState state) {
        // Calculate the drive output from the drive PID controller.
        double pidOutput = drivePIDcontroller.calculate(getDriveVelocity().in(MetersPerSecond),
                state.speedMetersPerSecond);

        double ffOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

        mDrive.setVoltage(pidOutput + ffOutput);
        //mDrive.setVoltage(0);//TODO revove after drive PID fixed
        
    }

    /**
     * Updates the angle position and rate controllers
     */
    private void updateAngle(SwerveModuleState state) {
        // Get current module angle
        Angle encoderRotation = getAnglePos();

        // Calculate target rate
        Angle error = encoderRotation.minus(state.angle.getMeasure());
        double compError = 2 * Math.PI - (Math.abs(error.in(Radians)));
        double compareError = Math.min(Math.abs(error.in(Radians)), compError);
        double direction = error.in(Radians) > 0 ? 1 : -1;

        if (compareError == compError)
            direction *= -1;

        double targetRate = Math.min(1.0, compareError / Constants.swerveAngleRampDist.getRadians()) * Constants.maxSwerveAngularSpeed;
        double angleVelocity = targetRate * direction;

        // Calculate motor output
        double pidOutput = anglePIDController.calculate(getAngleRate().in(RadiansPerSecond), angleVelocity);

        double ffOutput = angleFeedforward.calculate(angleVelocity);

        // Set Motor Output
        mAngle.setVoltage(pidOutput + ffOutput);

        sb_angleRate.setDouble(angleVelocity);
        sb_angleError.setDouble(Constants.swerveAngleRampDist.getRadians());

    }

    /**
     * Updated shuffleboard outputs
     */
    private void updateUI() {
        sb_angleSetPoint.setDouble(desiredState.angle.getDegrees());
        sb_angleCurrent.setDouble(getAnglePos().in(Degrees));
        sb_angleVolt.setDouble(mAngle.getBusVoltage() * mAngle.getAppliedOutput());
        sb_driveSetPoint.setDouble(desiredState.speedMetersPerSecond);
        sb_driveCurrent.setDouble(getDriveVelocity().in(MetersPerSecond));
        sb_driveVolt.setDouble(mDrive.getAppliedOutput() * mDrive.getBusVoltage());
    }
}
