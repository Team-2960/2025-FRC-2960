package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConst;
import frc.robot.simulation.SwerveSim;

public class Swerve extends SubsystemBase {

    /**
     * Main swerve module command
     */
    public class SwerveCommand extends Command {
        /**
         * Updates the swerve outputs
         */
        public void execute() {
            desiredState.optimize(getAnglePos());

            updateDrive(desiredState);
            updateAngle(desiredState);
        }
    }

    public final Translation2d translation;
    public final String name;

    public final SwerveSim sim;

    private final SparkFlex mDrive;
    private final SparkMax mAngle;

    private final RelativeEncoder encDrive;
    private final SparkAbsoluteEncoder encAngle;

    private final SparkFlexSim mDriveSim;
    private final SparkMaxSim mAngleSim;

    private final SparkRelativeEncoderSim encDriveSim;
    private final SparkAbsoluteEncoderSim encAngleSim;

    private final MutVoltage simDriveVoltage;
    private final MutVoltage simAngleVoltage;

    private final PIDController drivePIDcontroller;
    private final SimpleMotorFeedforward driveFeedforward;

    private final PIDController anglePIDController;
    private final SimpleMotorFeedforward angleFeedforward;

    private final MutDistance driveDist;
    private final MutLinearVelocity driveVel;
    private final MutAngularVelocity angleVel;

    private SwerveModuleState desiredState;

    private GenericEntry sb_angleSetPoint;
    private GenericEntry sb_angleCurrent;
    private GenericEntry sb_angleVolt;
    private GenericEntry sb_angleRate;
    private GenericEntry sb_angleError;
    private GenericEntry sb_driveSetPoint;
    private GenericEntry sb_driveCurrent;
    private GenericEntry sb_driveVolt;

    /**
     * Constructor
     * @param driveMotorID  ID of the drive motor SparkFlex
     * @param angleMotorID  ID of the angle motor sparkFlex
     * @param name          Name of the swerve module
     * @param invertDrive   True to invert the drive motor
     * @param invertAngle   True to invert the angle motor
     */
    public Swerve(int driveMotorID, int angleMotorID, String name, Translation2d translation, boolean invertDrive, boolean invertAngle) {
        this.translation = translation;
        this.name = name;

        // Initialize Drive Motor
        mDrive = new SparkFlex(driveMotorID, MotorType.kBrushless);

        SparkFlexConfig drive_config = new SparkFlexConfig();
        drive_config.encoder.positionConversionFactor(SwerveConst.distRatio.in(Meters));
        drive_config.encoder.velocityConversionFactor(SwerveConst.velRatio.in(MetersPerSecond));
        drive_config.inverted(invertDrive);
        mDrive.configure(drive_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Angle Motor
        mAngle = new SparkMax(angleMotorID, MotorType.kBrushless);
        
        SparkMaxConfig angle_config = new SparkMaxConfig();
        angle_config.inverted(invertAngle);
        angle_config.encoder.positionConversionFactor(2 * Math.PI);
        angle_config.encoder.velocityConversionFactor(2 * Math.PI / 60);
        angle_config.apply(new AbsoluteEncoderConfig().inverted(true));
        mAngle.configure(angle_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize drive encoder
        encDrive = mDrive.getEncoder();

        // Initialize Angle Sensor
        encAngle = mAngle.getAbsoluteEncoder();

        // Setup Simulation
        mDriveSim = new SparkFlexSim(mDrive, DCMotor.getNeoVortex(1));
        mAngleSim = new SparkMaxSim(mAngle, DCMotor.getNeo550(1));
        encDriveSim = new SparkRelativeEncoderSim(mDrive);
        encAngleSim = new SparkAbsoluteEncoderSim(mAngle);
        
        simDriveVoltage = Volts.mutable(0);
        simAngleVoltage = Volts.mutable(0);

        sim = new SwerveSim(
            DCMotor.getNeoVortex(1),
            DCMotor.getNeo550(1),
            SwerveConst.wheelDiameter,
            SwerveConst.driveGR,
            SwerveConst.angleGR,
            SwerveConst.driveFF,
            SwerveConst.angleFF,
            SwerveConst.angleMOI
        );

        // Initialize Drive rate controllers
        drivePIDcontroller = new PIDController(
            SwerveConst.drivePID.kP, 
            SwerveConst.drivePID.kI,
            SwerveConst.drivePID.kD
        );

        driveFeedforward = new SimpleMotorFeedforward(
            SwerveConst.driveFF.kS, 
            SwerveConst.driveFF.kV,
            SwerveConst.driveFF.kA
        );

        // Initialize angle position controllers
        anglePIDController = new PIDController(
            SwerveConst.anglePID.kP, 
            SwerveConst.anglePID.kI,
            SwerveConst.anglePID.kD
        );

        angleFeedforward = new SimpleMotorFeedforward(
            SwerveConst.angleFF.kS, 
            SwerveConst.angleFF.kV,
            SwerveConst.angleFF.kA
        );

        // Initialize mutable units
        driveDist = Meters.mutable(0);
        driveVel = MetersPerSecond.mutable(0);
        angleVel = RadiansPerSecond.mutable(0);


        // Initialize desired state
        desiredState = new SwerveModuleState();

        // Setup default command
        setDefaultCommand(new SwerveCommand());

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Drive")
                .getLayout(name + " Swerve", BuiltInLayouts.kList)
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
     * Gets the current swerve module angle
     * 
     * @return current swerve module angle
     */
    public Rotation2d getAnglePos() {
        return Rotation2d.fromRadians(encAngle.getPosition());
    }

    /**
     * Get the current swerve module angle rate
     */
    public AngularVelocity getAngleRate() {
        return angleVel.mut_setMagnitude(encAngle.getVelocity());
    }

    /**
     * Gets the current swerve module drive distance
     * 
     * @return current swerve module drive distance
     */
    public Distance getDrivePos() {
        return driveDist.mut_setMagnitude(encDrive.getPosition());
    }

    /**
     * Gets the current swerve module drive speed
     * 
     * @return current swerve module drive speed
     */
    public LinearVelocity getDriveVelocity() {
        return driveVel.mut_setMagnitude(encDrive.getVelocity());
    }

    /**
     * Gets the current swerve module positions
     * 
     * @return current swerve module positions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), getAnglePos());
    }

    /**
     * Gets the current swerve module state
     * 
     * @return current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAnglePos());
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
    }

    /**
     * Updates the angle position and rate controllers
     */
    private void updateAngle(SwerveModuleState state) {
        // Get current module angle
        Rotation2d encoderRotation = getAnglePos();

        // Calculate target rate
        double error = encoderRotation.getRadians() - state.angle.getRadians();
        double compError = 2 * Math.PI - (Math.abs(error));
        double compareError = Math.min(Math.abs(error), compError);
        double direction = error > 0 ? 1 : -1;

        if (compareError == compError)
            direction *= -1;

        double targetRate = Math.min(1.0, compareError / SwerveConst.angleRampDist.in(Radians)) * SwerveConst.maxAngularSpeed.in(RadiansPerSecond);
        double angleVelocity = targetRate * direction;

        // Calculate motor output
        double pidOutput = anglePIDController.calculate(getAngleRate().in(RadiansPerSecond), angleVelocity);

        double ffOutput = angleFeedforward.calculate(angleVelocity);

        // Set Motor Output
        mAngle.setVoltage(pidOutput + ffOutput);

        sb_angleRate.setDouble(angleVelocity);
        sb_angleError.setDouble(SwerveConst.angleRampDist.in(Radians));

    }

    /**
     * Updated shuffleboard outputs
     */
    private void updateUI() {
        sb_angleSetPoint.setDouble(desiredState.angle.getDegrees());
        sb_angleCurrent.setDouble(getAnglePos().getDegrees());
        sb_angleVolt.setDouble(mAngle.getBusVoltage() * mAngle.getAppliedOutput());
        sb_driveSetPoint.setDouble(desiredState.speedMetersPerSecond);
        sb_driveCurrent.setString(getDriveVelocity().toShortString());
        sb_driveVolt.setDouble(mDrive.getAppliedOutput() * mDrive.getBusVoltage());
    }

    /**
     * Get the simulated applied voltage for the drive motor
     * @return  simulated applied voltage for the drive motor
     */
    public Voltage getSimDriveVoltage() {
        return simDriveVoltage.mut_setMagnitude(mDriveSim.getAppliedOutput() * mDriveSim.getBusVoltage());
    }


    /**
     * Get the simulated applied voltage for the angle motor
     * @return  simulated applied voltage for the angle motor
     */
    public Voltage getSimAngleVoltage() {
        return simAngleVoltage.mut_setMagnitude(mAngleSim.getAppliedOutput() * mAngleSim.getBusVoltage());
    }

    /**
     * Update the current state of the simulation
     * @param driveDist Distance to increment the wheel drive distance
     * @param driveVel  Drive wheel velocity
     * @param angleDist Angle to increment the azimuth angle
     * @param angleVel  Angle azimuth angle rate
     */
    public void setSimState(Distance driveDist, LinearVelocity driveVel, Angle angleDist, AngularVelocity angleVel) {
        encDriveSim.setPosition(encDriveSim.getPosition() + driveDist.in(Meters));
        encDriveSim.setVelocity(driveVel.in(MetersPerSecond));
        encAngleSim.setPosition(encAngleSim.getPosition() + angleDist.in(Radians));
        encAngleSim.setVelocity(angleVel.in(RadiansPerSecond));
    }
}
