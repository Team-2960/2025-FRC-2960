package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Climber extends SubsystemBase {

    public enum ClimberStates {
        MATCH_START,
        CLIMB_START,
        CLIMB,
        IDLE
    }

    private static Climber climber = null;

    private SparkMax winchL;
    private SparkMax winchR;
    private SparkBaseConfig winchConfig;
    private LimitSwitchConfig winchLimitConfig;

    private SparkLimitSwitch winchLimit;
    private RelativeEncoder winchEncoder;
    private EncoderConfig winchEncoderConfig;

    private DoubleSolenoid ratchetRelease;
    private Timer ratchetTimer;

    private ClimberStates climbState = ClimberStates.IDLE;

    private GenericEntry sb_state;
    private GenericEntry sb_isDown;
    private GenericEntry sb_isClearOfArm;
    private GenericEntry sb_mLVoltage;
    private GenericEntry sb_mRVoltage;
    private GenericEntry sb_winchExt;
    private GenericEntry sb_ratchetTime;
    private GenericEntry sb_ratchetValve;

    /**
     * Constructor
     */
    private Climber() {
        // Initialize Motors
        winchL = new SparkMax(Constants.winchMotorL, MotorType.kBrushless);
        winchR = new SparkMax(Constants.winchMotorR, MotorType.kBrushless);
        
        //Initialize Configurations
        winchConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        //winchEncoderConfig = new EncoderConfig().positionConversionFactor(Constants.winchRatchedDelay);
        winchLimitConfig = new LimitSwitchConfig().forwardLimitSwitchType(Type.kNormallyOpen).forwardLimitSwitchEnabled(false);

        //Configure the winch motors, limit switch, encoder
        winchL.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        winchR.configure(winchConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Encoder
        winchEncoder = winchR.getEncoder();

        //Initialize Limit Switch
        winchLimit = winchL.getForwardLimitSwitch();

        // Initialize climber state
        climbState = ClimberStates.MATCH_START;

        // Initialize Ratchet Release
        ratchetRelease = new DoubleSolenoid(Constants.phCANID, PneumaticsModuleType.REVPH,
                Constants.climbRatchetRev, Constants.climbRatchetFor);
        ratchetRelease.set(DoubleSolenoid.Value.kReverse);

        ratchetTimer = new Timer();

        // Setup Shuffleboard

        var layout = Shuffleboard.getTab("Status")
                .getLayout("Climber", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_state = layout.add("State", climbState.name()).getEntry();
        sb_isDown = layout.add("Is Down", false).getEntry();
        sb_isClearOfArm = layout.add("Is Clear of Arm", false).getEntry();
        sb_mLVoltage = layout.add("Left Motor Voltage", 0).getEntry();
        sb_mRVoltage = layout.add("Right Motor Voltage", 0).getEntry();
        sb_winchExt = layout.add("Winch extension", 0).getEntry();
        sb_ratchetTime = layout.add("Ratchet timer", 0).getEntry();
        sb_ratchetValve = layout.add("Ratchet Valve", "").getEntry();
    }

    /**
     * Sets the current climber state
     * 
     * @param climbState new climber state
     */
    public void setClimbState(ClimberStates climbState) {
        this.climbState = climbState;
    }

    /**
     * Gets the current set state for the climber
     * 
     * @return current set climber state
     */
    public ClimberStates getClimbState() {
        return climbState;
    }

    /**
     * Gets the current extension distance
     * 
     * @return distance the climber is extended
     */
    public double getExtension() {
        return -winchEncoder.getPosition();
    }

    /**
     * Checks if the climber is hitting the limit sensor
     * 
     * @return true if the climber is hitting the limit sensor, false otherwise.
     */
    public boolean isDown() {
        return winchLimit.isPressed();
    }

    /**
     * Resets the climber encoder
     */
    public void resetClimber() {
        winchEncoder.setPosition(0);
    }

    /**
     * Checks if the climber is in a position that is clear of the arm
     * 
     * @return true if the climber is in a position clear of the arm
     */
    public boolean isClearOfArm() {
        double armContactHeight = 15;
        return isDown() || getExtension() < armContactHeight;
    }

    public void setRatchet(boolean enabled) {
        if (enabled) {
            ratchetRelease.set(Value.kForward);
        } else {
            ratchetRelease.set(Value.kReverse);
        }
    }

    /**
     * Climber periodic update
     */
    @Override
    public void periodic() {
        switch (climbState) {
            case MATCH_START:
                retractClimber(.2, false);
                if (isDown())
                    setClimbState(ClimberStates.IDLE);
                break;
            case CLIMB:
                retractClimber(1, true);
                break;
            case CLIMB_START:
                extendClimber();
                break;
            case IDLE:
            default:
                setMotor(0, true);
        }

        // Reset the climber encoder if the limit switch is set
        if (isDown())
            resetClimber();

        updateUI();
    }

    /**
     * Retracts the climber until the limit sensor is tripped
     */
    private void retractClimber(double winchSpeed, boolean useSoftStop) {

        if (isDown()) {
            setMotor(0, true);
        } else {
            setMotor(winchSpeed, true);

        }
    }

    /**
     * Extends the climber to the max height
     */
    private void extendClimber() {
        // Check if the arm is clear of the climber
        if (!Arm.getInstance().isInClimberZone()) {
            // Check if the climber is at its max extention
            if (getExtension() < Constants.winchMaxExtension) {
                setMotor(-.5, true);
            } else {
                setMotor(0, true);
            }
        }
    }

    private void setMotor(double value, boolean enableLimit) {
        winchLimitConfig.forwardLimitSwitchEnabled(enableLimit);
        winchL.set(value);
        winchR.set(value);

    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        sb_state.setString(climbState.name());
        sb_isDown.setBoolean(isDown());
        sb_isClearOfArm.setBoolean(isClearOfArm());
        sb_mLVoltage.setDouble(winchL.getBusVoltage() * winchL.getAppliedOutput());
        sb_mRVoltage.setDouble(winchR.getBusVoltage() * winchR.getAppliedOutput());
        sb_winchExt.setDouble(getExtension());
        sb_ratchetTime.setDouble(ratchetTimer.get());
        sb_ratchetValve.setString(ratchetRelease.get().name());
    }

    /**
     * Static initializer
     */
    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber();
        }

        return climber;
    }
}
