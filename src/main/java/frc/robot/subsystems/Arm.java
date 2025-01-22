package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Util.FieldLayout;

import java.util.Map;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm arm;

    public enum ArmControlMode {
        MANUAL_VOLT,
        MANUAL_RATE,
        AUTOMATIC
    }

    /**
     * Defines an arm position state
     */
    public class ArmStateValues {
        public Rotation2d targetAngle;
        public Rotation2d angleTol;

        public ArmStateValues(Rotation2d targetAngle) {
            this(targetAngle, Rotation2d.fromDegrees(2));
        }

        public ArmStateValues(Rotation2d targetAngle, Rotation2d angleTol) {
            this.targetAngle = targetAngle;
            this.angleTol = angleTol;
        }
    }

    private SparkFlex armMotor;

    private Encoder quadArmEncoder;

    private DutyCycleEncoder absoluteArmEncoder;

    private DigitalInput brakeModeDisableBtn;

    private PIDController armPID;

    private ArmFeedforward armFF;

    private final ArmStateValues defaultState = new ArmStateValues(Rotation2d.fromDegrees(15));

    private ArmControlMode control_mode;

    private ArmStateValues targetState = defaultState;

    private double manual_volt;
    private double manual_rate;
    private int manual_ext;

    private Map<String, ArmStateValues> armStates = Map.of(
            "home", new ArmStateValues(Rotation2d.fromDegrees(0)),
            "levelTrough", new ArmStateValues(Rotation2d.fromDegrees(0)),
            "levelMid", new ArmStateValues(Rotation2d.fromDegrees(0)),
            "level4", new ArmStateValues(Rotation2d.fromDegrees(0))
        );

    private GenericEntry sb_armMode;
    private GenericEntry sb_anglePosCurrent;
    private GenericEntry sb_anglePosSetPoint;
    private GenericEntry sb_angleRateCurrent;
    private GenericEntry sb_angleRateSetPoint;
    private GenericEntry sb_angleRateError;
    private GenericEntry sb_angleM1Volt;
    private GenericEntry sb_angleTargetVolt;
    private GenericEntry sb_extState;
    private GenericEntry sb_brakeModeDisabled;
    private GenericEntry sb_armClearOfClimber;
    private GenericEntry sb_anglePosRotations;
    private GenericEntry sb_atAngle;
    private GenericEntry sb_atTarget;

    /**
     * Constructor
     */
    private Arm() {
        //unused channels
        

        armMotor = new SparkFlex(Constants.armMotor, MotorType.kBrushless);

        absoluteArmEncoder = new DutyCycleEncoder(Constants.armDCEncoderPort);

        quadArmEncoder = new Encoder(Constants.armQuadEncoderAPort, Constants.armQuadEncoderBPort);
        quadArmEncoder.setDistancePerPulse(Constants.armEncAnglePerRot.getRadians() / Constants.revTBEncCountPerRev);

        brakeModeDisableBtn = new DigitalInput(Constants.armBrakeModeBtn);

        armPID = new PIDController(Constants.armPIDS0.kP, Constants.armPIDS0.kP, Constants.armPIDS0.kP);

        armFF = new ArmFeedforward(Constants.armFFS0.kS, Constants.armFFS0.kG, Constants.armFFS0.kV);

        armPID.enableContinuousInput(-Math.PI, Math.PI);

        //Auton Positions
        // TODO Set abs encoder offset

        // Set control mode
        control_mode = ArmControlMode.MANUAL_VOLT;
        manual_volt = 0;
        manual_rate = 0;
        manual_ext = 0;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Arm", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_armMode = layout.add("Arm Control Mode", control_mode.name()).getEntry();
        sb_anglePosCurrent = layout.add("Angle Position Current", 0).getEntry();
        sb_anglePosSetPoint = layout.add("Angle Position Set Point", 0).getEntry();
        sb_angleRateCurrent = layout.add("Angle Rate Current", 0).getEntry();
        sb_angleRateSetPoint = layout.add("Angle Rate Set Point", 0).getEntry();
        sb_angleRateError = layout.add("Angle Rate Error", 0).getEntry();
        sb_angleM1Volt = layout.add("Angle Motor 1 Voltage", 0).getEntry();
        sb_angleTargetVolt = layout.add("Angle Target Voltage", 0).getEntry();
        sb_extState = layout.add("Ext State", manual_ext).getEntry();
        sb_brakeModeDisabled = layout.add("Brake Mode Disabled", brakeModeDisableBtn.get()).getEntry();
        sb_armClearOfClimber = layout.add("Arm clear of climber", false).getEntry();
        sb_anglePosRotations = layout.add("Arm Encoder Rotations Output", 0).getEntry();
        
        sb_atAngle = layout.add("At Angle", false).getEntry();
        sb_atTarget = layout.add("At Target", false).getEntry();
    }

    /**
     * Gets the current arm angle
     * 
     * @return current arm angle
     */
    public Rotation2d getArmAngle() {
        double angle = Constants.armEncAngleOffset.getDegrees()
                - Rotation2d.fromRotations(absoluteArmEncoder.get()).getDegrees();
        return Rotation2d.fromDegrees(angle);
    }

    /**
     * Gets the current arm angle rate
     * 
     * @return current arm angle rate in radians per second
     */
    public double getArmVelocity() {
        return quadArmEncoder.getRate();
    }

    /**
     * Checks the current extension state
     * 
     * @return current extension state
     */

    /**
     * Sets the arm's output voltage to the motor. Puts the arm into manual
     * voltage mode. If the arm is not in a manual mode already, the extension
     * state is set to its current state.
     * 
     * @param voltage voltage to set to the motor
     */
    public void setArmVolt(double voltage) {
        manual_volt = voltage;

        control_mode = ArmControlMode.MANUAL_VOLT;
    }

    /**
     * Sets the arm's target rate. Puts the arm into manual mode. If the arm is
     * not in manual mode already, the extension state is set to its current
     * state.
     * 
     * @param rate new arm rate
     */
    public void setArmRate(double rate) {
        manual_rate = rate;

        control_mode = ArmControlMode.MANUAL_RATE;
    }

    /**
     * Check if the arm is at its target angle
     * 
     * @return true if the angle are at their target
     */
    public boolean atAngle() {
        Rotation2d currentAngle = getArmAngle();
        Rotation2d targetAngle = targetState.targetAngle;
        Rotation2d angleTol = targetState.angleTol;

        return Math.abs(targetAngle.getDegrees() - currentAngle.getDegrees()) < angleTol.getDegrees();
    }

    /**
     * Check if the arm is at its target angle and extension
     * 
     * @return true if the angle and extension are at their targets
     */
    public boolean atTarget() {
        return atAngle();
    }


    /**
     * Looks up a standard target state
     * 
     * @param Name of the standard state. If an unknown name is supplied,
     *             the state will be set to the home position
     */
    public void setState(String name) {
        setState(getTargetValues(name));
    }

    /**
     * Sets the target state for the arm
     * 
     * @param targetState Current targetState value for the arm
     */
    public void setState(ArmStateValues targetState) {
        this.targetState = targetState;
        control_mode = ArmControlMode.AUTOMATIC;
    }

    public ArmControlMode getControlMode() {
        return control_mode;
    }


    

    /**
     * Looks up standard target values
     */
    private ArmStateValues getTargetValues(String name) {
        ArmStateValues targetState = armStates.get(name);

        if (targetState == null)
            targetState = defaultState;

        return targetState;
    }

    /**
     * Determines the current target arm control rate
     * 
     * @return target arm control rate based on current settings
     */
    private double getTargetArmRate() {
        double targetSpeed = 0;

        switch (control_mode) {
            case AUTOMATIC:
                targetSpeed = calcTrapezoidalRate();
                break;
            case MANUAL_RATE:
                targetSpeed = manual_rate;
                break;
            default:
                targetSpeed = 0;
                break;
        }

        return targetSpeed;
    }

    /**
     * Calculate the trapezoidal control rate for the current arm target position
     * 
     * @return target arm control rate
     */
    private double calcTrapezoidalRate() {
        
        // Calculate trapezoidal profile
        Rotation2d currentAngle = getArmAngle();
        Rotation2d targetAngle = targetState.targetAngle;
        double maxAngleRate = Constants.maxArmAutoSpeed;

        // Keep arm in package
        if (currentAngle.getDegrees() <= Constants.minArmS2Angle.getDegrees()) {

            targetAngle = Constants.minArmS2Angle;

        }

        Rotation2d angleError = targetAngle.minus(currentAngle);

        double targetSpeed = maxAngleRate * (angleError.getRadians() > 0 ? 1 : +-1);
        double rampDownSpeed = angleError.getRadians() / Constants.armRampDownDist.getRadians() * maxAngleRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;

        return targetSpeed;
    }

    /**
     * Updates the control of the arm rate
     * 
     * @param targetSpeed target
     */
    private double getAngleControlVolt(double targetSpeed) {
        double result = this.manual_volt;

        if (this.control_mode != ArmControlMode.MANUAL_VOLT) {
            if (getArmAngle().getDegrees() <= Constants.minArmS2Angle.getDegrees()) {
                targetSpeed = Math.max(0, targetSpeed);
            }

            Rotation2d currentAngle = getArmAngle();
            double angleRate = getArmVelocity();

            armPID.setPID(Constants.armPIDS0.kP, Constants.armPIDS0.kI, Constants.armPIDS0.kD);

            sb_angleRateError.setDouble(angleRate - targetSpeed);

            // Calculate motor voltage output
            double calcPID = armPID.calculate(angleRate, targetSpeed);
            double calcFF = armFF.calculate(currentAngle.getRadians(), targetSpeed);

            result = calcPID + calcFF;
        }

        return result;
    }

    /**
     * Sets the motor voltage for the arm angle control. Manages soft limits as
     * well.
     * 
     * @param voltage desired motor voltage
     */
    private void setMotorVolt(double voltage) {
        // Set Motors
        //VoltageOut settings = new VoltageOut(voltage);
        //settings.EnableFOC = true;
        armMotor.setVoltage(voltage);
    }

    /*
    public void armAutoAlign(){
        Drive drive = Drive.getInstance();
        double distance = Math.abs(FieldLayout.getShootSpeakerPose().getX() - drive.getEstimatedPos().getX()) + 
            ((Math.cos(getArmAngle().minus(Rotation2d.fromDegrees(11)).getRadians()) * Constants.armLength) + 0.2413);
        double height =  FieldLayout.stageHeight - Constants.armHeightOffset - (Math.sin(getArmAngle().minus(Rotation2d.fromDegrees(11)).getRadians()) * Constants.armLength);
        double desiredAngle = 90 - Math.toDegrees(Math.atan2(height, distance));
        control_mode = ArmControlMode.AUTOMATIC;
        if(desiredAngle < 23){
            desiredAngle = 23;
        }else if(desiredAngle > 100 ){
            desiredAngle = 100;
        }
        new Rotation2d();
        ArmStateValues targetState = new ArmStateValues(Rotation2d.fromDegrees(desiredAngle));
        setState(targetState); 
    }
    */

    public ArmStateValues getState(){
        return targetState;
    }

    /**
     * Updates the brake mode control of the
     */
    private void updateBrakeMode() {
        //var motorConfigs = new MotorOutputConfigs();
        var motorConfigs = new SparkFlexConfig();
        
        SparkBaseConfig config = motorConfigs.idleMode(!brakeModeDisableBtn.get() ? IdleMode.kCoast : IdleMode.kBrake);
        armMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI(double targetRate, double targetVolt) {
        sb_armMode.setString(control_mode.name());
        sb_anglePosCurrent.setDouble(getArmAngle().getDegrees());
        sb_anglePosSetPoint.setDouble(targetState.targetAngle.getDegrees());
        sb_angleRateCurrent.setDouble(getArmVelocity());
        sb_angleRateSetPoint.setDouble(targetRate);
        sb_angleM1Volt.setDouble(armMotor.getBusVoltage());
        sb_angleTargetVolt.setDouble(targetVolt);
        sb_extState.setInteger(manual_ext);
        sb_brakeModeDisabled.setBoolean(!brakeModeDisableBtn.get());
        sb_anglePosRotations.setDouble(absoluteArmEncoder.get());
        sb_atAngle.setBoolean(atAngle());
        sb_atTarget.setBoolean(atTarget());
    }
    
    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic() {
        double targetArmRate = getTargetArmRate();
        double voltage = getAngleControlVolt(targetArmRate);
        updateBrakeMode();
        setMotorVolt(voltage);
        updateUI(targetArmRate, voltage);
    }
    
    /**
     * Static initializer for the arm class
     */
    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

}
