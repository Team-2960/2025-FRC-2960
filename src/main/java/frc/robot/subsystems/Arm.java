package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;

import java.util.Map;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj2.command.Command;
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

    private TalonFX armMotor1;
    private TalonFX armMotor2;

    private Encoder quadArmEncoder;

    private DutyCycleEncoder absoluteArmEncoder;

    private DigitalInput brakeModeDisableBtn;

    private PIDController armPID;

    private ArmFeedforward armFFS0;

    private final ArmStateValues defaultState = new ArmStateValues(Rotation2d.fromDegrees(15));

    private ArmControlMode control_mode;

    private ArmStateValues targetState = defaultState;

    private double armVolt;
    private double armRate;

    private Map<String, ArmStateValues> armStates = Map.of(
            "Match Start", new ArmStateValues(Rotation2d.fromDegrees(60)),
            "Home", defaultState,
            "Intake", new ArmStateValues(Rotation2d.fromDegrees(7)),
            "Speaker", new ArmStateValues(Rotation2d.fromDegrees(46)),
            "lineSpeaker", new ArmStateValues(Rotation2d.fromDegrees(56)),
            "longShot", new ArmStateValues(Rotation2d.fromDegrees(67.5)),
            "Amp", new ArmStateValues(Rotation2d.fromDegrees(102)),
            "Climb", new ArmStateValues(Rotation2d.fromDegrees(97.38)),
            "AmpSideShoot", new ArmStateValues(Rotation2d.fromDegrees(47)),
            "home", new ArmStateValues(Rotation2d.fromDegrees(23))
            //"Climb Balance", new ArmStateValues(Rotation2d.fromDegrees(97.38), 0),
            //"Trap Score", new ArmStateValues(Rotation2d.fromDegrees(70), 2)
        );

    private GenericEntry sb_armMode;
    private GenericEntry sb_anglePosCurrent;
    private GenericEntry sb_anglePosSetPoint;
    private GenericEntry sb_angleRateCurrent;
    private GenericEntry sb_angleRateSetPoint;
    private GenericEntry sb_angleRateError;
    private GenericEntry sb_angleM1Volt;
    private GenericEntry sb_angleM2Volt;
    private GenericEntry sb_angleTargetVolt;
    private GenericEntry sb_brakeModeDisabled;
    private GenericEntry sb_anglePosRotations;
    private GenericEntry sb_atAngle;
    private GenericEntry sb_atTarget;
    private GenericEntry sb_currentArmCommand;

    public class ArmVoltageCommand extends Command{
        private double targetVoltage;

        public ArmVoltageCommand(double targetVoltage){
            this.targetVoltage = targetVoltage;

            addRequirements(Arm.this);
        }

        @Override
        public void execute(){
            setMotorVolt(targetVoltage);
        }

        public void setVoltage(double targetVoltage){
            this.targetVoltage = targetVoltage;
        }

    }

    public class ArmRateCommand extends Command{
        private double targetRate;
        private double tolerance;

        public ArmRateCommand(double targetRate, double tolerance){
            this.targetRate = targetRate;
            this.tolerance = tolerance;
            addRequirements(Arm.this);
        }

        public void setRate(double targetRate){
            this.targetRate = targetRate;
        }

        public void setToleranceRate(double targetRate, double tolerance){
            SmartDashboard.putNumber("Arm tolerance", targetRate);
            this.targetRate = targetRate;
            this.tolerance = tolerance;
        }
        
        @Override
        public void execute(){
            setArmRate(targetRate);
        }

        @Override
        public boolean isFinished(){
            return targetRate == 0 || Math.abs(targetRate) <= tolerance;
        }
    }

    public class ArmHoldCommand extends Command{
        private Rotation2d target;

        public ArmHoldCommand(){
            target = new Rotation2d();
            addRequirements(Arm.this);
        }

        @Override
        public void initialize(){
            System.out.println("Command initialized");
            target = getArmAngle();
        }

        @Override
        public void execute(){
            setArmAngle(target);
        }
    }

    public class ArmAngleCommand extends Command{
        private Rotation2d armAngle;
        
        public ArmAngleCommand(Rotation2d armAngle){
            this.armAngle = armAngle;
            addRequirements(Arm.this);
        }

        public void setAngle(Rotation2d armAngle){
            this.armAngle = armAngle;
        }

        @Override
        public void execute(){
            setArmAngle(armAngle);
        }
        
        @Override
        public boolean isFinished(){
            return atAngle(armAngle, Rotation2d.fromDegrees(2));
        }
    }

    public class ArmBrakeModeCommand extends Command{
        
    }


    private final ArmVoltageCommand armVoltageCommand;
    private final ArmRateCommand armRateCommand;
    private final ArmAngleCommand armAngleCommand;
    private final ArmHoldCommand armHoldCommand;

    /**
     * Constructor
     */
    private Arm() {        

        armMotor1 = new TalonFX(Constants.armMotor1);
        armMotor2 = new TalonFX(Constants.armMotor2);

        absoluteArmEncoder = new DutyCycleEncoder(Constants.armDCEncoderPort);

        quadArmEncoder = new Encoder(Constants.armQuadEncoderAPort, Constants.armQuadEncoderBPort);
        quadArmEncoder.setDistancePerPulse(Constants.armEncAnglePerRot.getRadians() / Constants.revTBEncCountPerRev);

        brakeModeDisableBtn = new DigitalInput(Constants.armBrakeModeBtn);

        armPID = new PIDController(Constants.armPIDS0.kP, Constants.armPIDS0.kP, Constants.armPIDS0.kP);

        armFFS0 = new ArmFeedforward(Constants.armFFS0.kS, Constants.armFFS0.kG, Constants.armFFS0.kV);

        //Auton Positions
        // TODO Set abs encoder offset

        // Set control mode
        control_mode = ArmControlMode.MANUAL_VOLT;
        armVolt = 0;
        armRate = 0;

        // Set target state to current state
        targetState = new ArmStateValues(getArmAngle());

        // Initialize Timer        
        shuffleBoardInit();

        armVoltageCommand = new ArmVoltageCommand(0);
        armRateCommand = new ArmRateCommand(0, 0);
        armAngleCommand = new ArmAngleCommand(Rotation2d.fromDegrees(0));
        armHoldCommand = new ArmHoldCommand();

        setDefaultCommand(new ArmHoldCommand());
    }

    public void shuffleBoardInit(){
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
        sb_angleM2Volt = layout.add("Angle Motor 2 Voltage", 0).getEntry();
        sb_angleTargetVolt = layout.add("Angle Target Voltage", 0).getEntry();
        sb_brakeModeDisabled = layout.add("Brake Mode Disabled", brakeModeDisableBtn.get()).getEntry();
        sb_anglePosRotations = layout.add("Arm Encoder Rotations Output", 0).getEntry();
        sb_atAngle = layout.add("At Angle", false).getEntry();
        sb_atTarget = layout.add("At Target", false).getEntry();
        //sb_currentArmCommand = layout.add("Current Arm Command", getCurrentCommand().getName()).getEntry();

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
     * Check if the arm is at its target angle
     * 
     * @return true if the angle are at their target
     */
    public boolean atAngle(Rotation2d targetAngle, Rotation2d angleTol) {
        Rotation2d currentAngle = getArmAngle();

        return Math.abs(targetAngle.getDegrees() - currentAngle.getDegrees()) < angleTol.getDegrees();
    }

    /**
     * Check if the arm is at its target angle and extension
     * 
     * @return true if the angle and extension are at their targets
     */
    public boolean atTarget(ArmStateValues armState) {
        return atAngle(armState.targetAngle, armState.angleTol);
    }



    /**
     * Determines the current target arm control rate
     * 
     * @return target arm control rate based on current settings
     */

    /* 
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
    */

    /**
     * Calculate the trapezoidal control rate for the current arm target position
     * 
     * @return target arm control rate
     */
    private void setArmAngle(Rotation2d targetAngle) {
        
        // Calculate trapezoidal profile
        Rotation2d currentAngle = getArmAngle();
        double maxAngleRate = Constants.maxArmAutoSpeed;
        Rotation2d angleError = targetAngle.minus(currentAngle);

        double targetSpeed = maxAngleRate * (angleError.getRadians() > 0 ? 1 : +-1);
        double rampDownSpeed = angleError.getRadians() / Constants.armRampDownDist.getRadians() * maxAngleRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;
        
        setArmRate(targetSpeed);
    }

    private void setArmState(ArmStateValues armState){
        Rotation2d targetAngle = armState.targetAngle;

        setArmAngle(targetAngle);
    }

    /**
     * Updates the control of the arm rate
     * 
     * @param targetSpeed target
     */
    private void setArmRate(double targetSpeed) {
        double result = this.armRate;

        Rotation2d currentAngle = getArmAngle();
        double angleRate = getArmVelocity();

        ArmFeedforward armFF = armFFS0;

        armPID.setPID(Constants.armPIDS0.kP, Constants.armPIDS0.kI, Constants.armPIDS0.kD);
        armFF = armFFS0;
            

        sb_angleRateError.setDouble(angleRate - targetSpeed);

        // Calculate motor voltage output
        double calcPID = armPID.calculate(angleRate, targetSpeed);
        double calcFF = armFF.calculate(currentAngle.getRadians(), targetSpeed);

        result = calcPID + calcFF;
        
        setMotorVolt(result);
        
        //Shuffleboard display
        this.armRate = result;
    }

    /**
     * Sets the motor voltage for the arm angle control. Manages soft limits as
     * well.
     * 
     * @param voltage desired motor voltage
     */
    private void setMotorVolt(double voltage) {
        // Set soft limits
        if (absoluteArmEncoder.get() < Constants.upperEncLimit) {
            voltage = Math.min(0, voltage);
        }
        // Set Motors
        VoltageOut settings = new VoltageOut(voltage);
        settings.EnableFOC = true;
        armMotor1.setControl(settings);
        armMotor2.setControl(settings);
        
        //Shuffleboard display
        this.armVolt = settings.Output;
        
    }


    //TODO Adjust for 2025 Field and uses
    public void armAutoAlign(){
        Drive drive = Drive.getInstance();
        double distance = Math.abs(FieldLayout.getReef(ReefFace.CENTER).getX() - drive.getEstimatedPos().getX()) + 
            ((Math.cos(getArmAngle().minus(Rotation2d.fromDegrees(11)).getRadians()) * Constants.armLength) + 0.2413);
        double height =  /*change this value ->*/0 - Constants.armHeightOffset - (Math.sin(getArmAngle().minus(Rotation2d.fromDegrees(11)).getRadians()) * Constants.armLength);
        double desiredAngle = 90 - Math.toDegrees(Math.atan2(height, distance));
        control_mode = ArmControlMode.AUTOMATIC;
        if(desiredAngle < 23){
            desiredAngle = 23;
        }else if(desiredAngle > 100 ){
            desiredAngle = 100;
        }
        new Rotation2d();
        ArmStateValues targetState = new ArmStateValues(Rotation2d.fromDegrees(desiredAngle));
        setArmState(targetState);
    }


    public ArmStateValues getState(){
        return targetState;
    }

    /**
     * Updates the brake mode control of the
     */
    private void updateBrakeMode() {
        // var motorConfigs = new MotorOutputConfigs();

        // motorConfigs.NeutralMode = !brakeModeDisableBtn.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake;

        // armMotor1.getConfigurator().apply(motorConfigs);
        // armMotor2.getConfigurator().apply(motorConfigs);
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
        sb_angleM1Volt.setDouble(armMotor1.getMotorVoltage().getValueAsDouble());
        sb_angleM2Volt.setDouble(armMotor2.getMotorVoltage().getValueAsDouble());
        sb_angleTargetVolt.setDouble(targetVolt);
        sb_brakeModeDisabled.setBoolean(!brakeModeDisableBtn.get());
        sb_anglePosRotations.setDouble(absoluteArmEncoder.get());
        //sb_currentArmCommand.setString(getCurrentCommand().getName());
    }

    public void setRateCommand(double rate){
        armRateCommand.setRate(rate);
        if(getCurrentCommand() != armRateCommand) armRateCommand.schedule();
    }

    public void setTolRateCommand(double rate, double tolerance){
        armRateCommand.setToleranceRate(rate, tolerance);
        if(getCurrentCommand() != armRateCommand) armRateCommand.schedule();
    }

    public void setAngleCommand(Rotation2d angle){
        armAngleCommand.setAngle(angle);
        if(getCurrentCommand() != armAngleCommand) armAngleCommand.schedule();
    }

    public void setStateCommand(String stateName){
        ArmStateValues state = armStates.get(stateName);
        setAngleCommand(state.targetAngle);
    }

    public void setHoldCommand(){
        if(getCurrentCommand() != armHoldCommand) new ArmHoldCommand().schedule();
    }

    public Command getArmCommand(){
        return getCurrentCommand();
    }
    
    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic() {
        updateUI(armRate, armVolt);
        var currentCommand = getCurrentCommand();
        String curCommandName = "null";
        if (currentCommand != null) curCommandName = currentCommand.getName();
        
        SmartDashboard.putString("Current Command", curCommandName);
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
