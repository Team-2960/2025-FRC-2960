package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Arm.SoftLimCheckCommand.SoftLimDirection;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import static edu.wpi.first.units.Units.*;

import java.util.prefs.BackingStoreException;

public class Arm extends SubsystemBase {
    private static Arm arm = null;

    private SparkFlex motor;

    private RelativeEncoder relEncoder;

    private SparkAbsoluteEncoder absEncoder;

    private PIDController armPID;

    private ArmFeedforward armFF;

    private double armVolt;
    private double armRate;

    private GenericEntry sb_armCommand;
    private GenericEntry sb_anglePosCurrent;
    private GenericEntry sb_anglePosSetPoint;
    private GenericEntry sb_angleRateCurrent;
    private GenericEntry sb_angleRateSetPoint;
    private GenericEntry sb_angleRateError;
    private GenericEntry sb_angleVolt;
    private GenericEntry sb_angleTargetVolt;
    private GenericEntry sb_angleAtTopLim;
    private GenericEntry sb_angleAtBotLim;

    //System Identification
    private final SysIdRoutine sysIdRoutine;
    private final MutVoltage appliedVoltage;
    private final MutCurrent appliedCurrent;
    private final MutAngle angle;
    private final MutAngularVelocity angularVelocity;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    /**
     * Voltage Control Command
     */
    public class ArmVoltageCommand extends Command{
        private double targetVoltage;   /** Target Voltage */
        
        /**
         * Constructor
         * @param targetVoltage     Initial Target Voltage
         */
        public ArmVoltageCommand(double targetVoltage){
            this.targetVoltage = targetVoltage;

            addRequirements(Arm.this);
        }

        /**
         * Sets the motor voltage
         */
        @Override
        public void execute(){
            setMotorVolt(targetVoltage);
        }

        /**
         * Sets the target voltage
         * @param targetVoltage     new target voltage
         */
        public void setVoltage(double targetVoltage){
            this.targetVoltage = targetVoltage;
        }

    }

    //Command to send values to the PID + Feed Forward
    //This Controls the rate aka velocity of the arm, NOT The position
    public class ArmRateCommand extends Command{
        private double targetRate;
        private double tolerance;

        public ArmRateCommand(double targetRate, double tolerance){
            this.targetRate = targetRate;
            this.tolerance = tolerance;
            
            //Make Arm Subsystem required for this command
            addRequirements(Arm.this);
        }

        //Just sets rate
        public void setRate(double targetRate){
            this.targetRate = targetRate;
        }

        //Sets both rate and tolerance
        public void setToleranceRate(double targetRate, double tolerance){
            this.targetRate = targetRate;
            this.tolerance = tolerance;
        }
        
        //What actually sets the rate/ execute the action to set the rate
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

    public class SoftLimCheckCommand extends Command{
        public enum SoftLimDirection{
            UP,
            DOWN
        }

        SoftLimDirection softLimDirection;
        boolean isReached;

        public SoftLimCheckCommand(SoftLimDirection softLimDirection){
            this.softLimDirection = softLimDirection;
            isReached = false;
        }

        public void setDirection(SoftLimDirection softLimDirection){
            this.softLimDirection = softLimDirection;
        }

        @Override
        public void execute(){
            if(softLimDirection == SoftLimDirection.UP){
                isReached = topLimitReached();

            }else if(softLimDirection == SoftLimDirection.DOWN){
                isReached = botLimitReached();
            }
        }

        @Override
        public boolean isFinished(){
            return isReached;
        }
    }


    private final ArmVoltageCommand armVoltageCommand;
    private final ArmRateCommand armRateCommand;
    private final ArmAngleCommand armAngleCommand;
    private final ArmHoldCommand armHoldCommand;

    /**
     * Constructor
     */
    private Arm() {        
        motor = new SparkFlex(Constants.armMotor, MotorType.kBrushless);
        motor.configure(new SparkFlexConfig().inverted(true).apply(new AbsoluteEncoderConfig().zeroCentered(true)), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        absEncoder = motor.getAbsoluteEncoder();
        relEncoder = motor.getExternalEncoder();


        armPID = new PIDController(Constants.armPID.kP, Constants.armPID.kP, Constants.armPID.kP);

        armFF = new ArmFeedforward(Constants.armFF.kS, Constants.armFF.kG, Constants.armFF.kV);

        // Set control mode
        armVolt = 0;
        armRate = 0;

        //Motor Config
        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.absoluteEncoder
            .zeroCentered(true);

        armConfig.encoder
            .velocityConversionFactor(1/60 * 360 / 100);
        // Initialize Timer        
        shuffleBoardInit();

        armVoltageCommand = new ArmVoltageCommand(0);
        armRateCommand = new ArmRateCommand(0, 0);
        armAngleCommand = new ArmAngleCommand(Rotation2d.fromDegrees(0));
        armHoldCommand = new ArmHoldCommand();

        setDefaultCommand(armHoldCommand);

        //System Identification
        sysIdRoutine = new SysIdRoutine(
            new Config(
                Volts.per(Second).of(.25),
                Volts.of(3),
                Seconds.of(10)
            ), 
            new Mechanism(
                this::setMotorVolt,
                this::sysIDLogging, this)
        );

        appliedVoltage = Volts.mutable(0);
        appliedCurrent = Amps.mutable(0);
        angle = Degrees.mutable(Degrees.toBaseUnits(0));
        angularVelocity =  DegreesPerSecond.mutable(0);
        
        sysIdCommandUpQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandGroup =  new SequentialCommandGroup(
            new ParallelRaceGroup(
                sysIdCommandDownQuasi,
                new SoftLimCheckCommand(SoftLimDirection.DOWN)
            ),
            new ParallelRaceGroup(
                sysIdCommandUpQuasi,
                new SoftLimCheckCommand(SoftLimDirection.UP)
            ),
            new ParallelRaceGroup(
                sysIdCommandDownDyn,
                new SoftLimCheckCommand(SoftLimDirection.DOWN)
            ),
            new ParallelRaceGroup(
                sysIdCommandUpDyn,
                new SoftLimCheckCommand(SoftLimDirection.UP)
            )
        );
    }

    public void shuffleBoardInit(){
        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Arm", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_armCommand = layout.add("Arm Command", "").getEntry();
        sb_anglePosCurrent = layout.add("Angle Position Current", 0).getEntry();
        sb_anglePosSetPoint = layout.add("Angle Position Set Point", 0).getEntry();
        sb_angleRateCurrent = layout.add("Angle Rate Current", 0).getEntry();
        sb_angleRateSetPoint = layout.add("Angle Rate Set Point", 0).getEntry();
        sb_angleRateError = layout.add("Angle Rate Error", 0).getEntry();
        sb_angleVolt = layout.add("Angle Motor Voltage", 0).getEntry();
        sb_angleTargetVolt = layout.add("Angle Target Voltage", 0).getEntry();
        sb_angleAtTopLim = layout.add("Top Limit Angle", false).getEntry();
        sb_angleAtBotLim = layout.add("Bottom Limit Angle", false).getEntry();
    }

    /**
     * Gets the current arm angle
     * 
     * @return current arm angle
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromRotations(absEncoder.getPosition());
    }

    public double getVoltage(){
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    /**
     * Gets the current][\
     * \] arm angle rate
     * 
     * @return current arm angle rate in radians per second
     */
    public double getArmVelocity() {
        return relEncoder.getVelocity();
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
     * Calculate the trapezoidal control rate for the current arm target position
     * 
     * @return target arm control rate
     */
    private void setArmAngle(Rotation2d targetAngle) {
        
        // Calculate trapezoidal profile
        Rotation2d currentAngle = getArmAngle();
        double maxAngleRate = Constants.maxArmAutoSpeed;
        Rotation2d angleError = targetAngle.minus(currentAngle);

        double targetSpeed = maxAngleRate * (angleError.getDegrees() > 0 ? 1 : -1);
        double rampDownSpeed = angleError.getDegrees() / Constants.armRampDownDist.getDegrees() * maxAngleRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;
        
        setArmRate(targetSpeed);
    }

    /**
     * Updates the control of the arm rate
     * 
     * @param targetSpeed target
     */
    private void setArmRate(double targetSpeed) {
        Rotation2d currentAngle = getArmAngle();
        double angleRate = getArmVelocity();            

        sb_angleRateError.setDouble(angleRate - targetSpeed);

        // Calculate motor voltage output
        double calcPID = armPID.calculate(angleRate, targetSpeed);
        double calcFF = armFF.calculate(currentAngle.getRadians(), targetSpeed);

        double result = calcPID + calcFF;
        
        setMotorVolt(result);
        
        //Shuffleboard display
        this.armRate = targetSpeed;
    }

    /**
     * Sets the motor voltage for the arm angle control. Manages soft limits as
     * well.
     * 
     * @param voltage desired motor voltage
     */
    //TODO Change to use Spark Flex
    private void setMotorVolt(double voltage) {
        if (topLimitReached() && voltage > 0){
            voltage = 0;
        }
        if (botLimitReached() && voltage < 0){
            voltage = 0;
        }
        motor.setVoltage(voltage);
        
        //Shuffleboard display
        this.armVolt = voltage;
        
    }

    private void setMotorVolt(Voltage voltage){
        double voltNum = voltage.baseUnitMagnitude();
        if (topLimitReached() && voltNum > 0){
            voltNum = 0;
        }
        if (botLimitReached() && voltNum < 0){
            voltNum = 0;
        }
        
        motor.setVoltage(voltNum);

        //Shuffleboard display
        this.armVolt = voltNum;
    }

    public boolean topLimitReached(){
        return getArmAngle().getDegrees() >= Constants.armTopLim.getDegrees();
    }

    public boolean botLimitReached(){
        return getArmAngle().getDegrees() <= Constants.armBotLim.getDegrees();
    }

    private void sysIDLogging(SysIdRoutineLog log){
        double velocity = getArmVelocity();
        System.out.println(velocity);
        log.motor("Arm Angle")
            .voltage(appliedVoltage.mut_replace(getVoltage(), Volts))
            .current(appliedCurrent.mut_replace(getCurrent(), Amps))
            .angularPosition(angle.mut_replace(getArmAngle().getDegrees(), Degrees))
            .angularVelocity(angularVelocity.mut_replace(velocity, DegreesPerSecond));
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI(double targetRate, double targetVolt) {
        Command currentCmd = getCurrentCommand();
        String currentCmdName = "<null>";

        if(currentCmd != null) currentCmdName = currentCmd.getName();

        sb_armCommand.setString(currentCmdName);
        sb_anglePosCurrent.setDouble(getArmAngle().getDegrees());
        sb_anglePosSetPoint.setDouble(armAngleCommand.armAngle.getDegrees());
        sb_angleRateCurrent.setDouble(getArmVelocity());
        sb_angleRateSetPoint.setDouble(targetRate);
        sb_angleVolt.setDouble(motor.getAppliedOutput() * motor.getBusVoltage());
        sb_angleTargetVolt.setDouble(targetVolt);
        sb_angleAtTopLim.setBoolean(topLimitReached());
        sb_angleAtBotLim.setBoolean(botLimitReached());
    }

    public void setVoltCommand(double voltage) {
        armVoltageCommand.setVoltage(voltage);
        if(getCurrentCommand() != armVoltageCommand) armVoltageCommand.schedule();
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

    public void setHoldCommand(){
        if(getCurrentCommand() != armHoldCommand) new ArmHoldCommand().schedule();
    }
    
    public void stopCommands() {
        Command currentCmd = getCurrentCommand();
        if(currentCmd != null) currentCmd.cancel();
    }

    public void setSysIdCommandGroup(){
        if (getCurrentCommand() != sysIdCommandGroup) sysIdCommandGroup.schedule();
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
