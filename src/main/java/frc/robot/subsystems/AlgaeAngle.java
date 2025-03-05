package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator.SoftLimCheckCommand;
import frc.robot.subsystems.Elevator.SoftLimCheckCommand.SoftLimDirection;

import org.opencv.core.Mat;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.*;


public class AlgaeAngle extends SubsystemBase {
    private static AlgaeAngle instance = null;

    private SparkMax motor;

    private RelativeEncoder relEncoder;

    private SparkAbsoluteEncoder absEncoder;

    private PIDController pid;
    private ArmFeedforward ff;

    private double voltage;
    private double rate;

    private GenericEntry sb_Command;
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
    public class VoltageCommand extends Command{
        private double targetVoltage;   /** Target Voltage */
        
        /**
         * Constructor
         * @param targetVoltage     Initial Target Voltage
         */
        public VoltageCommand(double targetVoltage){
            this.targetVoltage = targetVoltage;

            addRequirements(AlgaeAngle.this);
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
    //This Controls the rate aka velocity of the , NOT The position
    public class RateCommand extends Command{
        private double targetRate;
        private double tolerance;

        public RateCommand(double targetRate, double tolerance){
            this.targetRate = targetRate;
            this.tolerance = tolerance;
            
            //Make  Subsystem required for this command
            addRequirements(AlgaeAngle.this);
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
            AlgaeAngle.this.setRate(targetRate);;
        }

        @Override
        public boolean isFinished(){
            return targetRate == 0 || Math.abs(targetRate) <= tolerance;
        }
    }

    public class HoldCommand extends Command{
        private Rotation2d target;

        public HoldCommand(){
            target = new Rotation2d();
            addRequirements(AlgaeAngle.this);
        }

        @Override
        public void initialize(){
            target = getAngle();
        }

        @Override
        public void execute(){
            setAngle(target);
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

    public class AngleCommand extends Command{
        private Rotation2d Angle;
        
        public AngleCommand(Rotation2d Angle){
            this.Angle = Angle;
            addRequirements(AlgaeAngle.this);
        }

        public void setAngle(Rotation2d Angle){
            this.Angle = Angle;
        }

        @Override
        public void execute(){
            AlgaeAngle.this.setAngle(Angle);
        }
        
        @Override
        public boolean isFinished(){
            return atAngle(Angle, Rotation2d.fromDegrees(2));
        }
    }


    private final VoltageCommand VoltageCommand;
    private final RateCommand RateCommand;
    private final AngleCommand AngleCommand;
    private final HoldCommand HoldCommand;

    /**
     * Constructor
     */
    private AlgaeAngle() {        
        motor = new SparkMax(Constants.algaeAngleMotor, MotorType.kBrushless);
        absEncoder = motor.getAbsoluteEncoder();
        relEncoder = motor.getEncoder();

        pid = new PIDController(Constants.algaeAnglePID.kP, Constants.algaeAnglePID.kP, Constants.algaeAnglePID.kP);

        ff = new ArmFeedforward(Constants.algaeAngleFF.kS, Constants.algaeAngleFF.kG, Constants.algaeAngleFF.kV);

        // Set control mode
        voltage = 0;
        rate = 0;

        // Initialize Timer        
        shuffleBoardInit();

        VoltageCommand = new VoltageCommand(0);
        RateCommand = new RateCommand(0, 0);
        AngleCommand = new AngleCommand(new Rotation2d());
        HoldCommand = new HoldCommand();

        setDefaultCommand(HoldCommand);

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
        angle = Degrees.mutable(0);
        angularVelocity =  DegreesPerSecond.mutable(0);
        
        sysIdCommandUpQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandGroup =  new SequentialCommandGroup(
            new ParallelRaceGroup(
                sysIdCommandDownQuasi,
                new SoftLimCheckCommand(frc.robot.subsystems.AlgaeAngle.SoftLimCheckCommand.SoftLimDirection.DOWN)
            ),
            new ParallelRaceGroup(
                sysIdCommandUpQuasi,
                new SoftLimCheckCommand(frc.robot.subsystems.AlgaeAngle.SoftLimCheckCommand.SoftLimDirection.UP)
            ),
            new ParallelRaceGroup(
                sysIdCommandDownDyn,
                new SoftLimCheckCommand(frc.robot.subsystems.AlgaeAngle.SoftLimCheckCommand.SoftLimDirection.DOWN)
            ),
            new ParallelRaceGroup(
                sysIdCommandUpDyn,
                new SoftLimCheckCommand(frc.robot.subsystems.AlgaeAngle.SoftLimCheckCommand.SoftLimDirection.UP)
            )
        );
    }

    public void shuffleBoardInit(){
        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Algae Angle", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_Command = layout.add(" Command", "").getEntry();
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
     * Gets the current  angle
     * 
     * @return current  angle
     */
    public Rotation2d getAngle() {
        double rotations = absEncoder.getPosition();
        if (rotations >= 0.5){
            rotations = 1.0 - rotations;
        }else{
            rotations = -rotations;
        }
        return Rotation2d.fromRotations(rotations);
    }

    /**
     * Gets the current  angle rate
     * 
     * @return current  angle rate in degrees per second
     */
    public double getVelocity() {
        //return -absEncoder.getVelocity()/60 * 360;
        return relEncoder.getVelocity()/60 * 360 * 22.0/(42.0 * 25.0);
    }

    public double getVoltage(){
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }


    /**
     * Check if the  is at its target angle
     * 
     * @return true if the angle are at their target
     */
    public boolean atAngle(Rotation2d targetAngle, Rotation2d angleTol) {
        Rotation2d currentAngle = getAngle();

        return Math.abs(targetAngle.minus(currentAngle).getDegrees()) < angleTol.getDegrees();
    }

    /**
     * Calculate the trapezoidal control rate for the current  target position
     * 
     * @return target  control rate
     */
    private void setAngle(Rotation2d targetAngle) {
        
        // Calculate trapezoidal profile
        Rotation2d currentAngle = getAngle();
        double maxAngleRate = Constants.maxAlgaeAutoSpeed/7;
        Rotation2d angleError = targetAngle.minus(currentAngle);

        double targetSpeed = maxAngleRate * (angleError.getRadians() > 0 ? 1 : -1);
        double rampDownSpeed = angleError.getRadians() / Constants.algaeRampDownDist.getRadians() * maxAngleRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;
        setRate(targetSpeed);
    }

    /**
     * Updates the control of the  rate
     * 
     * @param targetSpeed target
     */
    private void setRate(double targetSpeed) {
        double result = this.rate;

        Rotation2d currentAngle = getAngle();
        double angleRate = getVelocity();            

        sb_angleRateError.setDouble(angleRate - targetSpeed);

        // Calculate motor voltage output
        double calcPID = pid.calculate(angleRate, targetSpeed);
        double calcFF = ff.calculate(currentAngle.getRadians(), targetSpeed);

        result = calcPID + calcFF;

        setMotorVolt(result);
        
        //Shuffleboard display
        this.rate = result;
    }

    /**
     * Sets the motor voltage for the  angle control. Manages soft limits as
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
        this.voltage = voltage;
        
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
        this.voltage = voltNum;
    }

    public boolean topLimitReached(){
        return getAngle().getDegrees() >= Constants.algaeTopLim.getDegrees();
    }

    public boolean botLimitReached(){
        return getAngle().getDegrees() <= Constants.algaeBotLim.getDegrees();

    }

    private void sysIDLogging(SysIdRoutineLog log){
        double velocity = getVelocity();
        System.out.println(velocity);
        log.motor("Algae Angle")
            .voltage(appliedVoltage.mut_replace(getVoltage(), Volts))
            .current(appliedCurrent.mut_replace(getCurrent(), Amps))
            .angularPosition(angle.mut_replace(getAngle().getDegrees(), Degrees))
            .angularVelocity(angularVelocity.mut_replace(velocity, DegreesPerSecond));
    }


    /**
     * Updates shuffleboard
     */
    private void updateUI(double targetRate, double targetVolt) {
        Command currentCmd = getCurrentCommand();
        String currentCmdName = "<null>";

        if(currentCmd != null) currentCmdName = currentCmd.getName();

        sb_Command.setString(currentCmdName);
        sb_anglePosCurrent.setDouble(getAngle().getDegrees());
        sb_anglePosSetPoint.setDouble(AngleCommand.Angle.getDegrees());
        sb_angleRateCurrent.setDouble(getVelocity());
        sb_angleRateSetPoint.setDouble(targetRate);
        sb_angleVolt.setDouble(motor.getAppliedOutput() * motor.getBusVoltage());
        sb_angleTargetVolt.setDouble(targetVolt);
        sb_angleAtTopLim.setBoolean(topLimitReached());
        sb_angleAtBotLim.setBoolean(botLimitReached());
    }

    public void setVoltCommand(double voltage) {
        VoltageCommand.setVoltage(voltage);
        if(getCurrentCommand() != VoltageCommand) VoltageCommand.schedule();
    }

    public void setRateCommand(double rate){
        RateCommand.setRate(rate);
        if(getCurrentCommand() != RateCommand) RateCommand.schedule();
    }

    public void setTolRateCommand(double rate, double tolerance){
        RateCommand.setToleranceRate(rate, tolerance);
        if(getCurrentCommand() != RateCommand) RateCommand.schedule();
    }

    public void setAngleCommand(Rotation2d angle){
        AngleCommand.setAngle(angle);
        if(getCurrentCommand() != AngleCommand) AngleCommand.schedule();
    }

    public void setHoldCommand(){
        if(getCurrentCommand() != HoldCommand) new HoldCommand().schedule();
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
        updateUI(rate, voltage);
    }


    /**
     * Static initializer for the  class
     */
    public static AlgaeAngle getInstance() {
        if (instance == null) {
            instance = new AlgaeAngle();
        }
        return instance;
    }

}
