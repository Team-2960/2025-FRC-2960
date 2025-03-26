package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CAN_IDS;
import frc.robot.Constants.ElevConst;
import frc.robot.Util.Limits;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;


public class Elevator extends SubsystemBase {
    private SparkFlex motor;
    private RelativeEncoder encoder;
    private SparkLimitSwitch limitBot;

    private Trigger limBotTrigger;

    private PIDController pid;
    private ElevatorFeedforward ff;
    
    private MutDistance currentPos;
    private MutLinearVelocity currentRate;
    private MutVoltage currentVoltage;
    private MutCurrent currentCurrent;

    private GenericEntry sb_currentCommand;
    private GenericEntry sb_posPosCurrent;
    private GenericEntry sb_posPosSetPoint;
    private GenericEntry sb_posRateCurrent;
    private GenericEntry sb_posRateSetPoint;
    private GenericEntry sb_posRateError;
    private GenericEntry sb_posVolt;
    private GenericEntry sb_posTargetVolt;
    private GenericEntry sb_topSoftLim;
    private GenericEntry sb_botSoftLim;

    //System Identification
    private final SysIdRoutine sysIdRoutine;
    private final MutVoltage sysid_appliedVoltage;
    private final MutCurrent sysid_appliedCurrent;
    private final MutDistance sysid_distance;
    private final MutLinearVelocity sysid_linearVelocity;

    public final Command sysIdCommandUpQuasi;
    public final Command sysIdCommandDownQuasi;
    public final Command sysIdCommandUpDyn;
    public final Command sysIdCommandDownDyn;
    public final Command sysIdCommandGroup;

    /**
     * Command to control the elevator output voltage
     */
    public class VoltageCommand extends Command{
        private final Supplier<Voltage> voltageSupplier;  //< voltage supplier

        /**
         * Constructor
         * @param voltageSupplier    voltage supplier
         */
        public VoltageCommand(Supplier<Voltage> voltageSupplier){
            this.voltageSupplier = voltageSupplier;

            addRequirements(Elevator.this);
        }

        /**
         * Update the output voltage
         */
        @Override
        public void execute(){
            setMotorVolt(voltageSupplier.get());
        }
    }

    /**
     * Command to set the target rate of the elevator
     */
    public class RateCommand extends Command{
        private final Supplier<LinearVelocity> rateSupplier;  //< Rate supplier
        private final boolean autoComplete;                   //< Flag to determine if command will complete automatically

        public RateCommand(Supplier<LinearVelocity> rateSupplier, boolean autoComplete){
            this.rateSupplier = rateSupplier;
            this.autoComplete = autoComplete;
            
            //Make Elevator Subsystem required for this command
            addRequirements(Elevator.this);
        }
        
        //What actually sets the rate/ execute the action to set the rate
        @Override
        public void execute(){
            setRate(rateSupplier.get());
        }

        @Override
        public boolean isFinished() {
            return autoComplete && rateSupplier.get().in(InchesPerSecond) == 0;
        }
    }

    /**
     * Command to hold the elevator at the current postion
     */
    public class HoldCommand extends Command{
        private final MutDistance target;     //< Target position

        /**
         * Constructor
         */
        public HoldCommand(){
            target = Inches.mutable(0);
            addRequirements(Elevator.this);
        }

        /**
         * Get the current elevator position
         */
        @Override
        public void initialize(){
            target.mut_replace(getPos());
        }

        /**
         * Update target position
         */
        @Override
        public void execute(){
            setPos(target);
        }
    }

    /**
     * Command to move to a specific position
     */
    public class PositionCommand extends Command{
        private final Distance target;
        private final Distance tolerance;

        /**
         * Constructor. Tolerance is set to the default tolerance.
         * @param target    target position
         */
        public PositionCommand(Distance target){
            this(target, ElevConst.posTol); // TODO move default tolerance to containing class
        }
        
        /**
         * Constructor
         * @param target        target position
         * @param tolerance     position tolerance
         */
        public PositionCommand(Distance target, Distance tolerance){
            this.target = target;
            this.tolerance = tolerance;
            addRequirements(Elevator.this);
        }

        /**
         * Update the target position
         */
        @Override
        public void execute(){
            setPos(target);
        }
        
        /**
         * Check if the elevator is at position
         */
        @Override
        public boolean isFinished(){
            return Limits.inTol(target, tolerance, getPos());
        }
    }

    /**
     * Command to reset the encoder to a known value
     */
    public class EncoderResetCommand extends Command{
        private Distance resetValue;

        /**
         * Constructor. Reset value is 0 inches.
         */
        public EncoderResetCommand() {
            this(Inches.of(0));
        }

        /**
         * Constructor
         * @param resetValue    value to reset the encoder to
         */
        public EncoderResetCommand(Distance resetValue){
            this.resetValue = resetValue;
        }

        /**
         * Reset the encoder
         */
        @Override
        public void initialize(){
            encoder.setPosition(resetValue.in(Inches));
        }
        
        /**
         * Finish the command immediately
         */
        @Override
        public boolean isFinished(){
            return true;
        }
    }
    
    /**
     * Command to monitor the soft limits for SysID
     */
    public class SoftLimCheckCommand extends Command{
        public enum Direction{UP,DOWN}

        Direction direction;

        public SoftLimCheckCommand(Direction direction){
            this.direction = direction;
        }

        @Override
        public boolean isFinished(){
            return direction == Direction.UP && topLimitReached() || 
                direction == Direction.DOWN && botLimitReached();
        }
    }

    /**
     * Constructor
     */
    public Elevator() {        
        motor = new SparkFlex(CAN_IDS.elevatorMotor, MotorType.kBrushless);
        encoder = motor.getEncoder();
        limitBot = motor.getReverseLimitSwitch();

        pid = new PIDController(ElevConst.pid.kP, ElevConst.pid.kI, ElevConst.pid.kD);
        ff = new ElevatorFeedforward(ElevConst.ff.kS, ElevConst.ff.kG, ElevConst.ff.kV);

        //Motor Config
        SparkFlexConfig config = new SparkFlexConfig();
        config.encoder
            .positionConversionFactor(ElevConst.distScale.in(Inches))
            .velocityConversionFactor(ElevConst.velScale.in(InchesPerSecond));
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Setup encoder reset trigger
        limBotTrigger = new Trigger(limitBot::isPressed);
        limBotTrigger.onTrue(new EncoderResetCommand());

        // Initialize mutable units
        currentPos = Inches.mutable(0);
        currentRate = InchesPerSecond.mutable(0);
        currentVoltage = Volts.mutable(0);
        currentCurrent = Amps.mutable(0);

        // Initialize Timer        
        shuffleBoardInit();
        setDefaultCommand(new HoldCommand());

        // Initialize SysID
        sysIdRoutine = new SysIdRoutine(
            new Config(
                Volts.per(Second).of(.5),
                Volts.of(7),
                Seconds.of(10)
            ), 
            new Mechanism(
                this::setMotorVolt,
                this::sysIDLogging, this)
        );

        sysid_appliedVoltage = Volts.mutable(0);
        sysid_appliedCurrent = Amps.mutable(0);
        sysid_distance = Inches.mutable(0);
        sysid_linearVelocity =  InchesPerSecond.mutable(0);

        sysIdCommandUpQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownQuasi = sysIdRoutine.quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);
        sysIdCommandUpDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward);
        sysIdCommandDownDyn = sysIdRoutine.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse);

        sysIdCommandGroup =  new SequentialCommandGroup(
            new ParallelRaceGroup(
                sysIdCommandUpQuasi,
                new SoftLimCheckCommand(SoftLimCheckCommand.Direction.UP)
            ),
            new ParallelRaceGroup(
                sysIdCommandDownQuasi,
                new SoftLimCheckCommand(SoftLimCheckCommand.Direction.DOWN)
            ),
            new ParallelRaceGroup(
                sysIdCommandUpDyn,
                new SoftLimCheckCommand(SoftLimCheckCommand.Direction.UP)
            ),
            new ParallelRaceGroup(
                sysIdCommandDownDyn,
                new SoftLimCheckCommand(SoftLimCheckCommand.Direction.DOWN)
            )
        );
    }

    public void shuffleBoardInit(){
        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Elevator", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_currentCommand = layout.add("Current Command", "").getEntry();
        sb_posPosCurrent = layout.add("Pos Position Current", 0).getEntry();
        sb_posPosSetPoint = layout.add("Pos Position Set Point", 0).getEntry();
        sb_posRateCurrent = layout.add("Pos Rate Current", 0).getEntry();
        sb_posRateSetPoint = layout.add("Pos Rate Set Point", 0).getEntry();
        sb_posRateError = layout.add("Pos Rate Error", 0).getEntry();
        sb_posVolt = layout.add("Pos Motor Voltage", 0).getEntry();
        sb_posTargetVolt = layout.add("Pos Target Voltage", 0).getEntry();
        sb_topSoftLim = layout.add("Top Soft Lim", false).getEntry();
        sb_botSoftLim = layout.add("Bottom Soft Lim", false).getEntry();
    }

    /**
     * Gets the current elevator position
     * 
     * @return current elevator position
     */
    public Distance getPos() {
        return currentPos.mut_setMagnitude(encoder.getPosition());
    }

    /**
     * Gets the current elevator velocity
     * 
     * @return current elevator velocity
     */
    public LinearVelocity getVelocity() {
        return currentRate.mut_setMagnitude(encoder.getVelocity());
    }

    /**
     * Gets the current motor voltage
     * @return  current motor voltage
     */
    public Voltage getVoltage(){
        return currentVoltage.mut_setMagnitude(motor.getAppliedOutput() * motor.getBusVoltage());
    }

    /**
     * Gets the current motor current
     * @return  current motor current
     */
    public Current getCurrent(){
        return currentCurrent.mut_setMagnitude(motor.getOutputCurrent());
    }

    /**
     * Calculate the trapezoidal control rate for the current elevator target position
     * 
     * @return target elevator control rate
     */
    private void setPos(Distance target) {
        // Calculate trapezoidal profile
        MutLinearVelocity targetSpeed = ElevConst.maxAutoSpeed.mutableCopy();
        MutDistance error = getPos().mutableCopy();
        error.mut_times(-1).mut_plus(target);

        targetSpeed.mut_times(error.in(Inches) > 0 ? 1 : +-1);
        double rampDownSpeed = error.in(Inches) / ElevConst.rampDownDist.in(Inches) * targetSpeed.in(InchesPerSecond);

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed.in(InchesPerSecond)))
            targetSpeed.mut_replace(rampDownSpeed, InchesPerSecond);
        
        setRate(targetSpeed);

        // Update Shuffleboard
        sb_posPosSetPoint.setString(target.toShortString());
    }

    /**
     * Updates the control of the elevator rate
     * 
     * @param targetSpeed target
     */
    private void setRate(LinearVelocity target) {
        // Get current velocity
        MutLinearVelocity rate = getVelocity().mutableCopy();        
        
        // Calculate output voltage
        double calcPID = pid.calculate(rate.in(InchesPerSecond), target.in(InchesPerSecond));
        double calcFF = ff.calculate(target.in(InchesPerSecond));

        Voltage result = Volts.of(calcPID + calcFF);
        
        // Set output voltage
        setMotorVolt(result);
        
        //Shuffleboard display
        sb_posRateSetPoint.setString(target.toShortString());
        sb_posRateError.setString(rate.mut_minus(target).toShortString());
    }

    /**
     * Sets the motor voltage for the elevator angle control. Manages soft limits as
     * well.
     * 
     * @param voltage desired motor voltage
     */
    private void setMotorVolt(Voltage voltage) {
        if (topLimitReached() && voltage.in(Volts) > 0){
            voltage = Constants.motorOff;
        }
        if (botLimitReached() && voltage.in(Volts) < 0){
            voltage = Constants.motorOff;
        }

        motor.setVoltage(voltage);
        
        //Shuffleboard display
        sb_posTargetVolt.setString(voltage.toShortString());
    }

    /**
     * Check if the top limit has been reached
     * @return true if the top limit has been reached
     */
    public boolean topLimitReached(){
        return getPos().in(Inches) >= ElevConst.topLim.in(Inches);
    }

    /**
     * Check if the bottom limit has been reached
     * @return  true if the bottom limit has been reached
     */
    public boolean botLimitReached(){
        return getPos().in(Inches) <= ElevConst.botLim.in(Inches);

    }

    /**
     * Update sysID logging
     * @param log   sysID log object
     */
    private void sysIDLogging(SysIdRoutineLog log){
        log.motor("Elevator")
            .voltage(sysid_appliedVoltage.mut_replace(getVoltage()))
            .current(sysid_appliedCurrent.mut_replace(getCurrent()))
            .linearPosition(sysid_distance.mut_replace(getPos()))
            .linearVelocity(sysid_linearVelocity.mut_replace(getVelocity()));
    }


    /**
     * Updates shuffleboard
     */
    private void updateUI() {
        Command currentCmd = getCurrentCommand();
        String currentCmdName = "<null>";

        if(currentCmd != null) currentCmdName = currentCmd.getName();   

        sb_currentCommand.setString(currentCmdName);
        sb_posPosCurrent.setString(getPos().toShortString());
        sb_posRateCurrent.setString(getVelocity().toShortString());
        sb_posVolt.setString(getVoltage().toShortString());
        sb_topSoftLim.setBoolean(topLimitReached());
        sb_botSoftLim.setBoolean(botLimitReached());
    }

    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic() {
        updateUI();
        var currentCommand = getCurrentCommand();
        String curCommandName = "null";
        if (currentCommand != null) curCommandName = currentCommand.getName();
        
        SmartDashboard.putString("Current Command", curCommandName);
    }
}
