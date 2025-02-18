package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;

import java.util.Map;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
    private static Elevator elevator;

    /**
     * Defines an elevator position state
     */
    public class ElevatorStateValues {
        public double targetPos;
        public double angleTol;

        public ElevatorStateValues(double targetPos) {
            this.targetPos = targetPos;
            this.angleTol = Constants.elevatorDefTol;
        }

        public ElevatorStateValues(double targetPos, double angleTol) {
            this.targetPos = targetPos;
            this.angleTol = angleTol;
        }
    }

    private SparkFlex elevatorMotor;

    private RelativeEncoder elevatorEncoder;

    private SparkLimitSwitch elevatorLimitBot;
    
    private SparkLimitSwitch elevatorLimitTop;

    private Trigger elevLimBotTrigger;
    
    private Trigger elevLimTopTrigger;

    private PIDController elevatorPID;

    private ElevatorFeedforward elevatorFFS0;

    private final ElevatorStateValues defaultState = new ElevatorStateValues(0);

    private ElevatorStateValues targetState = defaultState;

    private double elevatorVolt;
    private double elevatorRate;

    private Map<String, ElevatorStateValues> elevatorStates = Map.of(
            "Match Start", new ElevatorStateValues(1),
            "Home", defaultState,
            "Intake", new ElevatorStateValues(2),
            "Speaker", new ElevatorStateValues(3),
            "lineSpeaker", new ElevatorStateValues(4),
            "longShot", new ElevatorStateValues(5),
            "Amp", new ElevatorStateValues(6),
            "Climb", new ElevatorStateValues(7),
            "AmpSideShoot", new ElevatorStateValues(8),
            "home", new ElevatorStateValues(9)
            //"Climb Balance", new ElevatorStateValues(Rotation2d.fromDegrees(97.38), 0),
            //"Trap Score", new ElevatorStateValues(Rotation2d.fromDegrees(70), 2)
        );

    private GenericEntry sb_elevatorMode;
    private GenericEntry sb_posPosCurrent;
    private GenericEntry sb_posPosSetPoint;
    private GenericEntry sb_posRateCurrent;
    private GenericEntry sb_posRateSetPoint;
    private GenericEntry sb_posRateError;
    private GenericEntry sb_posM1Volt;
    private GenericEntry sb_posM2Volt;
    private GenericEntry sb_posTargetVolt;
    private GenericEntry sb_posPosRotations;
    private GenericEntry sb_atPos;
    private GenericEntry sb_atTarget;
    private GenericEntry sb_currentElevatorCommand;


    //Command to set the voltage of the Elevator
    public class ElevatorVoltageCommand extends Command{
        private double targetVoltage;

        public ElevatorVoltageCommand(double targetVoltage){
            this.targetVoltage = targetVoltage;

            //Makes the Elevator Subsystem required for the command
            addRequirements(Elevator.this);
        }

        @Override
        public void execute(){
            //Actually executes the command aka sets the voltage
            setMotorVolt(targetVoltage);
        }

        //Method used after creating the command to set the voltage
        public void setVoltage(double targetVoltage){
            this.targetVoltage = targetVoltage;
        }

    }

    //Command to send values to the PID + Feed Forward
    //This Controls the rate aka velocity of the elevator, NOT The position
    public class ElevatorRateCommand extends Command{
        private double targetRate;
        private double tolerance;

        public ElevatorRateCommand(double targetRate, double tolerance){
            this.targetRate = targetRate;
            this.tolerance = tolerance;
            
            //Make Elevator Subsystem required for this command
            addRequirements(Elevator.this);
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
            setElevatorRate(targetRate);
        }

        @Override
        public boolean isFinished(){
            return targetRate == 0 || Math.abs(targetRate) <= tolerance;
        }
    }

    public class ElevatorHoldCommand extends Command{
        private double target;

        public ElevatorHoldCommand(){
            target = 0;
            addRequirements(Elevator.this);
        }

        @Override
        public void initialize(){
            target = getElevatorPos();
        }

        @Override
        public void execute(){
            setElevatorPos(target);
        }
    }

    public class ElevatorPosCommand extends Command{
        private double elevatorPos;
        
        public ElevatorPosCommand(double elevatorPos){
            this.elevatorPos = elevatorPos;
            addRequirements(Elevator.this);
        }

        public void setPos(double elevatorPos){
            this.elevatorPos = elevatorPos;
        }

        @Override
        public void execute(){
            setElevatorPos(elevatorPos);
        }
        
        @Override
        public boolean isFinished(){
            return atPos(elevatorPos, Constants.elevatorPosTol);
        }
    }

    public class EncoderResetCommand extends Command{
        private double resetValue;

        public EncoderResetCommand(double resetValue){
            this.resetValue = resetValue;
        }

        /**
         * @param resetValue
         * The value you want to reset the encoder to
        */
        public void resetTo(double resetValue){
            this.resetValue = resetValue;
        }

        @Override
        public void initialize(){
            elevatorEncoder.setPosition(resetValue);
        }
        
        @Override
        public boolean isFinished(){
            return true;
        }
    }

    public class ElevatorBrakeModeCommand extends Command{
        
    }


    private final ElevatorVoltageCommand elevatorVoltageCommand;
    private final ElevatorRateCommand elevatorRateCommand;
    private final ElevatorPosCommand elevatorPosCommand;
    private final ElevatorHoldCommand elevatorHoldCommand;

    /**
     * Constructor
     */
    private Elevator() {        

        elevatorMotor = new SparkFlex(Constants.elevatorMotor, MotorType.kBrushless);

        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorLimitTop = elevatorMotor.getForwardLimitSwitch();

        elevatorLimitBot = elevatorMotor.getReverseLimitSwitch();

        elevatorPID = new PIDController(Constants.elevatorPIDS.kP, Constants.elevatorPIDS.kP, Constants.elevatorPIDS.kP);

        elevatorFFS0 = new ElevatorFeedforward(Constants.elevatorFFS.kS, Constants.elevatorFFS.kG, Constants.elevatorFFS.kV);

        elevLimBotTrigger = new Trigger(elevatorLimitBot::isPressed);
        
        elevLimTopTrigger = new Trigger(elevatorLimitTop::isPressed);


        //TODO change reset values to whatever they need to be
        elevLimBotTrigger.whileTrue(new EncoderResetCommand(0));

        elevLimTopTrigger.whileTrue(new EncoderResetCommand(0));

        // TODO Set abs encoder offset

        // Set control mode
        elevatorVolt = 0;
        elevatorRate = 0;

        // Set target state to current state
        targetState = new ElevatorStateValues(getElevatorPos());

        // Initialize Timer        
        shuffleBoardInit();

        elevatorVoltageCommand = new ElevatorVoltageCommand(0);
        elevatorRateCommand = new ElevatorRateCommand(0, 0);
        elevatorPosCommand = new ElevatorPosCommand(0);
        elevatorHoldCommand = new ElevatorHoldCommand();

        setDefaultCommand(elevatorHoldCommand);
    }

    public void shuffleBoardInit(){
        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Elevator", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_elevatorMode = layout.add("Elevator Control Mode", "").getEntry();
        sb_posPosCurrent = layout.add("Pos Position Current", 0).getEntry();
        sb_posPosSetPoint = layout.add("Pos Position Set Point", 0).getEntry();
        sb_posRateCurrent = layout.add("Pos Rate Current", 0).getEntry();
        sb_posRateSetPoint = layout.add("Pos Rate Set Point", 0).getEntry();
        sb_posRateError = layout.add("Pos Rate Error", 0).getEntry();
        sb_posM1Volt = layout.add("Pos Motor 1 Voltage", 0).getEntry();
        sb_posM2Volt = layout.add("Pos Motor 2 Voltage", 0).getEntry();
        sb_posTargetVolt = layout.add("Pos Target Voltage", 0).getEntry();
        sb_posPosRotations = layout.add("Elevator Encoder Rotations Output", 0).getEntry();
        sb_atPos = layout.add("At Pos", false).getEntry();
        sb_atTarget = layout.add("At Target", false).getEntry();
        //sb_currentElevatorCommand = layout.add("Current Elevator Command", getCurrentCommand().getName()).getEntry();

    }

    /**
     * Gets the current elevator position
     * 
     * @return current elevator position (in)
     */
    public double getElevatorPos() {
        double pos = elevatorEncoder.getPosition() * Constants.elevatorScale;
        return pos;
    }

    /**
     * Gets the current elevator velocity
     * 
     * @return current elevator velocity (in per sec)
     */
    public double getElevatorVelocity() {
        return elevatorEncoder.getVelocity();
    }


    /**
     * Check if the elevator is at its target position
     * 
     * @return true if the position is at their target
     */
    public boolean atPos(double targetPos, double posTol) {
        double currentPos = getElevatorPos();

        return Math.abs(targetPos - currentPos) < posTol;
    }




    /**
     * Determines the current target elevator control rate
     * 
     * @return target elevator control rate based on current settings
     */

    /* 
     private double getTargetElevatorRate() {
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
     * Calculate the trapezoidal control rate for the current elevator target position
     * 
     * @return target elevator control rate
     */
    private void setElevatorPos(double targetPos) {
        
        // Calculate trapezoidal profile
        double currentPos = getElevatorPos();
        double maxPosRate = Constants.maxElevatorAutoSpeed;
        double posError = targetPos - currentPos;

        double targetSpeed = maxPosRate * (posError > 0 ? 1 : +-1);
        double rampDownSpeed = posError / Constants.elevatorRampDownDist * maxPosRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;
        
        setElevatorRate(targetSpeed);
    }

    private void setElevatorState(ElevatorStateValues elevatorState){
        double targetPos = elevatorState.targetPos;

        setElevatorPos(targetPos);
    }

    /**
     * Updates the control of the elevator rate
     * 
     * @param targetSpeed target
     */
    private void setElevatorRate(double targetSpeed) {
        double result = this.elevatorRate;

        double currentPos = getElevatorPos();
        double posRate = getElevatorVelocity();

        ElevatorFeedforward elevatorFF = elevatorFFS0;

        elevatorPID.setPID(Constants.elevatorPIDS.kP, Constants.elevatorPIDS.kI, Constants.elevatorPIDS.kD);
        elevatorFF = elevatorFFS0;
            

        sb_posRateError.setDouble(posRate - targetSpeed);

        // Calculate motor voltage output
        double calcPID = elevatorPID.calculate(posRate, targetSpeed);
        double calcFF = elevatorFF.calculate(currentPos, targetSpeed);

        result = calcPID + calcFF;
        
        setMotorVolt(result);
        
        //Shuffleboard display
        this.elevatorRate = result;
    }

    /**
     * Sets the motor voltage for the elevator angle control. Manages soft limits as
     * well.
     * 
     * @param voltage desired motor voltage
     */
    //TODO Change to use Spark Flex
    private void setMotorVolt(double voltage) {
        // Set Motors
        VoltageOut settings = new VoltageOut(voltage);
        settings.EnableFOC = true;
        //elevatorMotor.setControl(settings); TODO
        
        //Shuffleboard display
        this.elevatorVolt = settings.Output;
        
    }

    public ElevatorStateValues getState(){
        return targetState;
    }


    /**
     * Updates shuffleboard
     */
    private void updateUI(double targetRate, double targetVolt) {
        sb_elevatorMode.setString("");
        sb_posPosCurrent.setDouble(getElevatorPos());
        sb_posPosSetPoint.setDouble(targetState.targetPos);
        sb_posRateCurrent.setDouble(getElevatorVelocity());
        sb_posRateSetPoint.setDouble(targetRate);
        sb_posM1Volt.setDouble(elevatorMotor.getBusVoltage());//TODO
        sb_posTargetVolt.setDouble(targetVolt);
        sb_posPosRotations.setDouble(elevatorEncoder.getPosition());
    }

    public void setRateCommand(double rate){
        elevatorRateCommand.setRate(rate);
        if(getCurrentCommand() != elevatorRateCommand) elevatorRateCommand.schedule();
    }

    public void setTolRateCommand(double rate, double tolerance){
        elevatorRateCommand.setToleranceRate(rate, tolerance);
        if(getCurrentCommand() != elevatorRateCommand) elevatorRateCommand.schedule();
    }

    public void setPosCommand(double angle){
        elevatorPosCommand.setPos(angle);
        if(getCurrentCommand() != elevatorPosCommand) elevatorPosCommand.schedule();
    }

    public void setStateCommand(String stateName){
        ElevatorStateValues state = elevatorStates.get(stateName);
        setPosCommand(state.targetPos);
    }

    public void setHoldCommand(){
        if(getCurrentCommand() != elevatorHoldCommand) new ElevatorHoldCommand().schedule();
    }

    public Command getElevatorCommand(){
        return getCurrentCommand();
    }
    
    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic() {
        updateUI(elevatorRate, elevatorVolt);
        var currentCommand = getCurrentCommand();
        String curCommandName = "null";
        if (currentCommand != null) curCommandName = currentCommand.getName();
        
        SmartDashboard.putString("Current Command", curCommandName);
    }


    /**
     * Static initializer for the elevator class
     */
    public static Elevator getInstance() {
        if (elevator == null) {
            elevator = new Elevator();
        }
        return elevator;
    }

}
