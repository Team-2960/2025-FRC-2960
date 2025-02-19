package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeAngle extends SubsystemBase {
    private static AlgaeAngle instance;

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
            setRate(targetRate);
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
            setAngle(Angle);
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
        relEncoder = motor.getAlternateEncoder();

        pid = new PIDController(Constants.algaeAnglePID.kP, Constants.algaeAnglePID.kP, Constants.algaeAnglePID.kP);

        ff = new ArmFeedforward(Constants.algaeAngleFF.kS, Constants.algaeAngleFF.kG, Constants.algaeAngleFF.kV);

        // Set control mode
        voltage = 0;
        rate = 0;

        // Initialize Timer        
        shuffleBoardInit();

        VoltageCommand = new VoltageCommand(0);
        RateCommand = new RateCommand(0, 0);
        AngleCommand = new AngleCommand(Rotation2d.fromDegrees(0));
        HoldCommand = new HoldCommand();

        setDefaultCommand(HoldCommand);
    }

    public void shuffleBoardInit(){
        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_Command = layout.add(" Command", "").getEntry();
        sb_anglePosCurrent = layout.add("Angle Position Current", 0).getEntry();
        sb_anglePosSetPoint = layout.add("Angle Position Set Point", 0).getEntry();
        sb_angleRateCurrent = layout.add("Angle Rate Current", 0).getEntry();
        sb_angleRateSetPoint = layout.add("Angle Rate Set Point", 0).getEntry();
        sb_angleRateError = layout.add("Angle Rate Error", 0).getEntry();
        sb_angleVolt = layout.add("Angle Motor Voltage", 0).getEntry();
        sb_angleTargetVolt = layout.add("Angle Target Voltage", 0).getEntry();

    }

    /**
     * Gets the current  angle
     * 
     * @return current  angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(absEncoder.getPosition());
    }

    /**
     * Gets the current  angle rate
     * 
     * @return current  angle rate in radians per second
     */
    public double getVelocity() {
        return relEncoder.getVelocity();
    }


    /**
     * Check if the  is at its target angle
     * 
     * @return true if the angle are at their target
     */
    public boolean atAngle(Rotation2d targetAngle, Rotation2d angleTol) {
        Rotation2d currentAngle = getAngle();

        return Math.abs(targetAngle.getDegrees() - currentAngle.getDegrees()) < angleTol.getDegrees();
    }

    /**
     * Calculate the trapezoidal control rate for the current  target position
     * 
     * @return target  control rate
     */
    private void setAngle(Rotation2d targetAngle) {
        
        // Calculate trapezoidal profile
        Rotation2d currentAngle = getAngle();
        double maxAngleRate = Constants.maxAlgaeAutoSpeed;
        Rotation2d angleError = targetAngle.minus(currentAngle);

        double targetSpeed = maxAngleRate * (angleError.getRadians() > 0 ? 1 : +-1);
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
        motor.setVoltage(voltage);
        
        //Shuffleboard display
        this.voltage = voltage;
        
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
