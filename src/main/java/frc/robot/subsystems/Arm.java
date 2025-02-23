package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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


    private final ArmVoltageCommand armVoltageCommand;
    private final ArmRateCommand armRateCommand;
    private final ArmAngleCommand armAngleCommand;
    private final ArmHoldCommand armHoldCommand;

    /**
     * Constructor
     */
    private Arm() {        
        motor = new SparkFlex(Constants.armMotor, MotorType.kBrushless);
        absEncoder = motor.getAbsoluteEncoder();
        relEncoder = motor.getExternalEncoder();

        armPID = new PIDController(Constants.armPID.kP, Constants.armPID.kP, Constants.armPID.kP);

        armFF = new ArmFeedforward(Constants.armFF.kS, Constants.armFF.kG, Constants.armFF.kV);

        // Set control mode
        armVolt = 0;
        armRate = 0;

        // Initialize Timer        
        shuffleBoardInit();

        armVoltageCommand = new ArmVoltageCommand(0);
        armRateCommand = new ArmRateCommand(0, 0);
        armAngleCommand = new ArmAngleCommand(Rotation2d.fromDegrees(0));
        armHoldCommand = new ArmHoldCommand();

        setDefaultCommand(armHoldCommand);
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

    }

    /**
     * Gets the current arm angle
     * 
     * @return current arm angle
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromRotations(absEncoder.getPosition());
    }

    /**
     * Gets the current arm angle rate
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

        double targetSpeed = maxAngleRate * (angleError.getRadians() > 0 ? 1 : +-1);
        double rampDownSpeed = angleError.getRadians() / Constants.armRampDownDist.getRadians() * maxAngleRate;

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
        double result = this.armRate;

        Rotation2d currentAngle = getArmAngle();
        double angleRate = getArmVelocity();            

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
    //TODO Change to use Spark Flex
    private void setMotorVolt(double voltage) {
        motor.setVoltage(voltage);
        
        //Shuffleboard display
        this.armVolt = voltage;
        
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
