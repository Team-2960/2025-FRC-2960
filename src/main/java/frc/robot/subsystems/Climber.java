package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConst;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {


    public class ExtendCmd extends Command {
        public ExtendCmd() {
            addRequirements(Climber.this);
        }


        @Override
        public void initialize() {
            setMotorVolt(ClimberConst.extVolt);
        }

        // @Override
        // public boolean isFinished() {
        //     return getExtension() >= Constants.climberExtDist;
        // }

        @Override
        public void end(boolean interrupted) {
            setMotorVolt(Constants.motorOff);
        }
    }

    public class RetractCmd extends Command {
        public RetractCmd() {
            addRequirements(Climber.this);
        }
        
        @Override
        public void initialize() {
            setMotorVolt(ClimberConst.retVolt);
        }

        // @Override
        // public boolean isFinished() {
        //     return getExtension() >= Constants.climberRetDist;
        // }

        @Override
        public void end(boolean interrupted) {
            setMotorVolt(Constants.motorOff);
        }
    }

    public class ResetCmd extends Command{
        public ResetCmd() {
            addRequirements(Climber.this);
        }

        @Override
        public void initialize() {
            setMotorVolt(ClimberConst.resetVolt);
        }

        @Override
        public void end(boolean interrupted) {
            setMotorVolt(Constants.motorOff);
        }
    }

    public class ClimberVoltageCommand extends Command{
        private Voltage voltage;

        public ClimberVoltageCommand(Voltage voltage){
            this.voltage = voltage;
            addRequirements(Climber.this);
        }

        public void setVoltage(Voltage voltage){
            this.voltage = voltage;
        }

        @Override
        public void execute(){
            setMotorVolt(voltage);
        }
    }

    public class SetExtPosCommand extends Command{
        public SetExtPosCommand(){
            addRequirements(Climber.this);
        }

        @Override
        public void execute(){
            setMotorVolt(ClimberConst.extVolt);
        }

        @Override
        public boolean isFinished(){
            return getPosition().getRotations() >= ClimberConst.extAngle.in(Rotations);
        }

        @Override
        public void end(boolean interrupt){
            setMotorVolt(Constants.motorOff);
        }
        
    }

    public class SetRetPosCommand extends Command {
        public SetRetPosCommand(){
            addRequirements(Climber.this);
        }

        @Override
        public void execute(){
            setMotorVolt(ClimberConst.retVolt);
        }

        @Override
        public boolean isFinished(){
            return getPosition().getRotations() <= ClimberConst.retAngle.in(Rotations);
    
        }   

        @Override
        public void end(boolean interrupt){
            setMotorVolt(Constants.motorOff);
        }
    }

    public class HoldPositionCommand extends Command{
        Angle targetPosition;

        public HoldPositionCommand(){
            addRequirements(Climber.this);
        }

        @Override
        public void initialize(){
            encoder.setPosition(0);
            targetPosition = getRelativePos();
        }

        @Override
        public void execute(){
            setPos(targetPosition);
        }

    }


    private static Climber climber = null;

    private final SparkFlex motor;

    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEncoder;

    private final ExtendCmd extendCmd;
    private final RetractCmd retractCmd;
    private final ResetCmd resetCmd;
    private final ClimberVoltageCommand climberVoltageCommand;

    private final PIDController climberPID;

    private final GenericEntry sb_command;
    private final GenericEntry sb_voltage;
    private final GenericEntry sb_winchExt;

    private MutVoltage poseVolt;


    /**
     * Constructor
     */
    private Climber() {
        // Initialize Motors
        motor = new SparkFlex(Constants.CAN_IDS.climberMotor, MotorType.kBrushless);

        // Initialize Encoder
        encoder = motor.getEncoder();
        absEncoder = motor.getAbsoluteEncoder();

        // Initialize Commands
        extendCmd = new ExtendCmd();
        retractCmd = new RetractCmd();
        resetCmd = new ResetCmd();
        climberVoltageCommand = new ClimberVoltageCommand(Constants.motorOff);

        //setDefaultCommand(new HoldPositionCommand());

        climberPID = new PIDController(1.0, 0, 0);

        poseVolt = Volts.mutable(0);

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Climber", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_command = layout.add("Command", "").getEntry();
        sb_voltage = layout.add("Motor Voltage", 0).getEntry();
        sb_winchExt = layout.add("Winch extension", 0).getEntry();
    }

    /**
     * Gets the current extension distance
     * 
     * @return distance the climber is extended
     */
    public Distance getExtension() {
        return ClimberConst.posScale.times(encoder.getPosition());
    }

    public Rotation2d getPosition(){
        return Rotation2d.fromRotations(absEncoder.getPosition());
    }

    public Angle getRelativePos(){
        return Rotations.of(encoder.getPosition());
    }

    /**
     * Resets the climber encoder
     */
    public void resetExt() {
        encoder.setPosition(0);
    }

    public void runExtend() {
        if(getCurrentCommand() != extendCmd) extendCmd.schedule();
    }

    public void runRetract() {
        if(getCurrentCommand() != retractCmd) retractCmd.schedule();
    }

    public void runReset() {
        if(getCurrentCommand() != resetCmd) resetCmd.schedule();
    }

    public void setPos(Angle pose){
        setMotorVolt(poseVolt.mut_replace(climberPID.calculate(pose.in(Rotations)), Volts));
    }

    public void setClimberVoltage(Voltage voltage){
        climberVoltageCommand.setVoltage(voltage);
        if(getCurrentCommand() != climberVoltageCommand) climberVoltageCommand.schedule();
    }

    public void stop() {
        Command currentCmd = getCurrentCommand();
        if(currentCmd != null) currentCmd.cancel();
    }

    /**
     * Sets the motor voltage
     * @param voltage   sets the output voltage for the motor
     */
    private void setMotorVolt(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * Climber periodic update
     */
    @Override
    public void periodic() {
        updateUI();
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        Command currentCmd = getCurrentCommand();
        String currentCmdName = "<null>";

        if(currentCmd != null) currentCmdName = currentCmd.getName();

        sb_command.setString(currentCmdName);
        sb_voltage.setDouble(motor.getBusVoltage() * motor.getAppliedOutput());
        sb_winchExt.setDouble(getPosition().getRotations());
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
