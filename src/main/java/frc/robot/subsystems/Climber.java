package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
            setMotorVolt(Constants.climberExtVolt);
        }

        @Override
        public boolean isFinished() {
            return getExtension() >= Constants.climberExtDist;
        }

        @Override
        public void end(boolean interrupted) {
            setMotorVolt(0);
        }
    }

    public class RetractCmd extends Command {
        public RetractCmd() {
            addRequirements(Climber.this);
        }
        
        @Override
        public void initialize() {
            setMotorVolt(Constants.climberRetVolt);
        }

        @Override
        public boolean isFinished() {
            return getExtension() >= Constants.climberRetDist;
        }

        @Override
        public void end(boolean interrupted) {
            setMotorVolt(0);
        }
    }

    public class ResetCmd extends Command{
        public ResetCmd() {
            addRequirements(Climber.this);
        }

        @Override
        public void initialize() {
            setMotorVolt(Constants.climberResetVolt);
        }

        @Override
        public void end(boolean interrupted) {
            setMotorVolt(0);
        }
    }


    private static Climber climber = null;

    private final SparkFlex motor;

    private final RelativeEncoder encoder;

    private final ExtendCmd extendCmd;
    private final RetractCmd retractCmd;
    private final ResetCmd resetCmd;

    private final GenericEntry sb_command;
    private final GenericEntry sb_voltage;
    private final GenericEntry sb_winchExt;


    /**
     * Constructor
     */
    private Climber() {
        // Initialize Motors
        motor = new SparkFlex(Constants.climberMotor, MotorType.kBrushless);

        // Initialize Encoder
        encoder = motor.getEncoder();

        // Initialize Commands
        extendCmd = new ExtendCmd();
        retractCmd = new RetractCmd();
        resetCmd = new ResetCmd();

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
    public double getExtension() {
        return encoder.getPosition() * Constants.winchCircum;
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

    public void stop() {
        Command currentCmd = getCurrentCommand();
        if(currentCmd != null) currentCmd.cancel();
    }

    /**
     * Sets the motor voltage
     * @param voltage   sets the output voltage for the motor
     */
    private void setMotorVolt(double voltage) {
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
        sb_winchExt.setDouble(getExtension());
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
