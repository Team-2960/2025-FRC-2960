package frc.robot.subsystems;



import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Manages coral end effector
 */
public class EndEffector extends SubsystemBase{
    /*Coral intake/eject
    motor: Neo vortex (spark flex) used to control intake and eject
    photoeye: ?? Used to detect fully loaded coral
    photoeye: model#  class library:
    */
    private static EndEffector instance = null;
    
    private DigitalInput coralPresentPE;//Photoeye coral detector
    private SparkFlex coralDrive;
    private SparkBaseConfig coralConfig;

    private Trigger intakeTrigger;

    private final EjectCmd ejectCmd;
    private final Command timedEjectCmd;
    private final IntakeCmd intakeCmd;
    
    private GenericEntry sb_currentCmd;
    private GenericEntry sb_motorVoltage;
    private GenericEntry sb_photoeyeState;

    /**
     * Command for ejecting coral
     * */
    public class EjectCmd extends Command {
        public EjectCmd() {
            addRequirements(EndEffector.this);
        }

        @Override
        public void initialize(){
            setEject();
        }

        @Override
        public void end(boolean interupted){
            setStop();
        }
        
    }

    /**
     * Runs eject command for a specific amount of time
     */
    public class TimedEjectCmd extends WaitCommand {
        public TimedEjectCmd() {
            super(Constants.coralEjectTime);
            addRequirements(EndEffector.this);
        }

        public TimedEjectCmd(double runTime) {
            super(runTime);
            addRequirements(EndEffector.this);
        }

        @Override
        public void initialize(){
            setEject();
        }

        @Override
        public void end(boolean interupted){
            setStop();
        }
    }

    /**
     * Command for intaking coral
     */ 
    public class IntakeCmd extends Command{
        public IntakeCmd() {
            addRequirements(EndEffector.this);
        }

        @Override
        public void initialize(){
            setIntake();
        }

        @Override
        public void end(boolean interupted){
            setStop();
        }
    }

    public class StopCmd extends Command{
        public StopCmd(){
            addRequirements(EndEffector.this);
        }

        @Override
        public void initialize(){
            setStop();
        }

    }

    public class CoralPresentCommand extends Command{
        @Override
        public boolean isFinished(){
            return isCoralPresent();
        }
    }

    public class CoralNotPresentCommand extends Command{
        @Override
        public boolean isFinished(){
            return !isCoralPresent();
        }
    }

    /**
     * Constructor
     */
    private EndEffector(){
        coralDrive = new SparkFlex(Constants.coralMotor, MotorType.kBrushless);
        coralPresentPE = new DigitalInput(Constants.coralPresentPE);
        coralDrive.configure(new SparkFlexConfig().inverted(true), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ejectCmd = new EjectCmd();
        timedEjectCmd = new TimedEjectCmd();
        intakeCmd = new IntakeCmd();

        intakeTrigger = new Trigger(coralPresentPE::get);
        intakeTrigger.whileFalse(intakeCmd);
        

        //Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("End Effector", BuiltInLayouts.kList)
                .withSize(1, 3);

        sb_currentCmd = layout.add("current command End Effector", "").getEntry();
        sb_motorVoltage = layout.add("motor voltage End Effector", 2960).getEntry();
        sb_photoeyeState = layout.add("photoeye state End Effector", false).getEntry();


    }

    /**
     * Schedules timed eject command
     */
    public void runTimedEject(){
        if(getCurrentCommand() != timedEjectCmd)  timedEjectCmd.schedule();
    }

    /**
     * Schedules eject command
     */
    public void runEject(){
        if(getCurrentCommand() != ejectCmd) ejectCmd.schedule();
    }

    /**
     * Schedules intake command
     */
    public void runIntake(){
        if (getCurrentCommand() != intakeCmd) intakeCmd.schedule();
    }
    
    /**
     * Stops all current commands
     */
    public void stop() {
        Command currentCmd = getCurrentCommand();
        if (currentCmd != null) currentCmd.cancel();
    }

    /**
     * sets voltage on end effector
     * @param voltage
     */
    public void setMotorVolt(double voltage){
        coralDrive.setVoltage(voltage);
    }

    /**
     * set motor voltage for ejection
     */
    public void setEject(){
        setMotorVolt(Constants.coralEjectVolt);
    }

    /**
     * set motor voltage for intake
     */
    public void setIntake(){
        setMotorVolt(Constants.coralIntakeVolt);
    }

    /**
     * stop motor
     */
    public void setStop(){
        setMotorVolt(0);
    }

    public void setReverse(){
        setMotorVolt(-Constants.coralIntakeVolt);
    }

    public boolean isCoralPresent(){
        return !coralPresentPE.get();
    }


    /**
     * Periodic method
     */
    @Override
    public void periodic(){
        updateUI();
    }

    /**
     * Update Shuffleboard
     */
    private void updateUI(){
        Command currentCommand = getCurrentCommand();
        String commandName = "null";

        if(currentCommand != null){
            commandName = currentCommand.getName();
        }

        sb_currentCmd.setString(commandName);
        sb_motorVoltage.setDouble(coralDrive.getBusVoltage() * coralDrive.getAppliedOutput());
        sb_photoeyeState.setBoolean(coralPresentPE.get());
    }

     /**
     * Static Initializer
     */
    public static EndEffector getInstance() {
        if (instance == null) {
            instance = new EndEffector();
        }
        return instance;
    }
}

