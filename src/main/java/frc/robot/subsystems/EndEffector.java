package frc.robot.subsystems;



import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase{
    /*Coral intake/eject
    motor: Neo vortex (spark flex) used to control intake and eject
    photoeye: ?? Used to detect fully loaded coral
    photoeye: model#  class library:
    */
    private static EndEffector instance = null;
    
    private DigitalInput coralPresentPE;//Photoeye coral detector
    private SparkFlex coralDrive;

    private Trigger intakeTrigger;

    private EjectCmd ejectCmd;
    
    private GenericEntry sb_currentCmd;
    private GenericEntry sb_motorVoltage;
    private GenericEntry sb_photoeyeState;

    //Command for ejecting coral
    public class EjectCmd extends Command {
        private double startTime;
        private final double runTime;

        /**
         * Constructor
         */
        public EjectCmd(double runTime){
            startTime = 0;
            this.runTime = runTime;
        }

        @Override
        public void initialize(){
            startTime = Timer.getFPGATimestamp();
            setEject();
        }

        @Override
        public boolean isFinished(){
            return Timer.getFPGATimestamp() > startTime + runTime;
        }

        @Override
        public void end(boolean interupted){
            setStop();
        }
        
    }

    //Command for intaking coral 
    public class IntakeCmd extends Command{

        @Override
        public void initialize(){
            setIntake();
        }

        @Override
        public boolean isFinished(){
            return !coralPresentPE.get();
        }

        @Override
        public void end(boolean interupted){
            setStop();
        }
    }

    /**
     * Constructor
     */
    private EndEffector(){
        coralDrive = new SparkFlex(Constants.coralMotor, MotorType.kBrushless);
        coralPresentPE = new DigitalInput(Constants.coralPresentPE);

        intakeTrigger = new Trigger(coralPresentPE::get);
        intakeTrigger.onTrue(new IntakeCmd());

        ejectCmd = new EjectCmd(Constants.coralEjectTime);

        //Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("End Effector", BuiltInLayouts.kList)
                .withSize(1, 3);

        sb_currentCmd = layout.add("current command", "").getEntry();
        sb_motorVoltage = layout.add("motor voltage", 2960).getEntry();
        sb_photoeyeState = layout.add("photoeye state", false).getEntry();


    }

    /**
     * Schedules eject command
     */
    public void runEject(){
        if(getCurrentCommand() != ejectCmd){
            ejectCmd.schedule();
        }
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



    @Override
    public void periodic(){
        updateUI();
    }

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

