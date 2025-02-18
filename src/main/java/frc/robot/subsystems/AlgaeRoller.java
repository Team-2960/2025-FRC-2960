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

public class AlgaeRoller extends SubsystemBase{
    /*Algae intake/eject
    motor: Neo vortex (spark flex) used to control intake and eject
    photoeye: ?? Used to detect fully loaded algae
    photoeye: model#  class library:
    */

    //TODO run intake method, stop method

    private static AlgaeRoller instance = null;
    
    private SparkFlex algaeDrive;

    private EjectCmd ejectCmd;
    
    private GenericEntry sb_currentCmd;
    private GenericEntry sb_motorVoltage;
    private GenericEntry sb_photoeyeState;

    
    //Command for ejecting algae
    public class EjectCmd extends Command {

        @Override
        public void initialize(){
            setEject();
        }


        @Override
        public void end(boolean interupted){
            setStop();
        }
        
    }

    //Command for intaking algae 
    public class IntakeCmd extends Command{

        @Override
        public void initialize(){
            setIntake();
        }

        @Override
        public void end(boolean interupted){
            setStop();
        }
    }

    /**
     * Constructor
     */
    private AlgaeRoller(){
        algaeDrive = new SparkFlex(Constants.algaeRollerMotor, MotorType.kBrushless);

        ejectCmd = new EjectCmd();

        //Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("End Effector", BuiltInLayouts.kList)
                .withSize(1, 3);

        sb_currentCmd = layout.add("current command", "").getEntry();
        sb_motorVoltage = layout.add("motor voltage", 2960).getEntry();


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
        algaeDrive.setVoltage(voltage);
    }

    /**
     * set motor voltage for ejection
     */
    public void setEject(){
        setMotorVolt(Constants.algaeEjectVolt);
    }

    /**
     * set motor voltage for intake
     */
    public void setIntake(){
        setMotorVolt(Constants.algaeIntakeVolt);
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
        sb_motorVoltage.setDouble(algaeDrive.getBusVoltage() * algaeDrive.getAppliedOutput());
    }

     /**
     * Static Initializer
     */
    public static AlgaeRoller getInstance() {
        if (instance == null) {
            instance = new AlgaeRoller();
        }
        return instance;
    }
}

