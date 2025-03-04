package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevArmControl extends SubsystemBase{
    private static ElevArmControl instance = null;

    public final Command goToIntake;    /**< Command sequence to move to intake position */
    public final Command goToL1;        /**< Command sequence to move to the L1 scoring position */
    public final Command goToL2;        /**< Command sequence to move to the L2 scoring position */
    public final Command goToL3;        /**< Command sequence to move to the L3 scoring position */
    public final Command goToL4;        /**< Command sequence to move to the L4 scoring position */
    public final Command goToLowAlgae;  /**< Command sequence to move to the low algae removal position */
    public final Command goToHighAlgae; /**< Command sequence to move to the high algae removal position   */  
    public final Map<String, Command> elevArmCommands; 

    public final Arm arm;
    public final Elevator elev;

    /**
     * Constructor
     */
    private ElevArmControl(){
        arm = Arm.getInstance();
        elev = Elevator.getInstance();
        elevArmCommands = new HashMap<String, Command>();
        elevArmCommands.put("goToIntake", new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevIntakePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armIntakeAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        ));

        elevArmCommands.put("goToL1", new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevIntakePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armIntakeAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        ));

        elevArmCommands.put("goToL2", new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL2Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        ));

        elevArmCommands.put("goToL3", new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL3Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        ));

        elevArmCommands.put("goToL4", new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL3Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        ));

        elevArmCommands.put("goToLowAlgae", new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevLowAlgaePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armAlgaeRemoveAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        ));

        elevArmCommands.put("goToHighAlgae", new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevHighAlgaePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armAlgaeRemoveAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        ));

        goToIntake = elevArmCommands.get("goToIntake");
        goToL1 = elevArmCommands.get("goToL1");
        goToL2 = elevArmCommands.get("goToL2");
        goToL3 = elevArmCommands.get("goToL3");
        goToL4 = elevArmCommands.get("goToL4");
        goToLowAlgae = elevArmCommands.get("goToLowAlgae");
        goToHighAlgae = elevArmCommands.get("goToHighAlgae");
    }

    public void goToIntakePos() {
        goToIntake.schedule();
    }

    public void goToL1Pos() {
        goToL1.schedule();
    }

    public void goToL2Pos() {
        goToL2.schedule();
    }

    public void goToL3Pos() {
        goToL3.schedule();
    }

    public void goToL4Pos() {
        goToL4.schedule();
    }

    public void goToLowAlgaePos() {
        goToLowAlgae.schedule();
    }

    public void goToHighAlgaePos() {
        goToHighAlgae.schedule();
    }

    public Command getGoToIntakeCommand(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevIntakePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armIntakeAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }

    public Command getGoToL1Command(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevIntakePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armIntakeAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }

    public Command getGoToL2Command(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL2Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }


    public Command getGoToL3Command(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL3Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }


    public Command getGoToL4Command(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL3Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }

    public Command getGoToLowAlgaeCommand(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevLowAlgaePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armAlgaeRemoveAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }

    public Command getGoToHighAlgaeCommand(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevHighAlgaePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armAlgaeRemoveAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }


    /**
     * Singleton Instance access
     * @return  the instance of the ElevArmControl class
     */
    public static ElevArmControl getInstance() {
        if(instance == null) instance = new ElevArmControl();

        return instance;
    } 
    
}
