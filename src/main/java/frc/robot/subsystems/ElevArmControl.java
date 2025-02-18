package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevArmControl extends SubsystemBase{
    private static ElevArmControl instance = null;

    public final Command gotoIntake;    /**< Command sequence to move to intake position */
    public final Command gotoL1;        /**< Command sequence to move to the L1 scoring position */
    public final Command gotoL2;        /**< Command sequence to move to the L2 scoring position */
    public final Command gotoL3;        /**< Command sequence to move to the L3 scoring position */
    public final Command gotoL4;        /**< Command sequence to move to the L4 scoring position */
    public final Command gotoLowAlgae;  /**< Command sequence to move to the low algae removal position */
    public final Command gotoHighAlgae; /**< Command sequence to move to the high algae removal position   */   

    /**
     * Constructor
     */
    private ElevArmControl(){
        Arm arm = Arm.getInstance();
        Elevator elev = Elevator.getInstance();

        gotoIntake = new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevIntakePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armIntakeAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );

        gotoL1 = new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL1Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );

        gotoL2 = new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL2Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );

        gotoL3 = new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL3Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );

        gotoL4 = new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevL3Pos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armCoralScoreAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );

        gotoLowAlgae = new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevLowAlgaePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armAlgaeRemoveAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );

        gotoHighAlgae = new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armTravelAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(Constants.elevHighAlgaePos)),
            new ParallelRaceGroup(arm.new ArmAngleCommand(Constants.armAlgaeRemoveAngle), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorHoldCommand())
        );
    }

    public void gotoIntakePos() {
        gotoIntake.schedule();
    }

    public void gotoL1Pos() {
        gotoL1.schedule();
    }

    public void gotoL2Pos() {
        gotoL2.schedule();
    }

    public void gotoL3Pos() {
        gotoL3.schedule();
    }

    public void gotoL4Pos() {
        gotoL4.schedule();
    }

    public void gotoLowAlgaePos() {
        gotoLowAlgae.schedule();
    }

    public void gotoHighAlgaePos() {
        gotoHighAlgae.schedule();
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
