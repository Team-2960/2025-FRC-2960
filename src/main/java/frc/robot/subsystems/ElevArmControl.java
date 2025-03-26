package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConst;
import frc.robot.Constants.ElevConst;

public class ElevArmControl extends SubsystemBase{
    public final Command gotoIntake;    /**< Command sequence to move to intake position */
    public final Command gotoL1;        /**< Command sequence to move to the L1 scoring position */
    public final Command gotoL2;        /**< Command sequence to move to the L2 scoring position */
    public final Command gotoL3;        /**< Command sequence to move to the L3 scoring position */
    public final Command gotoL4;        /**< Command sequence to move to the L4 scoring position */
    public final Command gotoLowAlgae;  /**< Command sequence to move to the low algae removal position */
    public final Command gotoHighAlgae; /**< Command sequence to move to the high algae removal position   */  
     
    private Arm arm;
    private Elevator elev;

    /**
     * Constructor
     */
    public ElevArmControl(Arm arm, Elevator elevator){
        this.arm = arm;
        this.elev = elevator;

        gotoIntake = getGoToIntakeCommand();
        gotoL1 = getGoToL1Command();
        gotoL2 = getGoToL2Command();
        gotoL3 = getGoToL3Command();
        gotoL4 = getGoToL4Command();
        gotoLowAlgae = getGoToLowAlgaeCommand();
        gotoHighAlgae = getGoToHighAlgaeCommand();
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

    public Command getGoToIntakeCommand(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.travelAngle)), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(ElevConst.intakePos.in(Inches))),
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.intakeAngle)), elev.new ElevatorHoldCommand())
        );
    }

    public Command getGoToL1Command(){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.travelAngle)), elev.new ElevatorHoldCommand()),
            new ParallelRaceGroup(arm.new ArmHoldCommand(), elev.new ElevatorPosCommand(ElevConst.L1Pos.in(Inches))),
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.coralScoreAngle)), elev.new ElevatorHoldCommand())
        );
    }

    public Command getGoToL2Command(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(elev.new ElevatorPosCommand(ElevConst.L2Pos.in(Inches)), arm.new ArmAngleCommand(new Rotation2d(ArmConst.travelAngle))),
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.coralScoreAngle)), elev.new ElevatorHoldCommand())
        );
    }


    public Command getGoToL3Command(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(elev.new ElevatorPosCommand(ElevConst.L3Pos.in(Inches)), arm.new ArmAngleCommand(new Rotation2d(ArmConst.travelAngle))),
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.coralScoreAngle)), elev.new ElevatorHoldCommand())
        );
    }


    public Command getGoToL4Command(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(elev.new ElevatorPosCommand(ElevConst.L4Pos.in(Inches)), arm.new ArmAngleCommand(new Rotation2d(ArmConst.travelAngle))),
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.coralL4Angle)), elev.new ElevatorHoldCommand())
        );
    }
    public Command getGoToLowAlgaeCommand(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(elev.new ElevatorPosCommand(ElevConst.lowAlgaePos.in(Inches)), arm.new ArmAngleCommand(new Rotation2d(ArmConst.travelAngle))),
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.algaeRemoveAngle)), elev.new ElevatorHoldCommand())
        );
    }

    public Command getGoToHighAlgaeCommand(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(elev.new ElevatorPosCommand(ElevConst.highAlgaePos.in(Inches)), arm.new ArmAngleCommand(new Rotation2d(ArmConst.travelAngle))),
            new ParallelRaceGroup(arm.new ArmAngleCommand(new Rotation2d(ArmConst.algaeRemoveAngle)), elev.new ElevatorHoldCommand())
        );
    }
}