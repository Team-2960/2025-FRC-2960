package frc.robot.subsystems;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevArmControl extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private Command holdCommand;
    
    private double elevTopLimit;
    private double elevBottomLimit;
    private Rotation2d armTopLimit;
    private Rotation2d armBottomLimit;

    public class setElevArmCommand extends Command{
        double elevatorPos;
        Rotation2d armRot;
        double elevClearance;

        public setElevArmCommand(double elevatorPos, Rotation2d armRot, double elevClearance){
            this.elevatorPos = elevatorPos;
            this.armRot = armRot;
            this.elevClearance = elevClearance;
        }

        public void setPositions(double elevatorPos, Rotation2d armRot, double elevClearance){
            this.elevatorPos = elevatorPos;
            this. armRot = armRot;
            this.elevClearance = elevClearance;
        }

        @Override
        public void initialize(){
            setElevator(elevatorPos);
        }
        
        @Override
        public void execute(){
            setArm(armRot, elevClearance);
            
            setElevLimit(elevBottomLimit, elevatorPos, true);
            setElevLimit(elevTopLimit, elevatorPos, false);
            setArmLimit(armBottomLimit, armRot, true);
            setArmLimit(armTopLimit, armRot, false);
        }
    }

    public class ElevArmHoldCommand extends Command{

    }


    public ElevArmControl(){
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();

        //TODO Change values to actual elevator limits
        elevTopLimit = 0;
        elevBottomLimit = 0;
        armTopLimit = new Rotation2d();
        armBottomLimit = new Rotation2d();

    }

    public void setElevator(double elevatorPos){
        elevator.setPosCommand(elevatorPos);
    }

    /**
     * @param armRot
     * The target arm rotation
     * @param elevClearance
     * The position the elevator has to be at for the arm to begin going to an angle
     */
    public void setArm(Rotation2d armRot, double elevClearance){
        if (elevator.getElevatorPos() > elevClearance){
            arm.setAngleCommand(armRot);
        }
    }

    public void setHoldCommand(){
        elevator.setHoldCommand();
        arm.setHoldCommand();
    }


    /**
     * @param elevPosLim
     * The position limit of the elevator
     * @param targetPos
     * The target position of the elevator, used to check if your targetPos is less than the maximum/minimum position
     * @param lessThan
     * true if you want to stop the elevator when the position is less than the limit
     */
    public void setElevLimit(double elevPosLim, double targetPos, boolean lessThan){
        if(lessThan){
            if (elevator.getElevatorPos() <= elevPosLim && targetPos < elevPosLim){
                elevator.setHoldCommand();
            }
        }else{
            if (elevator.getElevatorPos() >= elevPosLim && targetPos > elevPosLim){
                elevator.setHoldCommand();
            }
        }
    }

    /**
     * @param armRotLimit
     * The rotation limit of the arm
     * @param targetAngle
     * The target angle of the arm, used to check if your targetAnlge is less than the maximum/minimum angle
     * @param lessThan
     * true if you want to stop the arm when the angle is less than the limit
     */
    public void setArmLimit(Rotation2d armRotLimit, Rotation2d targetAngle, boolean lessThan){
        if(lessThan){
            if (arm.getArmAngle().getDegrees() <= armRotLimit.getDegrees() && targetAngle.getDegrees() < armRotLimit.getDegrees()){
                elevator.setHoldCommand();
            }
        }else{
            if (arm.getArmAngle().getDegrees() >= armRotLimit.getDegrees() && targetAngle.getDegrees() > armRotLimit.getDegrees()){
                elevator.setHoldCommand();
            }
        }
    }
    
}
