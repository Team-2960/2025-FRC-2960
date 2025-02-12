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

    public class setElevArmCommand extends Command{
        double elevatorPos;
        Rotation2d armRot;

        public setElevArmCommand(double elevatorPos, Rotation2d armRot){
            this.elevatorPos = elevatorPos;
            this.armRot = armRot;
        }

        public void setPositions(double elevatorPos, Rotation2d armRot){
            this.elevatorPos = elevatorPos;
            this. armRot = armRot;
        }

        @Override
        public void initialize(){
            setElevArm(elevatorPos, armRot);
        }
    }

    public class ElevArmHoldCommand extends Command{

    }


    public ElevArmControl(){
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();

    }

    public void setElevArm(double elevatorPos, Rotation2d armRot){
        arm.setAngleCommand(armRot);
        elevator.setPosCommand(elevatorPos);
    }

    public void setHoldCommand(){
        elevator.setHoldCommand();
        arm.setHoldCommand();
    }
    
}
