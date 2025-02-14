package frc.robot.Auton;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
public class RobotContainer {  
  Drive drive = Drive.getInstance();
  Arm arm = Arm.getInstance();
  SendableChooser<Command> autoChooser;
  Command chosenAuto;

  public RobotContainer(){
    autoChooser = AutoBuilder.buildAutoChooser();
    NamedCommands.registerCommand("armLvl1", arm.new ArmAngleCommand(Rotation2d.fromDegrees(0)));
    NamedCommands.registerCommand("driveAlignCommand", drive.new LinearDriveCommands().new GoToPointCommand(new Translation2d()));
    SmartDashboard.putData("Choose Auto", autoChooser);
  }

  public Command getAutonomousCommand() {

    chosenAuto = autoChooser.getSelected();
    
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return chosenAuto;
  }

  
}