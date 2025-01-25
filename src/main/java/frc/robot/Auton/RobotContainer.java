package frc.robot.Auton;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Auton.Commands.Arm.armToPreset;
import frc.robot.Auton.Commands.Arm.autoAlignArm;
import frc.robot.Auton.Commands.Pizzabox.intakeNote;
import frc.robot.Auton.Commands.Pizzabox.prepShootNote;
import frc.robot.Auton.Commands.Pizzabox.shootNote;
import frc.robot.subsystems.Drive;
public class RobotContainer {  
  Drive drive = Drive.getInstance();
  SendableChooser<Command> autoChooser;
  Command chosenAuto;

  public RobotContainer(){
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Choose Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    NamedCommands.registerCommand("armIntake", new armToPreset("Intake"));
    NamedCommands.registerCommand("armHome", new armToPreset("home"));
    NamedCommands.registerCommand("armSpeaker", new armToPreset("Speaker"));
    NamedCommands.registerCommand("intakeNote", new intakeNote());
    NamedCommands.registerCommand("shootNote", new shootNote());
    NamedCommands.registerCommand("prepShoot", new prepShootNote());
    NamedCommands.registerCommand("armAutoAlign", new autoAlignArm());

    chosenAuto = autoChooser.getSelected();
    
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return chosenAuto;
  }

  
}
