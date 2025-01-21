package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    public enum IntakeState {
        IDLE,
        INTAKE,
        PUSH,
        REVERSE
    }

    private static Intake intake = null;

    private SparkFlex intakeRollers;

    private DigitalInput intakePhotoeye;

    private IntakeState state;

    private GenericEntry sb_state;
    private GenericEntry sb_intakeRollerVolt;
    private GenericEntry sb_intakeRollerCurrent;
    private GenericEntry sb_intakeRollerRate;
    private GenericEntry sb_intakeNotePresent;


    /**
     * Constructor
     */
    private Intake() {
        // Initialize Intake Motor
        intakeRollers = new SparkFlex(Constants.intakeRollers, MotorType.kBrushless);



        // Initialize 
        intakePhotoeye = new DigitalInput(5);

        // Initialize state
        state = IntakeState.IDLE;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Pizzabox", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_state = layout.add("State", state.name()).getEntry();
        sb_intakeRollerVolt = layout.add("Intake Roller Voltage", 0).getEntry();
        sb_intakeRollerRate = layout.add("Intake Roller Rate", 0).getEntry();
        sb_intakeNotePresent = layout.add("Intake Note Present", false).getEntry();
        sb_intakeRollerCurrent = layout.add("Intake Roller Current", 0).getEntry();

    }

    /**
     * Sets the Pizzabox State
     * 
     * @param new pizzabox state
     */
    public void setState(IntakeState state) {
        this.state = state;
    }

    /**
     * Gets the pizzabox state
     * 
     * @return current pizza box state
     */
    public IntakeState getState() {
        return state;
    }

    /**
     * Checks if a game piece is present
     * 
     * @return true if a gamepiece is present, false otherwise
     */
    

    public boolean isIntakeNotePresent(){
        return intakePhotoeye.get();
    }

    /**
     * Subsystem periodic function
     */
    @Override
    public void periodic() {
        updateUI();
    }

    private void updateUI() {
        sb_state.setString(state.name());
        sb_intakeRollerVolt.setDouble(intakeRollers.getBusVoltage());
        sb_intakeRollerRate.setDouble(intakeRollers.getEncoder().getVelocity());
        sb_intakeNotePresent.setBoolean(isIntakeNotePresent());
        sb_intakeRollerCurrent.setDouble(intakeRollers.getOutputCurrent());
    }

    /**
     * Static Initializer
     */
    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }
}