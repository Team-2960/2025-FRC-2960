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

public class IntakePizzaBox extends SubsystemBase {
    public enum PizzaboxState {
        IDLE,
        INTAKE,
        SHOOT,
        FAST_SHOOT,
        SHOOT_PREP,
        REVERSE
    }

    private static IntakePizzaBox intake = null;

    private TalonFX intakeRollers;

    private SparkFlex shooterTop;
    private SparkFlex shooterBot;

    private RelativeEncoder shootEncoder1;
    private RelativeEncoder shootEncoder2;

    private DigitalInput shooterPhotoeye;
    private DigitalInput intakePhotoeye;

    private PizzaboxState state;

    private GenericEntry sb_state;
    private GenericEntry sb_shooterTopVolt;
    private GenericEntry sb_shooterBotVolt;
    private GenericEntry sb_shooterTopRate;
    private GenericEntry sb_shooterBotRate;
    private GenericEntry sb_intakeRollerVolt;
    private GenericEntry sb_intakeRollerCurrent;
    private GenericEntry sb_intakeRollerRate;
    private GenericEntry sb_shooterNotePresent;
    private GenericEntry sb_intakeNotePresent;


    /**
     * Constructor
     */
    private IntakePizzaBox() {
        // Initialize Intake Motor
        intakeRollers = new TalonFX(Constants.intakeRollers);

        // Initialize Shooter Motors
        shooterTop = new SparkFlex(Constants.shooterTop, MotorType.kBrushless);
        shooterBot = new SparkFlex(Constants.shooterBot, MotorType.kBrushless);
        shooterTop.configure(new SparkFlexConfig().inverted(true), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Shooter Encoders
        shootEncoder1 = shooterTop.getEncoder();
        shootEncoder2 = shooterBot.getEncoder();

        // Initialize shooterPhotoeye
        shooterPhotoeye = new DigitalInput(3);
        intakePhotoeye = new DigitalInput(5);

        // Initialize state
        state = PizzaboxState.IDLE;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Pizzabox", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_state = layout.add("State", state.name()).getEntry();
        sb_shooterTopVolt = layout.add("Shooter 1 Voltage", 0).getEntry();
        sb_shooterBotVolt = layout.add("Shooter 2 Voltage", 0).getEntry();
        sb_shooterTopRate = layout.add("Shooter 1 Rate", 0).getEntry();
        sb_shooterBotRate = layout.add("Shooter 2 Rate", 0).getEntry();
        sb_intakeRollerVolt = layout.add("Intake Roller Voltage", 0).getEntry();
        sb_intakeRollerRate = layout.add("Intake Roller Rate", 0).getEntry();
        sb_shooterNotePresent = layout.add("Shooter Note Present", false).getEntry();
        sb_intakeNotePresent = layout.add("Intake Note Present", false).getEntry();
        sb_intakeRollerCurrent = layout.add("Intake Roller Current", 0).getEntry();

    }

    /**
     * Sets the Pizzabox State
     * 
     * @param new pizzabox state
     */
    public void setState(PizzaboxState state) {
        this.state = state;
    }

    /**
     * Gets the pizzabox state
     * 
     * @return current pizza box state
     */
    public PizzaboxState getState() {
        return state;
    }

    /**2
     * Checks if a game piece is present
     * 
     * @return true if a gamepiece is present, false otherwise
     */
    public void runShooter(double voltage){
        shooterTop.setVoltage(voltage);
        shooterBot.setVoltage(voltage);
    }
    
     public boolean isNotePresent() {
        return shooterPhotoeye.get();
    }

    public boolean isIntakeNotePresent(){
        return intakePhotoeye.get();
    }

    /**
     * Subsystem periodic function
     */
    @Override
    public void periodic() {
        if (state == PizzaboxState.INTAKE) {
            // Check if a gamepiece is present
            if (isNotePresent()) {
                intakeRollers.setVoltage(0);
                if(Arm.getInstance().getArmAngle().getDegrees() <= 10) Arm.getInstance().setStateCommand("home");
            }else if (isIntakeNotePresent()) {
                intakeRollers.setVoltage(Constants.intakeSlowVoltage);
            }else{
                intakeRollers.setVoltage(Constants.intakeInVoltage);
            }
            
        } else if (state == PizzaboxState.SHOOT_PREP) {
            runShooter(Constants.shooterPrepPower); // Turn shooter to idle speed
        } else if (state == PizzaboxState.SHOOT) {
            runShooter(Constants.shooterShootVoltage);// Turn shooter to max Voltage

            // Check if shooter is ready to shoot
            if (shootEncoder1.getVelocity() > Constants.shooterMinShootSpeed
                    && shootEncoder2.getVelocity() > Constants.shooterMinShootSpeed) {
                intakeRollers.setVoltage(Constants.intakeInVoltage); // Run intake
            } else {
                intakeRollers.setVoltage(0); // Turn Intake Off
            }

        } else if (state == PizzaboxState.FAST_SHOOT) {
            runShooter(Constants.shooterShootVoltage); // Turn shooter to max Voltage

            // Check if shooter is ready to shoot
            if (shootEncoder1.getVelocity() > Constants.shooterFastShootSpeed
                    && shootEncoder2.getVelocity() > Constants.shooterFastShootSpeed) {
                intakeRollers.setVoltage(Constants.intakeInVoltage); // Run intake
            } else {
                intakeRollers.setVoltage(0); // Turn Intake Off
            }

        } else if (state == PizzaboxState.REVERSE) {
            // Reverse shooter and intake
            runShooter(-Constants.shooterRevVoltage * 0.5);
            intakeRollers.setVoltage(-Constants.intakeOutVoltage);
        } else {
            // Turn shooter and intake off
            runShooter(0);
            intakeRollers.setVoltage(0);
        }

        updateUI();
    }

    private void updateUI() {
        sb_state.setString(state.name());
        sb_shooterTopVolt.setDouble(shooterTop.getBusVoltage() * shooterTop.getAppliedOutput());
        sb_shooterBotVolt.setDouble(shooterTop.getBusVoltage() * shooterTop.getAppliedOutput());
        sb_shooterTopRate.setDouble(shootEncoder1.getVelocity());
        sb_shooterBotRate.setDouble(shootEncoder2.getVelocity());
        sb_intakeRollerVolt.setDouble(intakeRollers.getMotorVoltage().getValueAsDouble());
        sb_intakeRollerRate.setDouble(intakeRollers.getVelocity().getValueAsDouble());
        sb_shooterNotePresent.setBoolean(isNotePresent());
        sb_intakeNotePresent.setBoolean(isIntakeNotePresent());
        sb_intakeRollerCurrent.setDouble(intakeRollers.getStatorCurrent().getValueAsDouble());
    }

    /**
     * Static Initializer
     */
    public static IntakePizzaBox getInstance() {
        if (intake == null) {
            intake = new IntakePizzaBox();
        }
        return intake;
    }
}