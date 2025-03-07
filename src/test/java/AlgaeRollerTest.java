import static frc.robot.Constants.algaeRollerMotor;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.AlgaeRoller;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AlgaeRollerTest {
    AlgaeRoller algaeRoller;
    SparkFlexSim algaeDriveSim;

    @BeforeEach
    void setup() {
        // Initialize AlgaeRoller 
        algaeRoller = new AlgaeRoller();

        // Initialize Simulation motor
        algaeDriveSim = new SparkFlexSim(algaeRoller.algaeDrive, DCMotor.getNeoVortex(1));
    }
    
    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void TearDown() throws Exception {
        algaeRoller.close();
    }
}
