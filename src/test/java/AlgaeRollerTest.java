import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeRoller;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AlgaeRollerTest {
    static final double DELTA = 1e-2; // acceptable deviation range

    AlgaeRoller algaeRoller;
    SparkFlexSim algaeDriveSim;

    @BeforeEach
    void setup() {
        // Initialize AlgaeRoller 
        algaeRoller = new AlgaeRoller();
        algaeDriveSim = algaeRoller.algaeDriveSim;
    }
    
    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void TearDown() throws Exception {
        algaeRoller.close();
    }

    @Test
    void setMotorVolt() {
        algaeRoller.setMotorVolt(12);
        double result = algaeDriveSim.getAppliedOutput() * algaeDriveSim.getBusVoltage();
        System.out.println(result);
        assertEquals(12, result, DELTA);

        algaeRoller.setMotorVolt(-12);
        result = algaeDriveSim.getAppliedOutput() * algaeDriveSim.getBusVoltage();
        assertEquals(-12, result, DELTA);

        algaeRoller.setMotorVolt(0);
        result = algaeDriveSim.getAppliedOutput() * algaeDriveSim.getBusVoltage();
        assertEquals(0, result, DELTA);
    }

    @Test
    void setEject() {
        algaeRoller.setEject();
        double result = algaeDriveSim.getAppliedOutput() * algaeDriveSim.getBusVoltage();
        assertEquals(Constants.algaeEjectVolt, result, DELTA);
    }

    @Test
    void setIntake() {
        algaeRoller.setIntake();
        double result = algaeDriveSim.getAppliedOutput() * algaeDriveSim.getBusVoltage();
        assertEquals(Constants.algaeIntakeVolt, result, DELTA);
    }

    @Test
    void setStop() {
        algaeRoller.setIntake();
        algaeRoller.setStop();
        double result = algaeDriveSim.getAppliedOutput() * algaeDriveSim.getBusVoltage();
        assertEquals(0, result, DELTA);
    }

    @Test
    void runEject() {
        algaeRoller.runEject();
        Command currentCmd = algaeRoller.getCurrentCommand();

        assertNotNull(currentCmd);
        assertEquals("EjectCmd", currentCmd.getName());
    }

    @Test
    void runIntake() {
        algaeRoller.runIntake();
        Command currentCmd = algaeRoller.getCurrentCommand();

        assertNotNull(currentCmd);
        assertEquals("IntakeCmd", currentCmd.getName());
    }

    @Test
    void stop() {
        algaeRoller.runEject();
        algaeRoller.stop();
        Command currentCmd = algaeRoller.getCurrentCommand();

        assertNull(currentCmd);
    }
}
