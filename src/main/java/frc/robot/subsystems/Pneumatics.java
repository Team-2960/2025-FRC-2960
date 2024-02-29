package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class Pneumatics extends SubsystemBase {

    private static Pneumatics instance = null;

    public Compressor compressor;

    private GenericEntry sb_pressure;
    private GenericEntry sb_current;

    private Pneumatics() {
        compressor = new Compressor(20,PneumaticsModuleType.REVPH);
        compressor.enableAnalog(80, 120);

        var layout = Shuffleboard.getTab("Status").getLayout("Pneumatics");
        sb_pressure = layout.add("Pressure");
        sb_current = layout.add("Current");
    }

    @Override
    public void periodic() {
        updateUI();
    }

    public void updateUI() {
        sb_pressure.setDouble(compressor.getPressure());
        sb_current.setDouble(compressor.getCurrent());
    }

    public static Pneumatics getInstance() {
        if(instance == null) instance = new Pneumatics();

        return instance;
    }

}
    