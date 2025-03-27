package frc.robot.simulation;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;

/**
 * Tracks the total current draw of a simulated robot 
 */
public class CurrentSimTracking {
    private static final HashMap<String, MutCurrent> currentList;
    private static final MutCurrent totalCurrent;

    /**
     * Static Initialization block
     */
    static {
        currentList = new HashMap<String, MutCurrent>();
        totalCurrent = Amps.mutable(0);
    }

    /**
     * Sets the current applied by a given device
     * @param name
     * @param current
     */
    public static void updateCurrent(String name, Current current) {
        if(!currentList.containsKey(name)){
            currentList.put(name, current.mutableCopy());
        } else {
            currentList.get(name).mut_replace(current);
        }
    }

    /**
     * Calculates the total current draw on the robot
     * @return
     */
    public static Current getTotalCurrent() {
        double totalAmps = 0;

        for(var current : currentList.values()) {
            totalAmps += current.in(Amps);
        }
        
        return totalCurrent.mut_setMagnitude(totalAmps);
    }
}
