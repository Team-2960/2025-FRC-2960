package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LEDConst;

/**
 * Manages the robot LEDs
 */
public class LEDControl extends SubsystemBase{
    /**
     * Sets the LEDs to the team colors
     */
    public class TeamColorCommand extends Command {
        /**
         * Sets the LEDs to the team colors
         */
        @Override
        public void initialize() {
            setColors(frontLEDs);
            setColors(leftLEDs);
            setColors(rightLEDs);

            leds.setData(ledBuffer);
        }

        /**
         * Sets the colors to alternating white and blue 
         * @param view  buffer view to modify
         */
        public void setColors(AddressableLEDBufferView view) {
            for(int i = 0; i < view.getLength(); i++) {
                if(i % 2 == 0) {
                    view.setRGB(i, 0, 0, 255);
                } else {
                    view.setRGB(i, 255,255, 255);
                }
            }
        }
    }

    /**
     * Command to cycle the LEDS through colors
     */
    public class CycleColors extends WaitCommand {
        private final LEDPattern[] colors;
        private final double period;
        private double start;

        /**
         * Constructor
         * @param duration  Duration of the color cycling
         * @param period    time between color changes in seconds
         * @param colors    list of colors to cycle through
         */
        public CycleColors(double duration, double period, Color ... colors) {
            super(duration);

            // Initialize Timer
            this.period = period;

            // Initialize Colors
            this.colors = new LEDPattern[colors.length];
            for(int i = 0; i < colors.length; i++) {
                this.colors[i] = LEDPattern.solid(colors[i]);
            }
        }

        /**
         * Initialize timer start and set LEDs to first color
         */
        @Override
        public void initialize() {
            start = Timer.getFPGATimestamp();
            colors[0].applyTo(ledBuffer);
            leds.setData(ledBuffer);
        }

        /**
         * Update the colors
         */
        @Override
        public void execute() {
            double runtime = Timer.getFPGATimestamp() - start;
            int colorIndex = (int)(runtime / period) % colors.length;

            colors[colorIndex].applyTo(ledBuffer);
            leds.setData(ledBuffer);
        }
    }

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView frontLEDs;
    private final AddressableLEDBufferView leftLEDs;
    private final AddressableLEDBufferView rightLEDs;

    private final Trigger coralTrigger;
    //TODO Change
    private final int ledPort = 0;

    /**
     * Constructor
     */
    public LEDControl(EndEffector endEffector) {
        // Initialize LEDS
        leds = new AddressableLED(ledPort);
        
        // Initialize LED buffer
        ledBuffer = new AddressableLEDBuffer(
            LEDConst.frontLEDCount + LEDConst.leftLEDCount + LEDConst.rightLEDCount
        );

        // Initialize Left LED buffer view
        leftLEDs = ledBuffer.createView(
            0, 
            LEDConst.leftLEDCount-1
        );
        
        // Initialize Right LED buffer view
        rightLEDs = ledBuffer.createView(
            LEDConst.leftLEDCount, 
            LEDConst.leftLEDCount + LEDConst.rightLEDCount - 1
        );
        
        // Initialize Front LED buffer view
        frontLEDs = ledBuffer.createView(
            LEDConst.leftLEDCount + LEDConst.rightLEDCount, 
            ledBuffer.getLength() - 1
        );

        // Set LED length
        leds.setLength(ledBuffer.getLength());

        // Start LED control
        leds.start();

        // Setup Coral LED blinking 
        coralTrigger = new Trigger(endEffector::isCoralPresent);
        coralTrigger.whileTrue(new CycleColors(1, .1, Color.kWhite, Color.kBlue));

        // Set default command to set the LEDs to alternating blue and white
        setDefaultCommand(new TeamColorCommand());
    }
}