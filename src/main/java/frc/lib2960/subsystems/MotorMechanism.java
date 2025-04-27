package frc.lib2960.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.util.DistanceTrapezoidProfile;

public abstract class MotorMechanism extends SubsystemBase{

    private final MotorMechanismSettings settings;
    private PIDController pidController;
    private SimpleMotorFeedforward ffController;

    public class HoldCommand extends Command{

        public HoldCommand(){
            addRequirements(MotorMechanism.this);
        }

        @Override
        public void execute(){
            setPosition(getPosition());
        }

    }

    public class setVoltageCommand extends Command{
        
        private Voltage voltage;

        public setVoltageCommand(Voltage voltage){
            this.voltage = voltage;
            addRequirements(MotorMechanism.this);
        }

        public void setTargetVoltage(Voltage voltage){
            this.voltage = voltage;
        }

        @Override
        public void execute(){
            setMotorVoltage(voltage.in(Volts));
        }
    }

    public class SetRateCommand extends Command{

        private AngularVelocity angularVelocity;

        public SetRateCommand(AngularVelocity angularVelocity){
            this.angularVelocity = angularVelocity;
            addRequirements(MotorMechanism.this);
        }

        public void seTargettRate(AngularVelocity angularVelocity){
            this.angularVelocity = angularVelocity;
        }

        @Override
        public void execute(){
            setRate(angularVelocity);
        }

    }

    public class SetPositionCommand extends Command{

        private Angle position;
        private boolean finishes;
        private Angle tolerance;

        public SetPositionCommand(Angle position){
            this.position = position;
            this.finishes = false;
            this.tolerance = Angle.ofBaseUnits(0, Rotations);
            addRequirements(MotorMechanism.this);
        }

        public SetPositionCommand(Angle position, boolean finishes, Angle tolerance){
            this.position = position;
            this.finishes = finishes;
            this.tolerance = tolerance;
            addRequirements(MotorMechanism.this);
        }


        public void setTargetPositionFinishes(Angle position, boolean finishes, Angle tolerance){
            this.position = position;
            this.finishes = finishes;
            this.tolerance = tolerance;
        }
        
        public void setTargetPosition(Angle position){
            this.position = position;
            this.finishes = false;
            this.tolerance = Angle.ofBaseUnits(0, Rotations);
        }

        @Override
        public void execute(){
            setPosition(position);
        }

        @Override
        public boolean isFinished(){
            if (finishes){
                return Math.abs(position.in(Rotations) - getPosition().in(Rotations)) <= tolerance.in(Rotations);
            }else{
                return false;
            }
        }


    }


    public MotorMechanism(MotorMechanismSettings motorMechanismSettings, int motorCount){
        settings = motorMechanismSettings;
        pidController = motorMechanismSettings.pidController;
        ffController = settings.ffController;
        if (settings.defaultHold){
            setDefaultCommand(new HoldCommand());
        }
    }

    private void setRate(AngularVelocity rate){
        setMotorVoltage(
            pidController.calculate(getRate().in(RotationsPerSecond), rate.in(RotationsPerSecond))
            +
            ffController.calculateWithVelocities(getRate().in(RotationsPerSecond), rate.in(RotationsPerSecond))
        );
    }

    private void setPosition(Angle position){
        AngularVelocity rate = RotationsPerSecond.of(
            new DistanceTrapezoidProfile(
                settings.trapezoidalRampDistance, 
                settings.maxSpeed.in(RotationsPerSecond)
            )
            .calculate(getPosition().in(Rotations), position.in(Rotations)));
        
        setRate(rate);
    }


    
    
    public abstract Voltage getVoltage();
    public abstract AngularVelocity getRate();
    public abstract Angle getPosition();
    public abstract void setMotorVoltage(double voltage);


    


}
