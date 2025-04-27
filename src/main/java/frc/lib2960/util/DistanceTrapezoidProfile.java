package frc.lib2960.util;

public class DistanceTrapezoidProfile {

    private double rampDistance;
    private double maxSpeed;
    
    public DistanceTrapezoidProfile(double rampDistance, double maxSpeed){
        this.rampDistance = rampDistance;
        this.maxSpeed = maxSpeed;
    }

    public double calculate(double current, double target){
        double angleError = target - current;

        double targetSpeed = maxSpeed * (angleError > 0 ? 1 : -1);
        double rampDownSpeed = angleError / rampDistance * maxSpeed;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed))
            targetSpeed = rampDownSpeed;

        return targetSpeed;
    }
}
