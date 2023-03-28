package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class PIDController {
    private double lasterror = 0;
    private double error;
    private double errorrate;
    private double errorsum = 0;
    private double output;
    private double p;
    private double i;
    private double d;
    private double lastTimestamp = 0;
    private double time;
    private double setIntegratorRange;
    private double tolerance = 2;
    private double bound;

    public PIDController(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }
    public void setIntegratorRange(double range){
        setIntegratorRange = range;
    }

    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }
    public void setBound(double bound){
        this.bound = Math.abs(bound);
    }



    // returns the appropriate power value
    public double calculate(double newError){
        
        error = newError;
        
        time = Timer.getFPGATimestamp() - lastTimestamp;
        if(Math.abs(error) < setIntegratorRange){
            errorsum += error * time;
        }

        errorrate = (error - lasterror)/time;

        lasterror = error;
        lastTimestamp = time;

        if(Math.abs(error) > tolerance)
            output = MathUtil.clamp(p * error + i * errorsum + d * errorrate, -bound, bound);
        else 
            output = 0;
        
        return output;
    }
}
