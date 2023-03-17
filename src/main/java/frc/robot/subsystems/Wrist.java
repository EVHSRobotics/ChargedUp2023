package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  // Wrist Position encoder setpoints
  public enum WristPosition{

   UP(-5000), MIDDLE(-100000), SHOOTING(-265100), HIGHINTAKE(-250100), STRAIGHT(-150100), STRAIGHTCUBE(-180100), STRAIGHTCONE(-190100), GROUNDCONE(-250000);
   

    public double wristSensorPosition;  
    private WristPosition(double wristSensorPosition) {
      this.wristSensorPosition = wristSensorPosition;
    }
  }
  
    public TalonFX wrist;
    
    private double baseStartAngle = 102; 
    private double topStartAngle = 15;
    private double wristStartAngle = baseStartAngle - topStartAngle; 
    private double wristAngleongnokizzy = 90; 
    private final double topArmEncodertickstoangle = 2048*360;
    private final double bottomArmEncodertickstoangle = 2048*360;    
    private final double gearratio = 0;
    private final double kp = 0.001;
    private double lastWristPos = 0;
    double errorsum = 0;
    double lasterror = 0;
    double error;

    double lastTimestamp;
    
    // setpoints for mag encoder
    // public enum WristPosition{

    //   UP(-5000*2/100), MIDDLE(-100000*2/100), SHOOTING(-235100*2/100), HIGHINTAKE(-250100*2/100), STRAIGHT(-150100*2/100), STRAIGHTCUBE(-160100*2/100), STRAIGHTCONE(-170100*2/100), GROUNDCONE(-250000*2/100);
   
    //    public double wristSensorPosition;
    //    private WristPosition(double wristSensorPosition) {
    //      this.wristSensorPosition = wristSensorPosition;
    //    }
    //  }
    // Resets all wristand sets encoder pos to 0
    public Wrist(){
        wrist = new TalonFX(Constants.Arm.wristMotor);    
        // wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        wrist.setSelectedSensorPosition(0);    

        

        wrist.setNeutralMode(NeutralMode.Brake);
        lastWristPos = 0.0;
    }
    // move wrist based on speed
    public void moveWrist(double speed){
      
        // lastWristPos = getWristMotorPosition();
        wrist.set(ControlMode.PercentOutput, speed);

    }


    // gets the wrist position
    public double getWristMotorPosition() {
      return wrist.getSelectedSensorPosition();
    }

    public void parallelWrist(double topEncoder, double bottomEncoder){
        double wristangle = 180 - (topStartAngle + (topEncoder/topArmEncodertickstoangle) + baseStartAngle - (bottomEncoder/bottomArmEncodertickstoangle));

        if(wrist.getSelectedSensorPosition() != ((wristStartAngle-wristangle)+wristAngleongnokizzy)*gearratio*2048){
          SmartDashboard.putNumber("Wrist Parallel", ((wristStartAngle-wristangle)+wristAngleongnokizzy)*gearratio*2048-wrist.getSelectedSensorPosition()*kp);
          SmartDashboard.updateValues();
        } 

    }

    // resets wrist encoders
    public void resetWristEncoder() {
      wrist.setSelectedSensorPosition(0);
    }

    // PID code for wrist
    // error is difference between expected vs current wrist position
    // p value for wrist with mag encoder should be 0.0016, integrator range 200
    public double setWristPosition(WristPosition wPosition) {

      error = wPosition.wristSensorPosition - wrist.getSelectedSensorPosition();

      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      double errorrate = (error-lasterror)/dt;
      if(Math.abs(lastWristPos - getWristMotorPosition()) < 10000){
          errorsum += dt * (lastWristPos - getWristMotorPosition());
      }
      double output = MathUtil.clamp(error*0.000008 + errorrate*0.0 + errorsum*0.0, -1, 1);
      double lowerLimit = -300000;
      if(wrist.getSelectedSensorPosition() < lowerLimit && output < 0) output = 0;
      wrist.set(ControlMode.PercentOutput, output);
      lastTimestamp = Timer.getFPGATimestamp();
      lasterror = error;
      lastWristPos = wrist.getSelectedSensorPosition();
      SmartDashboard.putNumber("out", output);
      SmartDashboard.updateValues();
      return MathUtil.applyDeadband(output, 0.05);

    }
}