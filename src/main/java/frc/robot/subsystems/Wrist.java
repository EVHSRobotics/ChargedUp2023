package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  // Wrist Position encoder setpoints
  // public enum WristPosition{

  //  UP(-5000), MIDDLE(-100000), SHOOTING(-245100), HIGHINTAKE(-250100), STRAIGHT(-150100), STRAIGHTCUBE(-180100), STRAIGHTCONE(-190100), GROUNDCONE(-250000);
   

  //   public double wristSensorPosition;  
  //   private WristPosition(double wristSensorPosition) {
  //     this.wristSensorPosition = wristSensorPosition;
  //   }
  // }
  private double encoderOff = 0;

  public enum WristPosition{
  
    UP(5), MIDDLE(95), SHOOTING(321), HIGHINTAKE(130), STRAIGHTCUBE(330), STRAIGHTCONE(300);
    
 
     private double wristSensorPosition;  
     public double getWrist() {
      return convertWristEncoder(wristSensorPosition);
     }
     private WristPosition(double wristSensorPosition) {
       this.wristSensorPosition = wristSensorPosition;
     }
   }


  
    public TalonFX wrist;
    public CANCoder wristCAN;
    
    // private double baseStartAngle = 102; 
    // private double topStartAngle = 15;
    // private double wristStartAngle = baseStartAngle - topStartAngle; 
    // private double wristAngleongnokizzy = 90; 
    // private final double topArmEncodertickstoangle = 2048*360;
    // private final double bottomArmEncodertickstoangle = 2048*360;    
    // private final double gearratio = 0;
    // private final double kp = 0.001;
    private double lastWristPos = 0;
    private static double wristOffset = 0;
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
        // wrist.setSelectedSensorPosition(0);    
        wristCAN = new CANCoder(Constants.Arm.wristCANID);


        

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
      return convertWristEncoder(wristCAN.getAbsolutePosition());
    }

    private static double convertWristEncoder(double val){
      return (360.0 - (val - wristOffset)) % 360.0;
    }

    // public void checkCycle(){
    //   if(getWristMotorPosition() < 10 && lastWristPos > 350){
    //     encoderOff += 360;
    //   }
    //   if(getWristMotorPosition() > 350 && lastWristPos < 10){
    //     encoderOff -= 360;
    //   }
    // }

    // public void parallelWrist(double topEncoder, double bottomEncoder){
    //     double wristangle = 180 - (topStartAngle + (topEncoder/topArmEncodertickstoangle) + baseStartAngle - (bottomEncoder/bottomArmEncodertickstoangle));

    //     if(wrist.getSelectedSensorPosition() != ((wristStartAngle-wristangle)+wristAngleongnokizzy)*gearratio*2048){
    //       SmartDashboard.putNumber("Wrist Parallel", ((wristStartAngle-wristangle)+wristAngleongnokizzy)*gearratio*2048-wrist.getSelectedSensorPosition()*kp);
    //       SmartDashboard.updateValues();
    //     } 

    // }

    // resets wrist encoders
    // public void resetWristEncoder() {
    //   wrist.setSelectedSensorPosition(0);
    // }

    // PID code for wrist
    // error is difference between expected vs current wrist position
    // p value for wrist with mag encoder should be 0.0016, integrator range 200
    public double setWristPosition(WristPosition wPosition) {
      error = (wPosition.wristSensorPosition) - wristCAN.getAbsolutePosition();
      
      SmartDashboard.putNumber("wrist encoder", wristCAN.getAbsolutePosition());
      SmartDashboard.putNumber("wrist en errorcoder", error);
      SmartDashboard.updateValues();
      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      double errorrate = (error-lasterror)/dt;
      if(Math.abs(lastWristPos - getWristMotorPosition()) < 15){
          errorsum += dt * (lastWristPos - getWristMotorPosition());
      }
      double output = MathUtil.clamp(error*0.015 + errorrate*0.0 + errorsum*0.0, -1, 1);
      double lowerLimit = -2;
      SmartDashboard.putNumber("updateeeeee", wPosition.wristSensorPosition);
      // if(wrist.getSelectedSensorPosition() < lowerLimit && output < 0) output = 0;
      wrist.set(ControlMode.PercentOutput, -output);
      lastTimestamp = Timer.getFPGATimestamp();
      lasterror = error;
      lastWristPos = getWristMotorPosition();
      SmartDashboard.putNumber("out", -output);
      SmartDashboard.updateValues();
      return MathUtil.applyDeadband(output, 0.05);

    }
}