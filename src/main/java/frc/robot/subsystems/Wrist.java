package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  public enum WristPosition{

   UP(0), SHOOTING(-31000), STRAIGHT(-20500);

    public double wristSensorPosition;
    private WristPosition(double wristSensorPosition) {
      this.wristSensorPosition = wristSensorPosition;
    }
  }

    public WristPosition currentWristPosition = WristPosition.UP;
    public TalonFX wrist;
    
    private double baseStartAngle = 102; 
    private double topStartAngle = 15;
    private double wristStartAngle = baseStartAngle - topStartAngle; 
    private double wristAngleongnokizzy = 90; 
    private final double topArmEncodertickstoangle = 2048*360;
    private final double bottomArmEncodertickstoangle = 2048*360;    
    private final double gearratio = 1;    
    private final double kp = 0.001;
    private double lastWristPos = 0;
    double errorsum = 0;
    double lasterror = 0;
    double error;

    double lastTimestamp;

    
    public Wrist(){
        wrist = new TalonFX(Constants.Arm.wristMotor);    
        wrist.setSelectedSensorPosition(0);    

        wrist.setNeutralMode(NeutralMode.Brake);
        lastWristPos = 0.0;
    }
    public void moveWrist(double speed){
      
      if (Math.abs(speed) >= 0.05) {
        lastWristPos = getWristMotorPosition();
        wrist.set(ControlMode.PercentOutput, speed);

      }
      else {
        double errorsum = 0;
        if(Math.abs(lastWristPos - getWristMotorPosition()) < 500){
          errorsum += Math.abs(lastWristPos - getWristMotorPosition());
        }
        wrist.set(ControlMode.PercentOutput, ((lastWristPos - getWristMotorPosition()) * 0.00015) + errorsum*0.000 + wrist.getSelectedSensorVelocity() * 0.0000);
        SmartDashboard.putNumber("Wrist", ((lastWristPos - getWristMotorPosition()) * 0.00015));
        SmartDashboard.updateValues();
      }

      

    }


    public double getWristMotorPosition() {
      return wrist.getSelectedSensorPosition();
    }

    public void parallelWrist(double topEncoder, double bottomEncoder){
        double wristangle = 180 - (topStartAngle + (topEncoder/topArmEncodertickstoangle) + baseStartAngle - (bottomEncoder/bottomArmEncodertickstoangle));

        if(wrist.getSelectedSensorPosition() != ((wristStartAngle-wristangle)+wristAngleongnokizzy)*gearratio*2048){
          SmartDashboard.putNumber("Wrist Parallel", ((wristStartAngle-wristangle)+wristAngleongnokizzy)*gearratio*2048-wrist.getSelectedSensorPosition()*kp);
          SmartDashboard.updateValues();
            // wrist.set(ControlMode.PercentOutput, );
        } 

    }

    public void setWristPosition(WristPosition wPosition) {

      

      errorsum = 0;
      error = wPosition.wristSensorPosition - wrist.getSelectedSensorPosition();

      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      double errorrate = (error-lasterror)/dt;
      if(Math.abs(lastWristPos - getWristMotorPosition()) < 500){
          errorsum += dt * (lastWristPos - getWristMotorPosition());
      }
      double output = MathUtil.clamp(error*0.0008 + errorrate*0.0004 + errorsum*0.0001, -1, 1);

      wrist.set(ControlMode.PercentOutput, output);
 
      lastTimestamp = Timer.getFPGATimestamp();
      lasterror = error;
      lastWristPos = wrist.getSelectedSensorPosition();

      SmartDashboard.putNumber("wristPos", output);
      SmartDashboard.updateValues();
    }
}