// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private TalonFX bottomArm;
  public TalonFX topArm;

  private Spark sparkLights;

private double bErrorSum = 0;
private double tErrorSum = 0;
private double bError = 0;
private double tError = 0;
private double bLastError = 0;
private double tLastError = 0;
private double lastTimestamp = 0;
private double tLastPos = 0;
private double bLastPos = 0;

public enum SparkLEDColors {

  RAINBOW(-0.99), SHOOT(-0.15), PURPLE(0.89), YELLOW(0.69);

  private double ledColorValue;

  public double getColor() {
    return ledColorValue;
  }

  private SparkLEDColors(double ledColor) {
    this.ledColorValue = ledColor;
  }

}

  // enum to get various positions of the top motor
  public enum TopArmPosition {

    DOWN(0), MIDDLE(190000), GROUNDMIDDLE(210000), STRAIGHT(395000);
 
     private double tArmSensorPosition;
     public double getPos() {
      return tArmSensorPosition;
     }
     private TopArmPosition(double tArmSensorPosition) {
       this.tArmSensorPosition = tArmSensorPosition;
     }
   }

    // enum to get various positions of the bottom motor

   public enum BottomArmPosition {

    IN(0), MIDDLE(-227500), OUT(-455000);
 
     private double bArmSensorPosition;
     public double getPos() {
      return bArmSensorPosition;
     }
     private BottomArmPosition(double bArmSensorPosition) {
       this.bArmSensorPosition = bArmSensorPosition;
     }
   }
  /** Creates a new Arm. */
  public Arm() {

    bottomArm = new TalonFX(Constants.Arm.bottomArmMotor);
    topArm = new TalonFX(Constants.Arm.topArmMotor);

    topArm.setNeutralMode(NeutralMode.Brake);
    bottomArm.setNeutralMode(NeutralMode.Brake);

    sparkLights = new Spark(Constants.Arm.colorsDIO);

    sparkLights.set(SparkLEDColors.RAINBOW.getColor());

    // bottomArm.configForwardSoftLimitThreshold(Constants.Arm.bottomForwardLimit);
    // bottomArm.configReverseSoftLimitThreshold(Constants.Arm.bottomReverseLimit);
    
    // topArm.configForwardSoftLimitThreshold(Constants.Arm.topForwardLimit);
    // topArm.configReverseSoftLimitThreshold(Constants.Arm.topReverseLimit);
    
  }


  public void resetArmEncoders() {
    topArm.setSelectedSensorPosition(0);
    bottomArm.setSelectedSensorPosition(0);
    
  }

  public void setLED(SparkLEDColors ledColor) {
  
    sparkLights.set(ledColor.getColor());
  }

  public double getTopArmPosition() {
    return topArm.getSelectedSensorPosition();
  }
  public double getBottomArmPosition() {
    return bottomArm.getSelectedSensorPosition();
  }
  public void rotateBottom(double pow) {
   
    bottomArm.set(ControlMode.PercentOutput, pow);
   
  }

  public void rotateTop(double pow) {
    // if((pow > 0 && topArm.getSelectedSensorPosition() <= Constants.Arm.topForwardLimit) || (pow < 0 && topArm.getSelectedSensorPosition() >= Constants.Arm.topReverseLimit)){
      
      topArm.set(ControlMode.PercentOutput, pow);

  // }
  // else {
  //   topArm.set(ControlMode.PercentOutput, 0);
  // }
  }

  public double setTopPosition(TopArmPosition tPosition){
    SmartDashboard.putNumber("top pos PID", MathUtil.applyDeadband((tPosition.getPos() - getTopArmPosition()) * 0.000003, 0.01));
    SmartDashboard.updateValues();
    
    
    tError = tPosition.getPos() - getTopArmPosition();

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (tError-tLastError)/dt;
    if(Math.abs(tLastPos - getTopArmPosition()) < 10000){
        tErrorSum += dt * (tLastPos - getTopArmPosition());
    }
    double output = MathUtil.clamp(tError*0.000004 + errorrate *0.0+tErrorSum*0.00001, -1, 1);

    SmartDashboard.putNumber("Top PID Output", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    tLastError = tError;
    tLastPos = getTopArmPosition();


    
    topArm.set(ControlMode.PercentOutput, MathUtil.applyDeadband(output, 0.05));


    // We can know how much the output is off by in caller
    return MathUtil.applyDeadband(output, 0.05);
    // topArm.set(ControlMode.PercentOutput, MathUtil.applyDeadband((tPosition.getPos() - getTopArmPosition()) * 0.000003, 0.01));
 
  }
  public void setBottomPosition(BottomArmPosition bPosition){
    SmartDashboard.putNumber("bottom pos PID", MathUtil.applyDeadband((bPosition.getPos() - getBottomArmPosition()) * 0.00000095, 0.01));
    SmartDashboard.updateValues();


    bError = bPosition.getPos()-getBottomArmPosition();

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (bError-bLastError)/dt;
    if(Math.abs(bLastPos - getBottomArmPosition()) < 500){
        bErrorSum += dt * (bLastPos - getBottomArmPosition());
    }
    double output = MathUtil.clamp(bError*0.0008 + errorrate *0+bErrorSum*0, -1, 1);

    SmartDashboard.putNumber("Bottom PID Output", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    bLastError = bError;
    bLastPos = getBottomArmPosition();


    bottomArm.set(ControlMode.PercentOutput, output);
    // bottomArm.set(ControlMode.PercentOutput, MathUtil.applyDeadband((bPosition.getPos() - getBottomArmPosition()) * 0.00000095, 0.01));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
