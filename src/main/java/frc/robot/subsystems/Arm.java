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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private TalonFX bottomArm;
  public TalonFX topArm;



public TopArmPosition tArmPosition = TopArmPosition.DOWN;
public BottomArmPosition bArmPosition = BottomArmPosition.IN;

double bErrorSum = 0;
double tErrorSum = 0;
double bError = 0;
double tError = 0;
double bLastError = 0;
double tLastError = 0;
double lastTimestamp = 0;
double tLastPos = 0;
double bLastPos = 0;
  public enum TopArmPosition {

    DOWN(0), MIDDLE(190000), STRAIGHT(380000);
 
     private double tArmSensorPosition;
     public double getPos() {
      return tArmSensorPosition;
     }
     private TopArmPosition(double tArmSensorPosition) {
       this.tArmSensorPosition = tArmSensorPosition;
     }
   }
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

    // bottomArm.configForwardSoftLimitThreshold(Constants.Arm.bottomForwardLimit);
    // bottomArm.configReverseSoftLimitThreshold(Constants.Arm.bottomReverseLimit);
    
    // topArm.configForwardSoftLimitThreshold(Constants.Arm.topForwardLimit);
    // topArm.configReverseSoftLimitThreshold(Constants.Arm.topReverseLimit);
    
  }


  public void resetArmEncoders() {
    topArm.setSelectedSensorPosition(0);
    bottomArm.setSelectedSensorPosition(0);
    
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

  public void setTopPosition(TopArmPosition tPosition){
tArmPosition = tPosition;
    SmartDashboard.putNumber("top pos PID", MathUtil.applyDeadband((tPosition.getPos() - getTopArmPosition()) * 0.000003, 0.01));
    SmartDashboard.updateValues();
    
    
    tError = tPosition.getPos() - getTopArmPosition();

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (tError-tLastError)/dt;
    if(Math.abs(tLastPos - getTopArmPosition()) < 10000){
        tErrorSum += dt * (tLastPos - getTopArmPosition());
    }
    double output = MathUtil.clamp(tError*0.000004 + errorrate *0.000001+tErrorSum*0.0, -1, 1);

    SmartDashboard.putNumber("Top PID Output", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    tLastError = tError;
    tLastPos = getTopArmPosition();


    topArm.set(ControlMode.PercentOutput, MathUtil.applyDeadband(output, 0.01));

    // topArm.set(ControlMode.PercentOutput, MathUtil.applyDeadband((tPosition.getPos() - getTopArmPosition()) * 0.000003, 0.01));
 
  }
  public void setBottomPosition(BottomArmPosition bPosition){
bArmPosition = bPosition;
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
