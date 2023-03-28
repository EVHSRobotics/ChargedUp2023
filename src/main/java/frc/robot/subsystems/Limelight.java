// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry tv;
  private int pipeline;

  private double mountAngle; // Degrees
  private double lensHeight = 20; // Inches
  // Target Height

  /** Creates a new Limelight. */
  public Limelight(int pipeline, String tableName, double mountangle) {
    this.mountAngle = mountangle;
    table = NetworkTableInstance.getDefault().getTable(tableName);
    this.pipeline = pipeline;
    table.getEntry("pipeline").setNumber(this.pipeline);

  }

  @Override
  public void periodic() {

    tx = table.getEntry("tx");
    ty = table.getEntry("ty"); 
    tv = table.getEntry("tv");
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.updateValues();

  }
  
  // Uses tangent to get distance based on angle of tilt from vertical
  public double calculateDistanceObject() {
    double totalAngleDeg = mountAngle + getY();
    // convert to radians
    double totalAngleRads = totalAngleDeg * (Math.PI/180);

    return -lensHeight/Math.tan(totalAngleRads);
  }

  // gets the error of the limelight to the detected object
  public double getX() {
    double x = tx.getDouble(0.0);
   
    if (tv.getBoolean(false)) {
      x = 0;
    }
    return x;   
  }

  // gets the error of the limelight to the detected object
  public double getY() {
    double y = ty.getDouble(0.0);
   
    if (tv.getBoolean(false)) {
      y = 0;
    }
    return y;   
  }
}
