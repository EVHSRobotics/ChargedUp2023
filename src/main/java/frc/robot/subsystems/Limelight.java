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

  private NetworkTableEntry tv;
  private int pipeline;

  /** Creates a new Limelight. */
  public Limelight(int pipeline) {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    
    this.pipeline = pipeline;
    table.getEntry("pipeline").setNumber(this.pipeline);

  }

  @Override
  public void periodic() {
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.updateValues();
  }
  
  // gets the error of the limelight to the detected object
  public double getX() {
    double x = tx.getDouble(0.0);
   
    if (tv.getBoolean(false)) {
      x = 0;
    }
    return x ;   
  }

  
}
