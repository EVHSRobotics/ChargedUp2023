// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VideoServer;

public class Vision extends CommandBase {

  private VideoServer videoServer;
  // private AprilScanner aprilScanner;
  private Limelight limelight;
  private Swerve swerve;
  private XboxController xboxController;

  double errorsum = 0;
  double lasterror = 0;
  double error;
  double lastTimestamp = 0;

  /** Creates a new Vision. */
  public Vision(Swerve swerve, Limelight limelight, VideoServer videoServer, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    // VideoServer videoServer, AprilScanner aprilScanner,
    this.videoServer = videoServer;
    // this.aprilScanner = aprilScanner;
    this.limelight = limelight;
    this.swerve = swerve;

    this.xboxController = xboxController;

    // addRequirements(videoServer);
    // addRequirements(aprilScanner);
    addRequirements(limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // aprilScanner.detectAprilCode();
    if (xboxController.getAButton()) {

      aimLimelight();
    }
    if (xboxController.getYButton()) {
      for (SwerveModule mod : swerve.mSwerveMods) {

        double absolutePosition = Conversions.degreesToFalcon(mod.getCanCoder().getDegrees() - 100,
            Constants.Swerve.angleGearRatio);
        mod.mAngleMotor.setSelectedSensorPosition(absolutePosition);
      }
    }

  }

  public void aimLimelight() {

    double x = limelight.getX();
    errorsum = 0;
    error = x;

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (error-lasterror)/dt;
    if(Math.abs(x) < 0.1){
        errorsum += dt *  x;
    }
    double output = MathUtil.clamp(error*0.00025 + errorrate *0+errorsum*0.0001, -1, 1);

    SmartDashboard.putNumber("limelight", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    lasterror = error;
    
    swerve.drive(new Translation2d(
        0, MathUtil.applyDeadband(output, 0.001)).times(Constants.Swerve.maxSpeed).times(0.5),
        0, false, false);
  }

  public void goright() {

    // swerve.drive(new Translation2d(0.1), 0, isFinished(), isFinished());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
