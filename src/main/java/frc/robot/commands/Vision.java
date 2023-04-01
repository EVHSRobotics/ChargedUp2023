// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VideoServer;
import frc.robot.subsystems.Intake.GameObject;

public class Vision extends CommandBase {

  private VideoServer videoServer;
  // private AprilScanner aprilScanner;
  private Limelight aprilLimelight;
  private Limelight reflectiveLimelight;
  private Swerve swerve;
  private XboxController xboxController;
  private Limelight gameObjectLimelight;
  private Intake intake;

  double errorsum = 0;
  double lasterror = 0;
  double error;
  double lastTimestamp = 0;

  /** Creates a new Vision. */
  public Vision(Swerve swerve, Limelight aprilLimelight, Limelight reflectiveLimelight, Limelight gameObjectLimelight, Intake intake, VideoServer videoServer, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    // VideoServer videoServer, AprilScanner aprilScanner,
    this.videoServer = videoServer;
    // this.aprilScanner = aprilScanner;
    this.aprilLimelight = aprilLimelight;
    this.reflectiveLimelight = reflectiveLimelight;
    this.swerve = swerve;
    this.xboxController = xboxController;
    this.gameObjectLimelight = gameObjectLimelight;
this.intake = intake;
    // addRequirements(videoServer);
    // addRequirements(aprilScanner);
    addRequirements(aprilLimelight);
    addRequirements(reflectiveLimelight);
addRequirements(gameObjectLimelight);
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
    
      aimLimelightGameObjectPickup(gameObjectLimelight.getGameObject());



      // if (intake.gameObject == GameObject.CUBE) {

      //   aimLimelightAprilTags();
      // }
      // else if (intake.gameObject == GameObject.CONE) {
      //   aimLimelightReflective();
      // }
      
    }

    if (xboxController.getYButtonPressed()) {

    
    if (intake.gameObject == GameObject.CUBE) {
      aimLimelightAprilTags();
    }
    else {
      aimLimelightReflective();
    }
  }
   
  }

  public boolean aimLimelightGameObjectPickup(GameObject gameObject) {
      


    
    if (!gameObjectLimelight.getObjectSeen()) { return true; }

    double gameObjectDistance = gameObjectLimelight.getObjectDistanceCone();

    if (gameObject == GameObject.CUBE) {
      gameObjectDistance = gameObjectLimelight.getObjectDistanceCube();

    }
       SmartDashboard.putNumber("game Object Distance meters",gameObjectLimelight.getObjectDistanceCone());
       SmartDashboard.updateValues();
        
        
        SmartDashboard.putBoolean("Game Object", gameObject == GameObject.CUBE);
SmartDashboard.updateValues();
        // Max is 1.5 Meters to take affect
          double kpVert = MathUtil.applyDeadband((((gameObject == GameObject.CUBE) ? 15 : 22) -gameObjectDistance) * 0.03, 0.02);
          double kpHori = MathUtil.applyDeadband(gameObjectLimelight.getX() * 0.03, 0.02);
          SmartDashboard.putNumber("KP Vertical Object Detection", kpVert);
          SmartDashboard.updateValues();
            // 1.5 meters
            swerve.drive(
            new Translation2d(kpVert, kpHori).times(Constants.Swerve.maxSpeed).times(0.75), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
        if (kpVert <= 0.02 && kpHori <= 0.02) {
          return true;
        }
        else {
          return false;
        }

        
        
        
    }

    public GameObject getCurrentDetectedGameObject() {
return gameObjectLimelight.getGameObject();
    }
  public void aimLimelightReflective() {

    double x = reflectiveLimelight.getX();
    errorsum = 0;
    error = x;

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (error-lasterror)/dt;
    if(Math.abs(x) < 0.1){
        errorsum += dt *  x;
    }
    
    double output = MathUtil.clamp(error*0.045 + errorrate *0+errorsum*0.0, -1, 1);

    SmartDashboard.putNumber("limelight", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    lasterror = error;
    
    swerve.drive(new Translation2d(
        0, MathUtil.applyDeadband(output, 0.05)).times(Constants.Swerve.maxSpeed).times(0.5),
        0, true, false);
  }

  public void aimLimelightAprilTags() {

    double x = aprilLimelight.getX();
    errorsum = 0;
    error = x;

    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    double errorrate = (error-lasterror)/dt;
    if(Math.abs(x) < 0.1){
        errorsum += dt *  x;
    }
    double output = MathUtil.clamp(error*0.045 + errorrate *0+errorsum*0.0, -1, 1);

    SmartDashboard.putNumber("limelight", ( output));
    SmartDashboard.updateValues();
    lastTimestamp = Timer.getFPGATimestamp();
    lasterror = error;
    
    swerve.drive(new Translation2d(
        0, MathUtil.applyDeadband(output, 0.05)).times(Constants.Swerve.maxSpeed).times(0.5),
        0, true, false);
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
