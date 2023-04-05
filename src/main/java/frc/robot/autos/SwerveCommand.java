// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.sql.Time;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.FourBar;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Vision;
// import frc.robot.subsystems.Arm.BottomArmPosition;
import frc.robot.subsystems.Arm.TopArmPosition;
import frc.robot.subsystems.Intake.GameObject;
import frc.robot.subsystems.Wrist.WristPosition;

public class SwerveCommand extends CommandBase {

  private PathCommandAction commandAction;
  private FourBar fourBar;
  private Vision vision;
  private TeleopSwerve teleopSwerve;
  private double time = -1;
  
  public static enum PathCommandAction {
      
    INTAKE, OUTTAKEHIGH, OUTTAKEMID, AUTOALIGNRAMP, ALIGNCUBE;

}

  /** Creates a new SwerveCommand. */
  public SwerveCommand(PathCommandAction commandAction, FourBar fourBar, Vision vision, TeleopSwerve teleopSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.commandAction = commandAction;
    this.fourBar = fourBar;
    this.teleopSwerve = teleopSwerve;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (commandAction) {
    
      case ALIGNCUBE:
      time = -1;
      fourBar.setIntakeGameObject(GameObject.CONE);

      break;
      case AUTOALIGNRAMP:
        break;
      case INTAKE:
      
      fourBar.setIntakeGameObject(GameObject.CONE);
      fourBar.outtakePower = 1.0;
      fourBar.intake.cIntakeType = frc.robot.subsystems.Intake.IntakeType.LOW;
      fourBar.deployIntake = true;

        break;
      case OUTTAKEMID:
      fourBar.setIntakeGameObject(GameObject.CUBE);

      // Midddle
      fourBar.tPositionScoring = TopArmPosition.MIDDLE;
      // fourBar.bPositionScoring = BottomArmPosition.IN;
      fourBar.wPositionScoring = WristPosition.MIDDLE;
      fourBar.shootArmTime = -1;
      fourBar.deployShoot = true;
        
        break;
      case OUTTAKEHIGH:
      fourBar.setIntakeGameObject(GameObject.CUBE);
      // High
      fourBar.tPositionScoring = TopArmPosition.STRAIGHT;
      // fourBar.bPositionScoring = BottomArmPosition.IN;
      fourBar.wPositionScoring = WristPosition.SHOOTINGCUBE;
      fourBar.shootArmTime = -1;
      fourBar.deployShoot = true;
        break;
      default:

        break;
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (commandAction) {
     
      case AUTOALIGNRAMP:

        teleopSwerve.autoAlignRamp();
        break;
      case INTAKE:
      
      SmartDashboard.putNumber("time", System.currentTimeMillis() - time);
      SmartDashboard.updateValues();
        fourBar.intakeAuto();
        break;
      case OUTTAKEHIGH:
      fourBar.shootAuto(fourBar.deployShoot);
        break;
      case OUTTAKEMID:
      
       fourBar.shootAuto(fourBar.deployShoot);

        break;
        case ALIGNCUBE:
        if (time == -1) {
          time = System.currentTimeMillis();
        }
        
        break;
      default:

        break;
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (commandAction) {
case INTAKE:
      while (Math.abs(fourBar.wrist.setWristPosition(WristPosition.UP)) >= 0.1) { }
      fourBar.wrist.moveWrist(0);
      break;
     case ALIGNCUBE:
      teleopSwerve.s_Swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        false, 
        true
    );
    }
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (commandAction) {
   
      case AUTOALIGNRAMP:
      return false;
      case INTAKE:
      // fourBar.activateCube();
      // if ((System.currentTimeMillis() - time) >= 2 && time != -1) {
      //   fourBar.deployIntake = false;
      //  if (fourBar.wrist.setWristPosition(WristPosition.UP) <= 0.1) {
        // return true;
      //  }
      //  else { 
      //   return false;
      // }
return !fourBar.deployIntake;
      // }
      // else { 
      //   return false;
      // }
      
      case OUTTAKEHIGH:
      return !fourBar.deployShoot;

      
      case OUTTAKEMID:
      
      return !fourBar.deployShoot;
      case ALIGNCUBE:
      return vision.aimLimelightGameObjectAutoPickup(vision.gameObjectBottomLimelight, GameObject.CONE, false);

      default:
      return false;
      
    }    
  }
}
