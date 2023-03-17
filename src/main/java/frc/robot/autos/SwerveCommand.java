// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;
import frc.robot.commands.FourBar;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Vision;
import frc.robot.subsystems.Arm.BottomArmPosition;
import frc.robot.subsystems.Arm.TopArmPosition;
import frc.robot.subsystems.Intake.GameObject;
import frc.robot.subsystems.Wrist.WristPosition;

public class SwerveCommand extends CommandBase {

  private PathCommandAction commandAction;
  private FourBar fourBar;
  private Vision vision;
  private TeleopSwerve teleopSwerve;
  private boolean isFinished = false;
  
  public static enum PathCommandAction {
      
    INTAKECONE, INTAKECUBE, OUTTAKECONE, OUTTAKECUBE, AUTOALIGNRAMP;

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
      case INTAKECONE:
        // fourBar.activateCone();
        break;
      case AUTOALIGNRAMP:
        break;
      case INTAKECUBE:
      // fourBar.activateCube();
        break;
      case OUTTAKECONE:
      // fourBar.activateCube();

        break;
      case OUTTAKECUBE:
      fourBar.gameObject = GameObject.CUBE;
      fourBar.tPositionScoring = TopArmPosition.STRAIGHT;
      fourBar.bPositionScoring = BottomArmPosition.IN;
      fourBar.wPositionScoring = WristPosition.SHOOTING;
      fourBar.shootArmTime = -1;
        break;
      default:

        break;
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (commandAction) {
      case INTAKECONE:
        // fourBar.activateCone();
        break;
      case AUTOALIGNRAMP:
        teleopSwerve.autoAlignRamp();
        break;
      case INTAKECUBE:
      // fourBar.activateCube();
        break;
      case OUTTAKECONE:
      // fourBar.activateCube();

        break;
      case OUTTAKECUBE:
      
      this.isFinished = fourBar.shootAuto(true);

      SmartDashboard.putBoolean("d", isFinished);
        break;
      default:

        break;
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
