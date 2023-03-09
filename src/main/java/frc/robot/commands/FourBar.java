// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.BottomArmPosition;
import frc.robot.subsystems.Arm.TopArmPosition;
import frc.robot.subsystems.Intake.GameObject;
import frc.robot.subsystems.Wrist.WristPosition;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class FourBar extends CommandBase {

  private Wrist wrist;
  private Intake intake;
  public Arm arm;
  private XboxController xboxController;
  private XboxController driveController;
  private TopArmPosition tPositionScoring = TopArmPosition.DOWN;
  private BottomArmPosition bPositionScoring = BottomArmPosition.IN;
  private WristPosition wPositionScoring = WristPosition.UP;

  private long currentIntakeTime = -1;
  private boolean cubeBooleanFlag = false;
  private boolean deployIntake = false;
  private boolean deployShoot = false;
  private GameObject gameObject = GameObject.UNKNOWN;

  private double shootArmTime = -1;

  /** Creates a new Claw. */
  public FourBar(Arm arm, Wrist wrist, Intake intake, XboxController controller, XboxController driveController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.arm = arm;
    this.intake = intake;
    this.xboxController = controller;
    this.driveController = driveController;

    addRequirements(wrist);
    addRequirements(arm);
    addRequirements(intake);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetArmEncoders();
    wrist.wrist.setSelectedSensorPosition(0);    


    deployIntake = false;
deployShoot = false;
    // intake.actionIntake(true);

  }

 
  public void shoot(boolean action) {

    if (action) {
      if (shootArmTime == -1) {
        shootArmTime = System.currentTimeMillis();
      }
      arm.setTopPosition(tPositionScoring);
arm.setBottomPosition(bPositionScoring);
      if (System.currentTimeMillis() - shootArmTime >= 700) {
        wrist.setWristPosition(wPositionScoring);

      }
      else if (System.currentTimeMillis() - shootArmTime >= 2000) {
        if (gameObject != GameObject.UNKNOWN) {
          intake.runIntake(gameObject == GameObject.CUBE ? -0.6 : 0.6);
        }
      }
      else if (System.currentTimeMillis() - shootArmTime >= 4000) {
        deployShoot = false;
      }
      else {
        deployShoot = true;

      }
      // if (!xboxController.getAButton()) {
      //   if (gameObject != GameObject.UNKNOWN) {
 
      //   intake.runIntake(gameObject == GameObject.CUBE ? -0.6 : 0.6);
      //   }
      // }


    }
    else {
      arm.setTopPosition(TopArmPosition.DOWN);
      arm.setBottomPosition(BottomArmPosition.IN);
      wrist.setWristPosition(WristPosition.UP);
      shootArmTime = -1;

   

      deployShoot = false;
    }
   
  }
  public void intakeAction(boolean action, GameObject gameObject) {
    if (action) {
    // arm.setTopPosition(TopArmPosition.STRAIGHT);
    wrist.setWristPosition(WristPosition.STRAIGHT);
      SmartDashboard.putBoolean("itnae banne", intake.getIntakeBanner());
      SmartDashboard.updateValues();
      // if (intake.getIntakeBanner()) {

        // intake.runIntake(gameObject == GameObject.CUBE ? -0.7 : 0.7);
        deployIntake = true;

  //     }
  //     else {
  // deployIntake = false;
  //     }
    }
    else {
    // arm.setTopPosition(TopArmPosition.DOWN);
    wrist.setWristPosition(WristPosition.UP);
    deployIntake = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake.actionIntake(!xboxController.getAButton());
    // intake.runIntake(MathUtil.applyDeadband(xboxController.getRightY(), Constants.stickDeadband));


    // intakeAction(xboxController.getAButton());

    // if (xboxController.getAButtonPressed()) {
    //   intake.runIntake(gameObject == GameObject.CUBE ? -xboxController.getRightTriggerAxis() : xboxController.getRightTriggerAxis());

    // }

    arm.setArmSolenoids(true);

    if (xboxController.getLeftTriggerAxis() > 0) {
      // if (intake.getIntakeBanner()) {
      //   intake.runIntake(0);

      // }
      // else {
        intake.runIntake(0.7);

      // }
    }
    else if (xboxController.getRightTriggerAxis() > 0) {
      
      // if (intake.getIntakeBanner()) {
      //   intake.runIntake(0);

      // }
      // else {
        intake.runIntake(-0.7);

      // }
    }
    else {
      intake.runIntake(0);
    }

    
    // if (driveController.getLeftTriggerAxis() > 0) {
    //   arm.rotateBottom(MathUtil.applyDeadband(driveController.getLeftTriggerAxis() * 0.5, 0.1));

    // }
    // else {
    //   arm.rotateBottom(MathUtil.applyDeadband(-driveController.getRightTriggerAxis() * 0.5, 0.1));

    // }
    // if (xboxController.getAButton()) 
      // arm.setTopPosition(10000);
    

    SmartDashboard.putNumber("volts bus", arm.topArm.getBusVoltage());
    SmartDashboard.putNumber("motor out ", arm.topArm.getMotorOutputVoltage());
    SmartDashboard.putNumber("wrist encoder", wrist.getWristMotorPosition());
    SmartDashboard.updateValues();
  // if (xboxController.getBButton()) {
  //   runIntakeObject();
  // }

  
  
  SmartDashboard.putBoolean("deployIntake", deployIntake);
  SmartDashboard.putBoolean("deployShoot", deployShoot);
  SmartDashboard.updateValues();

  if (!xboxController.getYButton() && !xboxController.getXButton() && !xboxController.getBButton()) {
    wrist.setWristPosition(WristPosition.UP);
  }
    if (xboxController.getYButtonPressed()) {
      gameObject = GameObject.CONE;

      deployIntake = !deployIntake;
    }

 if (xboxController.getXButtonPressed()) {
  gameObject = GameObject.CUBE;
deployIntake = !deployIntake;
 }

    // if (xboxController.getBButtonPressed()) {
    //   deployShoot = !deployShoot;
    // }
  

    
    if (xboxController.getPOV() == 0) {
      shootArmTime = -1;

      tPositionScoring = TopArmPosition.STRAIGHT;
      bPositionScoring = BottomArmPosition.MIDDLE;
      wPositionScoring = WristPosition.SHOOTING;
      deployShoot = !deployShoot;

    }
    else if (xboxController.getPOV() == 90) {
      shootArmTime = -1;

      tPositionScoring = TopArmPosition.STRAIGHT;
      bPositionScoring = BottomArmPosition.IN;
      wPositionScoring = WristPosition.SHOOTING;
      deployShoot = !deployShoot;

    }
    else if (xboxController.getPOV() == 180) {
      shootArmTime = -1;

      tPositionScoring = TopArmPosition.DOWN;
      bPositionScoring = BottomArmPosition.IN;
      wPositionScoring = WristPosition.UP;
      deployShoot = !deployShoot;

    }

    if (!deployIntake) {
      shoot(deployShoot);
    }
    else {
      intakeAction(deployIntake, gameObject);

    }

    

  

  if(xboxController.getLeftBumper()){
    arm.resetArmEncoders();
  }
  

  // SmartDashboard.putNumber("Proximit", intake.intakeColorSensor.getProximity());
  // SmartDashboard.putBoolean("Cone", intake.getColorSensorGameObject() == GameObject.CONE);
  // SmartDashboard.putBoolean("Cube", intake.getColorSensorGameObject() == GameObject.CUBE);
  // SmartDashboard.putBoolean("UNKNOWN", intake.getColorSensorGameObject() == GameObject.UNKNOWN);
  // SmartDashboard.putNumber("Red", intake.intakeColorSensor.getColor().red);
  // SmartDashboard.putNumber("Green", intake.intakeColorSensor.getColor().green);
  // SmartDashboard.putNumber("BLue", intake.intakeColorSensor.getColor().blue);


  // arm.setTopPosition(100000);
  SmartDashboard.putNumber("Top Arm", arm.getTopArmPosition());
  SmartDashboard.putNumber("Bottom Arm", arm.getBottomArmPosition());
    SmartDashboard.updateValues();
    // arm.rotateBottom(MathUtil.applyDeadband(xboxController.getRightX(), Constants.stickDeadband));
    
    // if (!xboxController.getAButton()) arm.rotateTop(MathUtil.applyDeadband(xboxController.getLeftY() * -1, Constants.stickDeadband));
  
    // wrist.moveWrist(xboxController.getLeftX() * 0.2);
  
  }

  // public void runIntakeObject() {
  //   switch (intake.getColorSensorGameObject()) {
  //     case CONE:
  //       activateCone();
  //       break;
  //     case CUBE:
  //       activateCube();
  //       break;
  //     case UNKNOWN:
  //       break;
  //     default:
  //       break;
  //   }
  // }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
