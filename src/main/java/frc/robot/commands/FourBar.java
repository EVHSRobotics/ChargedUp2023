// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.BottomArmPosition;
import frc.robot.subsystems.Arm.SparkLEDColors;
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
  public TopArmPosition tPositionScoring = TopArmPosition.DOWN;
  public BottomArmPosition bPositionScoring = BottomArmPosition.IN;
  public WristPosition wPositionScoring = WristPosition.UP;

  private long currentIntakeTime = -1;
  private boolean cubeBooleanFlag = false;
  private boolean deployIntake = false;
  private boolean deployShoot = false;
  public GameObject gameObject = GameObject.CUBE;
  double outtakePower = 1;
  private double shootArmTime = -1;

  private GenericEntry boardIntakeIn;
  private GenericEntry boardLowIntake;
  private GenericEntry boardShoot;
  private boolean lowIntake = true;

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

    boardIntakeIn = Shuffleboard.getTab("DriverStation_2023").add("Intake In", true).getEntry();
    boardLowIntake = Shuffleboard.getTab("DriverStation_2023").add("Low Intake", true).getEntry();
    boardShoot = Shuffleboard.getTab("DriverStation_2023").add("Shoot", true).getEntry();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetArmEncoders();
    wrist.resetWristEncoder();


    deployIntake = false;
deployShoot = false;
shootArmTime = -1;

  }

 
  public void shoot(boolean action) {

    if (action) {
      if (shootArmTime == -1) {
        shootArmTime = System.currentTimeMillis();
      }
      arm.setTopPosition(tPositionScoring);
      arm.setBottomPosition(bPositionScoring);

      if (System.currentTimeMillis() - shootArmTime >= 5500) {
        deployShoot = false;

      }
      else if (System.currentTimeMillis() - shootArmTime >= 5000) {
        wrist.setWristPosition(WristPosition.UP);
      }
      else if (System.currentTimeMillis() - shootArmTime >= 3000) {
          intake.runIntake(gameObject == GameObject.CUBE ? outtakePower : -outtakePower);
          arm.setLED(SparkLEDColors.SHOOT);
      }
      else if (System.currentTimeMillis() - shootArmTime >= 700) {
        wrist.setWristPosition(wPositionScoring);

      }
    
    }
  
   
  }
  public void intakeAction(boolean action) {
    if (!lowIntake) {
      if (action) {
        arm.setTopPosition(TopArmPosition.STRAIGHT);
        if (xboxController.getRawButtonPressed(XboxController.Button.kStart.value)) {
          wrist.moveWrist(MathUtil.applyDeadband(xboxController.getLeftY(), 0.05));
        }
        else {
          wrist.setWristPosition(WristPosition.HIGHINTAKE);

        }

          intake.runIntake(gameObject == GameObject.CUBE ? -outtakePower : outtakePower);
          if (gameObject == GameObject.CUBE) {
            arm.setLED(SparkLEDColors.PURPLE);
          }
          else {
            // Cone
            arm.setLED(SparkLEDColors.YELLOW);
          }
        
        
        
  deployIntake = true;
      }
     
    }
    else {

    if (action) {
      if (gameObject == GameObject.CONE) {
        wrist.setWristPosition(WristPosition.STRAIGHTCONE);
        arm.setLED(SparkLEDColors.PURPLE);

      }
      else  {
        // arm.setBottomPosition(BottomArmPosition.MIDDLE);
        wrist.setWristPosition(WristPosition.STRAIGHTCUBE);
        arm.setLED(SparkLEDColors.YELLOW);

      }
    
      SmartDashboard.putBoolean("itnae banne", intake.getIntakeBanner());
      SmartDashboard.updateValues();
        intake.runIntake(gameObject == GameObject.CUBE ? -outtakePower : outtakePower);
       
        deployIntake = true;

    }
    // else {
    // wrist.setWristPosition(WristPosition.UP);
    // deployIntake = false;
    // }
  }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    SmartDashboard.putNumber("volts bus", arm.topArm.getBusVoltage());
    SmartDashboard.putNumber("motor out ", arm.topArm.getMotorOutputVoltage());
    SmartDashboard.putNumber("wrist encoder", wrist.getWristMotorPosition());
    SmartDashboard.updateValues();
  
  SmartDashboard.putNumber("Wrist", wrist.getWristMotorPosition());
  SmartDashboard.putNumber("Top Arm", arm.getTopArmPosition());
  SmartDashboard.putNumber("Bottom Arm", arm.getBottomArmPosition());
  SmartDashboard.putBoolean("deployIntake", deployIntake);
  SmartDashboard.putBoolean("deployShoot", deployShoot);
  SmartDashboard.updateValues();

 
    if (xboxController.getYButtonPressed()) {
      gameObject = GameObject.CONE;
      outtakePower = 1.0;
      lowIntake = true;
      deployIntake = !deployIntake;
    }

 if (xboxController.getXButtonPressed()) {
  gameObject = GameObject.CUBE;
  outtakePower = 1.0;
  lowIntake = true;
deployIntake = !deployIntake;
 }

 if (xboxController.getLeftBumperPressed()) {
  gameObject = GameObject.CUBE;
  outtakePower = 1.0;
  lowIntake = false;
deployIntake = !deployIntake;
 }
 if (xboxController.getRightBumperPressed()) {
  gameObject = GameObject.CONE;
  outtakePower = 1.0;
  lowIntake = false;
deployIntake = !deployIntake;
 }

  

 boardLowIntake.setBoolean(lowIntake);
 boardIntakeIn.setBoolean(deployIntake);
 boardShoot.setBoolean(deployShoot);
 

    SmartDashboard.putNumber("Selected Sensor Intake Velocity", intake.getIntakeCurrent());
    SmartDashboard.putNumber("Intake Motor Velocity", intake.getIntakeVelocity());
    SmartDashboard.putNumber("POV", xboxController.getPOV());
    SmartDashboard.updateValues();

    
    if (xboxController.getPOV() == 0) {
      shootArmTime = -1;
      outtakePower = 0.55;

      tPositionScoring = TopArmPosition.STRAIGHT;
      bPositionScoring = BottomArmPosition.IN;
      wPositionScoring = WristPosition.SHOOTING;
      deployShoot = !deployShoot;
   

    }
    else if (xboxController.getPOV() == 90) {
      shootArmTime = -1;

      outtakePower = 0.55;
      tPositionScoring = TopArmPosition.MIDDLE;
      bPositionScoring = BottomArmPosition.IN;
      wPositionScoring = WristPosition.MIDDLE;
      deployShoot = !deployShoot;
    }
    else if (xboxController.getPOV() == 180) {
      shootArmTime = -1;

      outtakePower = 0.4;
      tPositionScoring = TopArmPosition.DOWN;
      bPositionScoring = BottomArmPosition.IN;
      wPositionScoring = WristPosition.UP;
      deployShoot = !deployShoot;
    }
    else if (xboxController.getPOV() == 270) {

      deployShoot = false;
    }

    shoot(deployShoot);
    intakeAction(deployIntake);

    if (!deployIntake && !deployShoot) {
      arm.setBottomPosition(BottomArmPosition.IN);
      arm.setTopPosition(TopArmPosition.DOWN);
      wrist.setWristPosition(WristPosition.UP);
      arm.setLED(SparkLEDColors.RAINBOW);
          
      
      // Manual Intake Control
    if (xboxController.getLeftTriggerAxis() > 0) {
    
      intake.runIntake(0.7);

  }
  else if (xboxController.getRightTriggerAxis() > 0) {

      intake.runIntake(-0.7);

  }
  else {
    intake.runIntake(0);
  }
    }
  
  SmartDashboard.putNumber("Top Arm", arm.getTopArmPosition());
  SmartDashboard.putNumber("Bottom Arm", arm.getBottomArmPosition());
    SmartDashboard.updateValues();

  }

  // Auto Functions 
  public boolean shootAuto(boolean action) {

    if (action) {
      if (shootArmTime == -1) {
        shootArmTime = System.currentTimeMillis();
      }
      arm.setTopPosition(tPositionScoring);
      arm.setBottomPosition(bPositionScoring);
      
      if (gameObject == GameObject.CUBE) {
        arm.setLED(SparkLEDColors.PURPLE);
      }
      else {
        // Cone
        arm.setLED(SparkLEDColors.YELLOW);
      }
      if (System.currentTimeMillis() - shootArmTime >= 6700) {
        return true;
      }
      else if (System.currentTimeMillis() - shootArmTime >= 5000) {
        tPositionScoring = TopArmPosition.DOWN;
        bPositionScoring = BottomArmPosition.IN;
        wrist.setWristPosition(WristPosition.UP);
      
        return false;
      }
      else if (System.currentTimeMillis() - shootArmTime >= 3000) {
          intake.runIntake(gameObject == GameObject.CUBE ? outtakePower : -outtakePower);
        
        return false;
      }
      else if (System.currentTimeMillis() - shootArmTime >= 700) {
        wrist.setWristPosition(WristPosition.SHOOTING);
        return false;

      }
      else {
        return false;
      }
    }
    else { return false; }
    
    }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
