// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
// import frc.robot.subsystems.Arm.BottomArmPosition;
import frc.robot.subsystems.Arm.SparkLEDColors;
import frc.robot.subsystems.Arm.TopArmPosition;
import frc.robot.subsystems.Intake.GameObject;
import frc.robot.subsystems.Wrist.WristPosition;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class FourBar extends CommandBase {

  public Wrist wrist;
  private Intake intake;
  public Arm arm;
  private XboxController xboxController;
  private XboxController driveController;
  public TopArmPosition tPositionScoring = TopArmPosition.DOWN;
  // public BottomArmPosition bPositionScoring = BottomArmPosition.IN;
  public WristPosition wPositionScoring = WristPosition.UP;

  private long currentIntakeTime = -1;
  private boolean cubeBooleanFlag = false;
  public boolean deployIntake = false;
  public boolean deployShoot = false;
  public double outtakePower = 1;
  public double shootArmTime = -1;

  private GenericEntry boardIntakeIn;
  private GenericEntry boardLowIntake;
  private GenericEntry boardShoot;
  private Vision vision;
  public IntakeType cIntakeType = IntakeType.LOW;

  // Different intake types
  // High - Human Player Station
  // Low - When you lower the intake right on the game element
  // Ground - You can intake the cone when it is flipped
  public enum IntakeType {
    LOW, HIGH, GROUND;
  }

  /** Creates a new Claw. */
  public FourBar(Arm arm, Wrist wrist, Intake intake, Vision vision, XboxController controller, XboxController driveController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.arm = arm;
    this.intake = intake;
    this.xboxController = controller;
    this.driveController = driveController;
this.vision = vision;
    addRequirements(wrist);
    addRequirements(arm);
    addRequirements(intake);

    boardIntakeIn = Shuffleboard.getTab("DriverStation_2023").add("Intake In", true).getEntry();
    boardLowIntake = Shuffleboard.getTab("DriverStation_2023").add("Low Intake", true).getEntry();
    boardShoot = Shuffleboard.getTab("DriverStation_2023").add("Shoot", true).getEntry();

  }

  public void setIntakeGameObject(GameObject gameObject) {
    intake.gameObject = gameObject;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Resets Encoders at the start
    // arm.resetArmEncoders();
    // wrist.resetWristEncoder();
    wPositionScoring = WristPosition.UP;
    tPositionScoring = TopArmPosition.DOWN;
    // bPositionScoring = BottomArmPosition.IN;
    // Resets deploy intake and shoot, shootime
    deployIntake = false;
    deployShoot = false;
    cIntakeType = IntakeType.LOW;
    shootArmTime = -1;

  }

  // This is the shoot method which can be fed a bool at all times
  public void shoot(boolean action) {

    // If the action is true, and the first time this is getting called in a
    // routine, we reset shoot arm time
    // Routine
    // shoot arm time is reseted it is set to current system
    // set the top position of the arm to the current scoring top
    // set the bottom position of the arm to the current scoring bottom
    // time based
    // 5500 ms - resets the arm & wrist completely and stops calling the shoot
    // function
    // 5000 ms - resets the wrist positon back to stowed away state first when
    // resetting
    // 3000 ms - sets intake to run for 2000 ms
    // 700 ms - sets wrist position to the variable pos

    if (action) {
      if (shootArmTime == -1) {
        shootArmTime = System.currentTimeMillis();
      }
      arm.setTopPosition(tPositionScoring);
      // arm.setBottomPosition(bPositionScoring);
      if (intake.gameObject == GameObject.CONE) {
        arm.setLED(SparkLEDColors.SHOOTYELLOW);

      }
      else if (intake.gameObject == GameObject.CUBE) {
        arm.setLED(SparkLEDColors.SHOOTBLUE);

      }
      else {
        arm.setLED(SparkLEDColors.SHOOT);

      }
      if (wPositionScoring == WristPosition.UP && tPositionScoring == TopArmPosition.DOWN) {
        if (System.currentTimeMillis() - shootArmTime >= 700) {
          intake.gameObject = GameObject.UNKNOWN;

          deployShoot = false;

        }
        else {
          intake.runIntake(intake.gameObject == GameObject.CUBE ? outtakePower : -outtakePower);

        } 
      }
      else {
      if (System.currentTimeMillis() - shootArmTime >= 5500) {
        intake.gameObject = GameObject.UNKNOWN;

        deployShoot = false;


      } else if (System.currentTimeMillis() - shootArmTime >= 5000) {
        wrist.setWristPosition(WristPosition.UP);
      } else if (System.currentTimeMillis() - shootArmTime >= 3000) {
        intake.runIntake(intake.gameObject == GameObject.CUBE ? outtakePower : -outtakePower);
      } else if (System.currentTimeMillis() - shootArmTime >= 700) {
        wrist.setWristPosition(wPositionScoring);

      }
    }
    }

  }


  // This is the intake method which can keep taking in a boolean which decides if it runs
  public void intakeAction(boolean action) {
    
   
    // If the intake is high, then i first set the arm position to straight, set the wirst to high intake
    if (cIntakeType == IntakeType.HIGH) {
      if (action) {
        
        if (arm.setTopPosition(TopArmPosition.HIGHINTAKE) < 0.4) {
          wrist.setWristPosition(WristPosition.HIGHINTAKE);
 // If the motor is stalling the intake current goes around 24, so
        // the intake runs till it stalls and then we just put the intake back up
          intake.runIntake(intake.gameObject == GameObject.CUBE ? -outtakePower : outtakePower);
     
        }
        
       
        
        // Set the LEDS based on what game object we are intaking
        if (intake.gameObject == GameObject.CUBE) {
          // Cube
          arm.setLED(SparkLEDColors.PURPLE);
        } else {
          // Cone
          arm.setLED(SparkLEDColors.YELLOW);
        }

      }

    }
    // if the intake is low, then we 
    else if (cIntakeType == IntakeType.LOW) {
      
      if (action) {

        // if (vision.aimLimelightGameObjectPickup(intake.gameObject)) {

        
        // If the game object is a cone vs a cube we set different wrist positions and led colors
        if (intake.gameObject == GameObject.CONE) {
          wrist.setWristPosition(WristPosition.STRAIGHTCONE);
          arm.setLED(SparkLEDColors.YELLOW);

        } else {
          wrist.setWristPosition(WristPosition.STRAIGHTCUBE);
          arm.setLED(SparkLEDColors.PURPLE);

        }

        // If the motor is stalling the intake current goes around 24, so
        // the intake runs till it stalls and then we just put the intake back up
          intake.runIntake(intake.gameObject == GameObject.CUBE ? -outtakePower : outtakePower);
      // }
      }
      }
      // This is for the ground intake, which would be used to intake
      // a cone that was flipped
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    
    //add wrist offset for cycles 
    // wrist.checkCycle();


    // if(driveController.getYButtonPressed()){
      // arm.topArm.setNeutralMode(NeutralMode.Coast);
      // arm.bottomArm.setNeutralMode(NeutralMode.Coast);
      // wrist.wrist.setNeutralMode(NeutralMode.Coast);
    // }


    SmartDashboard.putNumber("volts bus", arm.topArm.getBusVoltage());
    SmartDashboard.putNumber("motor out ", arm.topArm.getMotorOutputVoltage());
    SmartDashboard.putNumber("wrist encoder", wrist.getWristMotorPosition());
    SmartDashboard.updateValues();

    SmartDashboard.putNumber("Wrist", wrist.getWristMotorPosition());
    SmartDashboard.putNumber("Top Arm", arm.getTopArmPosition());
    // SmartDashboard.putNumber("Bottom Arm", arm.getBottomArmPosition());
    SmartDashboard.putBoolean("deployIntake", deployIntake);
    SmartDashboard.putBoolean("deployShoot", deployShoot);
    SmartDashboard.updateValues();

    // All of the button controls with setting values
    if (xboxController.getYButtonPressed()) {
      intake.gameObject = GameObject.CONE;
      outtakePower = 1.0;
      cIntakeType = IntakeType.LOW;
      deployIntake = !deployIntake;
    }

    if (xboxController.getXButtonPressed()) {
      intake.gameObject = GameObject.CUBE;
      outtakePower = 1.0;
      cIntakeType = IntakeType.LOW;
      deployIntake = !deployIntake;
    }
    if (xboxController.getAButtonPressed()) {
      intake.gameObject = vision.getCurrentDetectedGameObject();
      outtakePower = 1.0;
      cIntakeType = IntakeType.LOW;
      deployIntake = !deployIntake;
    }

    if (xboxController.getBButtonPressed()) {
      intake.gameObject = GameObject.CONE;
      outtakePower = 1.0;
      cIntakeType = IntakeType.GROUND;
      deployIntake = !deployIntake;
    }
    if (xboxController.getLeftBumperPressed()) {
      intake.gameObject = GameObject.CUBE;
      outtakePower = 1.0;
      cIntakeType = IntakeType.HIGH;
      deployIntake = !deployIntake;
    }
    if (xboxController.getRightBumperPressed()) {
      intake.gameObject = GameObject.CONE;
      outtakePower = 1.0;
      cIntakeType = IntakeType.HIGH;
      deployIntake = !deployIntake;
    }

    // boardLowIntake.setBoolean(cI/);
    boardIntakeIn.setBoolean(deployIntake);
    boardShoot.setBoolean(deployShoot);

    SmartDashboard.putNumber("Selected Sensor Intake Velocity", intake.getIntakeCurrent());
    SmartDashboard.putNumber("Intake Motor Velocity", intake.getIntakeVelocity());
    SmartDashboard.putNumber("POV", xboxController.getPOV());
    SmartDashboard.updateValues();

    // This is for the dpad, where if you click the top it will reset all of the properties for the top scoring
    if (xboxController.getPOV() == 0) {
      shootArmTime = -1;
        outtakePower = 0.8;

   

      tPositionScoring = TopArmPosition.STRAIGHT;
      // bPositionScoring = BottomArmPosition.IN;
      if (intake.gameObject == GameObject.CUBE) {

      wPositionScoring = WristPosition.SHOOTINGCUBE;
      }
      else {
        wPositionScoring = WristPosition.SHOOTINGCONE;

      }
      deployShoot = true;

    } 
        // This is for the dpad, where if you click the middle it will reset all of the properties for the middle scoring
    else if (xboxController.getPOV() == 90) {
      shootArmTime = -1;

      outtakePower = 0.6;
      tPositionScoring = TopArmPosition.MIDDLE;
      // bPositionScoring = BottomArmPosition.IN;
      wPositionScoring = WristPosition.MIDDLE;
      deployShoot = true;
    } 
    // This is for the dpad, where if you click the bottom it will reset all of the properties for the bottom scoring
    else if (xboxController.getPOV() == 180) {
      shootArmTime = -1;

      outtakePower = 0.7;
      tPositionScoring = TopArmPosition.DOWN;
      // bPositionScoring = BottomArmPosition.IN;
      wPositionScoring = WristPosition.UP;
        
      deployShoot = true;
    }
    // if you click anywhere else on the dpad it will stop the deploy shoot
    else if (xboxController.getPOV() == 270) {

      deployShoot = false;
    }


    // This is for the deploy shoot and the deploy intake to feed the booleans
    shoot(deployShoot);
    intakeAction(deployIntake);

    // If the intake and shoot are not deployed,
    if (!deployIntake && !deployShoot) {
      // arm.setBottomPosition(BottomArmPosition.IN);
      arm.setTopPosition(TopArmPosition.DOWN);
      wrist.setWristPosition(WristPosition.UP);

      // arm.resetArmEncoders();
      // wrist.resetWristEncoders();
          
        
        // Set the LEDS based on what game object we are intaking
        if (intake.gameObject == GameObject.CUBE) {
          // Cube
          arm.setLED(SparkLEDColors.PURPLE);
        } else if (intake.gameObject == GameObject.CONE) {
          // Cone
          arm.setLED(SparkLEDColors.YELLOW);
        }
        else {
          arm.setLED(SparkLEDColors.RAINBOW);

        }

        if (xboxController.getLeftTriggerAxis() > 0.1) {

          intake.runIntake(0.7);
  
        } else if (xboxController.getRightTriggerAxis() > 0.1) {
  
          intake.runIntake(-0.7);
  
        } else {
          
      // Manual Intake Control
      // if (Math.abs(intake.getIntakeCurrent()) < 22) {
        intake.runIntake(0);

      // }
      // else {
      //   intake.runIntake(gameObject == GameObject.CUBE ? -outtakePower : outtakePower);

      // }

        }
        // Once intaked, we can move the intkae back up
      
     
      
      
    }

    SmartDashboard.putNumber("Top Arm", arm.getTopArmPosition());
    // SmartDashboard.putNumber("Bottom Arm", arm.getBottomArmPosition());
    SmartDashboard.updateValues();

  }

  public void intakeAuto() {
    
    if (intake.getIntakeCurrent() >= 22) {
      deployIntake = false;
    }
      
  }

   // This is the shoot method which can be fed a bool at all times
   public void shootAuto(boolean action) {

    // If the action is true, and the first time this is getting called in a
    // routine, we reset shoot arm time
    // Routine
    // shoot arm time is reseted it is set to current system
    // set the top position of the arm to the current scoring top
    // set the bottom position of the arm to the current scoring bottom
    // time based
    // 5500 ms - resets the arm & wrist completely and stops calling the shoot
    // function
    // 5000 ms - resets the wrist positon back to stowed away state first when
    // resetting
    // 3000 ms - sets intake to run for 2000 ms
    // 700 ms - sets wrist position to the variable pos

    if (action) {
      if (shootArmTime == -1) {
        shootArmTime = System.currentTimeMillis();
      }
      arm.setTopPosition(tPositionScoring);
      // arm.setBottomPosition(bPositionScoring);
    
      if (System.currentTimeMillis() - shootArmTime >= 5000) {

        arm.rotateTop(0);

        wrist.wrist.set(ControlMode.PercentOutput, 0);
        deployShoot = false;

      } else if (System.currentTimeMillis() - shootArmTime >= 3000) {
        
        tPositionScoring = TopArmPosition.DOWN;
        intake.runIntake(0);
        wrist.setWristPosition(WristPosition.UP);
      } else if (System.currentTimeMillis() - shootArmTime >= 2000) {
        intake.runIntake(intake.gameObject == GameObject.CUBE ? outtakePower : -outtakePower);
        arm.setLED(SparkLEDColors.SHOOT);
      } else if (System.currentTimeMillis() - shootArmTime >= 700) {
        wrist.setWristPosition(wPositionScoring);

      }
    }
    

  }

//   // Auto Functions
//   public boolean shootAuto(boolean action) {

//     if (action) {
//       if (shootArmTime == -1) {
//         shootArmTime = System.currentTimeMillis();
//       }
//       arm.setTopPosition(tPositionScoring);
//       // arm.setBottomPosition(bPositionScoring);

//       if (gameObject == GameObject.CUBE) {
//         arm.setLED(SparkLEDColors.PURPLE);
//       } else {
//         // Cone
//         arm.setLED(SparkLEDColors.YELLOW);
//       }
//       if (System.currentTimeMillis() - shootArmTime >= 5000) {
//         tPositionScoring = TopArmPosition.DOWN;
//         bPositionScoring = BottomArmPosition.IN;
//         intake.runIntake(0);
//         if (wrist.setWristPosition(WristPosition.UP) <= 0.05) {
//           SmartDashboard.putBoolean("update", true);
//           SmartDashboard.updateValues();
//           return true;
//         }

        
//         return false;
        

//       } else if (System.currentTimeMillis() - shootArmTime >= 3000) {
//         intake.runIntake(gameObject == GameObject.CUBE ? outtakePower : -outtakePower);
// return false;
//       } else if (System.currentTimeMillis() - shootArmTime >= 700) {
//         wrist.setWristPosition(WristPosition.SHOOTING);
//         return false;
//       }
//       else {
//         return false;
//       }
//     }
//     else {
//       return false;
//     }

//   }

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
