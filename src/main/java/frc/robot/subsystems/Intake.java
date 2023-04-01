// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public enum GameObject {
    CUBE(new Color(0.143, 0.427, 0.429)),
    CONE(new Color(0.361, 0.524, 0.113)),
    UNKNOWN(new Color(0, 0, 0));

    private Color objectColor;

    public Color getGameObjectColor() {
      return objectColor;
    }
    private GameObject(Color color) {
      this.objectColor = color;
    }
  }

  public GameObject gameObject = GameObject.UNKNOWN;

  private TalonSRX intakeMotor;

  private DigitalInput bannerIntake;
  private boolean didSeeBanner;

  // public ColorSensorV3 intakeColorSensor;
  private ColorMatch intakeColorMatch;
 

  /** Creates a new Intake. */
  public Intake() {

    // IntakeSpinL is master motor
    intakeMotor = new TalonSRX(Constants.Intake.intakeMotor);

    
    bannerIntake = new DigitalInput(Constants.Intake.banner);
    

    // intakeColorSensor = new ColorSensorV3(Port.kOnboard);
    
    intakeColorMatch = new ColorMatch();
    intakeColorMatch.addColorMatch(GameObject.CONE.getGameObjectColor());
    intakeColorMatch.addColorMatch(GameObject.CUBE.getGameObjectColor());
    intakeColorMatch.addColorMatch(GameObject.UNKNOWN.getGameObjectColor());
    intakeColorMatch.addColorMatch(new Color(0.561, 0.232, 0.114));
    intakeColorMatch.addColorMatch(new Color(0.197, 0.561, 0.240));


    didSeeBanner = false;
  }



  // Gets intake current in Amps
  // more than 20 AMPS = Motor stalling
  public double getIntakeCurrent() {
    return Math.abs(intakeMotor.getStatorCurrent());
  }
  // Matches the color reading from the color sensor the closest color
  public GameObject getColorSensorGameObject() {

    // Color colorRetrieved = intakeColorSensor.getColor();
    ColorMatchResult matchedColorRetrieved = intakeColorMatch.matchClosestColor(new Color(0, 0, 0));

    if (matchedColorRetrieved.confidence >= 0.8) {
      if (matchedColorRetrieved.color.equals(GameObject.CONE.getGameObjectColor())) {
        return GameObject.CONE;
      }
      else if (matchedColorRetrieved.color.equals(GameObject.CUBE.getGameObjectColor())) {
        return GameObject.CUBE;
      }
      else {
        return GameObject.UNKNOWN;
      }
    }
    else {
      return GameObject.UNKNOWN;
    }

  }

  public void runIntake(double powDouble) {
  
    // if (!getIntakeBanner() && !didSeeBanner) {
      SmartDashboard.putNumber("helel", powDouble);
      SmartDashboard.updateValues();
      intakeMotor.set(ControlMode.PercentOutput, powDouble);

    // }
  
    // else {
      
    //   didSeeBanner = true;

    //   if (powDouble < 0) {
    //     intakeMotor.set(ControlMode.PercentOutput, powDouble);
    //   }
    //   else {
    //     intakeMotor.set(ControlMode.PercentOutput, 0);

    //   }
    // }
  }

  public boolean getIntakeBanner() {
    return bannerIntake.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
