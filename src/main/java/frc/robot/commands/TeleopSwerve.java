package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private XboxController swerveController;
    private SlewRateLimiter[] limiter;

    private GenericEntry driveSpeedEntry;
    private GenericEntry rotationalSpeedEntry;
    private GenericEntry autoAlignRampEntry;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, XboxController swerveController) {
        this.s_Swerve = s_Swerve;
        this.limiter = new SlewRateLimiter[3];
        for (int i = 0; i < 3; i++) this.limiter[i] = new SlewRateLimiter(2.5);
        this.limiter[2] = new SlewRateLimiter(2.5);
        this.swerveController = swerveController;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(s_Swerve);

        driveSpeedEntry = Shuffleboard.getTab("DriverStation_2023").add("Drive Speed", 0.0).getEntry();
        rotationalSpeedEntry = Shuffleboard.getTab("DriverStation_2023").add("Rotational Speed", 0.0).getEntry();
        autoAlignRampEntry = Shuffleboard.getTab("DriverStation_2023").add("Balanced", true).getEntry();

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(limiter[0].calculate(translationSup.getAsDouble()), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(limiter[1].calculate(strafeSup.getAsDouble()), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(limiter[2].calculate(rotationSup.getAsDouble()), Constants.stickDeadband);
        SmartDashboard.putNumber("translationVal", translationVal);
        SmartDashboard.updateValues();
        /* Drive */

        s_Swerve.drive(
                new Translation2d(-translationVal, -strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false, 
                true
            );
        
            driveSpeedEntry.setDouble(translationVal*Constants.Swerve.maxSpeed);
            rotationalSpeedEntry.setDouble(rotationVal*Constants.Swerve.maxAngularVelocity);

        SmartDashboard.putNumber("AHRS Nav Angle X", s_Swerve.gyro.getQuaternionX());
        SmartDashboard.putNumber("AHRS Nav Angle Y", s_Swerve.gyro.getQuaternionY());
        SmartDashboard.putNumber("AHRS Nav Angle Z", s_Swerve.gyro.getQuaternionZ());
        SmartDashboard.putNumber("AHRS Nav Angle Roll", gyroRoll());
        SmartDashboard.putNumber("AHRS Nav Angle Pitch", gyroPitch());
        SmartDashboard.putNumber("AHRS Nav Angle Yaw", gyroYaw());
        SmartDashboard.putNumber("AHRS Nav Angle Deg", s_Swerve.gyro.getRotation2d().getDegrees());
        SmartDashboard.putNumber("AHRS Nav Angle Cos", s_Swerve.gyro.getRotation2d().getCos());
        SmartDashboard.putNumber("AHRS Nav Angle Sin", s_Swerve.gyro.getRotation2d().getSin());
        SmartDashboard.putNumber("AHRS Nav Angle rot", s_Swerve.gyro.getRotation2d().getRotations());
        SmartDashboard.putNumber("AHRS Nav Angle angle", s_Swerve.gyro.getAngle());
        SmartDashboard.putNumber("AHRS Nav Angle heading", s_Swerve.gyro.getCompassHeading());
        SmartDashboard.updateValues();
        
        if (swerveController.getXButton()) {

            autoAlignRamp();
        }
        
        
        if (MathUtil.applyDeadband(5.35-gyroPitch(), 0.05) == 0) {
            autoAlignRampEntry.setBoolean(true);
        }
        else {
            autoAlignRampEntry.setBoolean(false);
        }
    }
    public double gyroPitch() {
        return (s_Swerve.gyro.getPitch());
    }
    public double gyroRoll() {
        return (s_Swerve.gyro.getRoll());
    }
    public double gyroYaw() {
        return (s_Swerve.gyro.getAngle());
    }
    public void autoAlignRamp() {
            

                s_Swerve.drive(
                    new Translation2d((5.35-gyroPitch()) * 0.02, 0).times(Constants.Swerve.maxSpeed).times(0.5), 
                    0 * Constants.Swerve.maxAngularVelocity, 
                    false, 
                    true
                );
            
    
    }
}