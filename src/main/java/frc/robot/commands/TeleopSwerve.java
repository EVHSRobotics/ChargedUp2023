package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.PIDController;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final double degtolerance = 2;
    private PIDController yawController = new PIDController(0.006, 0.00, 0.00);

    private double compassOffset;

    public double driverMatchTime = 0.0;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, XboxController swerveController) {
        this.s_Swerve = s_Swerve;
        this.limiter = new SlewRateLimiter[3];
        for (int i = 0; i < 3; i++) this.limiter[i] = new SlewRateLimiter(3);
        this.limiter[2] = new SlewRateLimiter(3);
        this.swerveController = swerveController;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(s_Swerve);

        driveSpeedEntry = Shuffleboard.getTab("DriverStation_2023").add("Drive Speed", 0.0).getEntry();
        rotationalSpeedEntry = Shuffleboard.getTab("DriverStation_2023").add("Rotational Speed", 0.0).getEntry();
        autoAlignRampEntry = Shuffleboard.getTab("DriverStation_2023").add("Balanced", true).getEntry();

        compassOffset = s_Swerve.gyro.getYaw() + 180;
        yawController.setIntegratorRange(20);
        yawController.setBound(1);

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(limiter[0].calculate(translationSup.getAsDouble()), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(limiter[1].calculate(strafeSup.getAsDouble()), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(limiter[2].calculate(rotationSup.getAsDouble()), Constants.stickDeadband);

        driverMatchTime = DriverStation.getMatchTime();

        
        /* Drive */

        // Turn left
        if(swerveController.getLeftBumper() ){
            if(convertCompass(gyroYaw360()) > 180){
                rotationVal = yawController.calculate((180-convertCompass(gyroYaw360())));
            }
            if(convertCompass(gyroYaw360()) < 180){
                rotationVal = yawController.calculate((180-convertCompass(gyroYaw360())));
            }
        }

        if(swerveController.getRightBumper()){
            if(convertCompass(gyroYaw360()) > 180){
                rotationVal = -yawController.calculate((convertCompass(gyroYaw360()) - 360));
            }
            if(convertCompass(gyroYaw360()) < 180){
                rotationVal = -yawController.calculate((convertCompass(gyroYaw360())));
            }
        }

        SmartDashboard.putNumber("Compass heading", s_Swerve.gyro.getYaw() + 180);
        SmartDashboard.putNumber("Compass heading converted", convertCompass(gyroYaw360()));
        SmartDashboard.updateValues();  
    
        s_Swerve.drive(
                new Translation2d(-translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false, 
                true
            );
        
            driveSpeedEntry.setDouble(translationVal*Constants.Swerve.maxSpeed);
            rotationalSpeedEntry.setDouble(rotationVal*Constants.Swerve.maxAngularVelocity);

     

        
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
    public double gyroYaw360(){
        return s_Swerve.gyro.getYaw() + 180;
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

    public double convertCompass(double heading){
        if(heading >= compassOffset){
            return heading - compassOffset;
        } else {
            return (360 - compassOffset) + heading;
        }
    }

    // If this isn't working change field relative to false
    public void autoAlignRamp() {
                s_Swerve.drive(
                    new Translation2d((5.35-gyroPitch()) * 0.02, 0).times(Constants.Swerve.maxSpeed).times(0.5), 
                    0 * Constants.Swerve.maxAngularVelocity, 
                    true, 
                    true
                );
            
    
    }
}