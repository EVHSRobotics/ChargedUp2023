package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveAuto extends SequentialCommandGroup {
    String swervePath;
    public SwerveAuto(Swerve s_Swerve, String swervePath){
        this.swervePath = swervePath;
   try {

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/output/TestPath.wpilib.json");

    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    
   
     ProfiledPIDController thetaController =
     new ProfiledPIDController(
         Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
 thetaController.enableContinuousInput(-Math.PI, Math.PI);

 SwerveControllerCommand swerveControllerCommand =
     new SwerveControllerCommand(
        trajectory,
         s_Swerve::getPose,
         Constants.Swerve.swerveKinematics,
         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
         thetaController,
         s_Swerve::setModuleStates,
         s_Swerve);


 addCommands(
     new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
     swerveControllerCommand
 );
   }
   catch (IOException ex) {
    SmartDashboard.putString("ERROR", ex.getLocalizedMessage());
    SmartDashboard.updateValues();
   }

       
    }
}