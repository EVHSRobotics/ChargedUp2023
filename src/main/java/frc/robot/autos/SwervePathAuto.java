// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.autos.SwerveCommand.PathCommandAction;
import frc.robot.commands.FourBar;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class SwervePathAuto extends CommandBase {

  private CommandBase c;
  private FourBar fourBar;
  private Vision vision;
  private TeleopSwerve teleopSwerve;

  public static enum PathType {
      
    
    Basic("Basic"),
    Cube1Cone1 ("1_Cube_1_Cone_Auto"),
    Cube2Cone1 ("2_Cube_1_Cone_Auto"),
    Cube2 ("2_Cube_Auto");


    private String path;

    public String getPath() {
      return path;
    }
    private PathType(String type){
      this.path = type;
    }

}

  Swerve s_Swerve;
  /** Creates a new PathPlanner. */
  public SwervePathAuto(Swerve s_Swerve, PathType pathType, FourBar fourBar, Vision vision, TeleopSwerve teleopSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    this.fourBar = fourBar;
    this.teleopSwerve = teleopSwerve;
    this.vision = vision;
    s_Swerve.resetModulesToAbsolute();


    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathType.getPath(), new PathConstraints(1, 0.5));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("Auto Align Ramp", new SwerveCommand(PathCommandAction.AUTOALIGNRAMP, fourBar, vision, teleopSwerve));
eventMap.put("Outtake Cube", new SwerveCommand(PathCommandAction.OUTTAKEHIGH, fourBar, vision, teleopSwerve));
// eventMap.put("Outake Cube 1", new SwerveCommand(PathCommandAction.OUTTAKECUBE, fourBar, vision, teleopSwerve));
// eventMap.put("Outake Cone 1", new SwerveCommand(PathCommandAction.OUTTAKECONE, fourBar, vision, teleopSwerve));
// eventMap.put("Intake Cube 2", new SwerveCommand(PathCommandAction.INTAKECUBE, fourBar, vision, teleopSwerve));
// eventMap.put("Intake Cone 1", new SwerveCommand(PathCommandAction.INTAKECONE, fourBar, vision, teleopSwerve));
// eventMap.put("marker1", new PrintCommand("Passed marker 1"));
// eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.Swerve.swerveKinematics, // SwerpatveDriveKinematics
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);

// Command fullAuto = autoBuilder.fullAuto(pathGroup);
c = autoBuilder.followPathGroupWithEvents(pathGroup);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerServer.startServer(5811);

     // 5858111 = port number. adjust this according to your needs
    c.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public Command dropBlock(){
    return null;
  }
}
