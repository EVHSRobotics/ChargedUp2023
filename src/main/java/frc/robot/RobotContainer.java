package frc.robot;

import java.sql.Time;
import java.util.HashMap;
import java.util.List;

import org.opencv.video.Video;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.autos.SwerveCommand.PathCommandAction;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final Vision vision;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Commands */
    private final TeleopSwerve s_Swerve;
    private final FourBar fourBar;
    /* Subsystem */
    private final Swerve swerve;
    private final VideoServer videoServer;
    private final Limelight limelight;
    private final Arm arm;
    private final Wrist wrist;
    private final Intake intake;
    private SendableChooser<String> autoChooser;
    // private final AprilScanner aprilScanner;

    private final XboxController driveController;
    private final XboxController operatorController;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve = new Swerve();
        limelight = new Limelight();
        arm = new Arm();
        intake = new Intake();

        wrist = new Wrist();
        videoServer = new VideoServer();
        // aprilScanner = new AprilScanner();

        driveController = new XboxController(Constants.Controller.driveControllerPort);
        operatorController = new XboxController(Constants.Controller.operatorControllerPort);

        autoChooser = new SendableChooser<String>();
        autoChooser.addOption("None", "None");
        autoChooser.setDefaultOption("Left Basic Backup", "Left Basic Backup");
        autoChooser.addOption("Middle Basic Backup", "Middle Basic Backup");
        autoChooser.addOption("Right Basic Backup", "Right Basic Backup");
        autoChooser.addOption("Left Shoot", "Left Shoot");
        autoChooser.addOption("Middle Shoot", "Middle Shoot");
        autoChooser.addOption("Right Shoot", "Right Shoot");
        autoChooser.addOption("2 Cube Balance 28 PT", "2 Cube Balance 28 PT");

        SmartDashboard.putData(autoChooser);
        SmartDashboard.updateValues();

        s_Swerve = new TeleopSwerve(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                driveController
            );

        fourBar = new FourBar(arm, wrist, intake, operatorController, driveController);
        vision = new Vision(swerve, limelight, videoServer, driveController);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));



    }

    public Command[] getTeleCommand() {
        Command[] ret = {s_Swerve, fourBar, vision};
        return ret;
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous

        if (autoChooser.getSelected() == "None") return null;
        // return new exampleAuto(s_Swerve);
        swerve.resetModulesToAbsolute();
        fourBar.arm.resetArmEncoders();
        fourBar.wrist.resetWristEncoder();

        SmartDashboard.putString("auto", autoChooser.getSelected());
        SmartDashboard.updateValues();
        
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoChooser.getSelected(), new PathConstraints(2, 1.5));
    
    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Auto Align Ramp", new SwerveCommand(PathCommandAction.AUTOALIGNRAMP, fourBar, vision, s_Swerve));
    eventMap.put("Outtake High", new SwerveCommand(PathCommandAction.OUTTAKEHIGH, fourBar, vision, s_Swerve));
    eventMap.put("Outtake Mid", new SwerveCommand(PathCommandAction.OUTTAKEMID, fourBar, vision, s_Swerve));
    eventMap.put("Intake", new SwerveCommand(PathCommandAction.INTAKE, fourBar, vision, s_Swerve));
    // eventMap.put("Outake Cube 1", new SwerveCommand(PathCommandAction.OUTTAKECUBE, fourBar, vision, teleopSwerve));
    // eventMap.put("Outake Cone 1", new SwerveCommand(PathCommandAction.OUTTAKECONE, fourBar, vision, teleopSwerve));
    // eventMap.put("Intake Cube 2", new SwerveCommand(PathCommandAction.INTAKECUBE, fourBar, vision, teleopSwerve));
    // eventMap.put("Intake Cone 1", new SwerveCommand(PathCommandAction.INTAKECONE, fourBar, vision, teleopSwerve));
    // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // eventMap.put("intakeDown", new IntakeDown());
    
    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        swerve::getPose, // Pose2d supplier
        swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerpatveDriveKinematics
        new PIDConstants(2, 1.2, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(2.5, 1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
    // PathPlannerServer.startServer(5811);

    // Thread.sleep(1000);
    return autoBuilder.fullAuto(pathGroup);
    // c = autoBuilder.followPathGroupWithEvents(pathGroup);


}
}
