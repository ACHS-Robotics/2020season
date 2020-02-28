/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.Spline.ControlVector;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.limelight.LimeLight;
import frc.robot.commands.drive_commands.SetAngle;
import frc.robot.commands.duotake_commands.RunExtakeIn;
import frc.robot.commands.duotake_commands.RunExtakeOut;
import frc.robot.commands.duotake_commands.RunIntakeIn;
import frc.robot.commands.duotake_commands.RunIntakeOut;
import frc.robot.Constants.AutoID;
import frc.robot.commands.climb_commands.ClimbPneumaticControl;
import frc.robot.commands.climb_commands.PrimeWinch;
import frc.robot.commands.drive_commands.ManualDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  Compressor comp;

  // SUBSYSTEMS
  public final static S_Drive sdrive = new S_Drive();
  private final S_Climb sclimb = new S_Climb();
  private final S_Duotake sduotake = new S_Duotake();

  private final ManualDrive c_manualDrive = new ManualDrive(sdrive);
  private final RunIntakeOut c_runIntakeOut = new RunIntakeOut(sduotake);
  private final RunIntakeIn c_runIntakeIn = new RunIntakeIn(sduotake);
  private final RunExtakeOut c_runExtakeOut = new RunExtakeOut(sduotake);
  private final RunExtakeIn c_runExtakeIn = new RunExtakeIn(sduotake);
  private final ClimbPneumaticControl c_climbPneumaticControl = new ClimbPneumaticControl(sclimb);
  private final PrimeWinch c_primeWinch = new PrimeWinch(sclimb);
  // TODO: if we need distance pid just change to having a trajectory?
  // private final SetAngle c_setAngle = new SetAngle(sdrive);

  // controllers
  public static Joystick driveController = new Joystick(Constants.logitechDriveCont);
  public static Joystick weaponsController = new Joystick(Constants.logitechWeaponsCont);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    sdrive.setDefaultCommand(c_manualDrive);

    
    comp = new Compressor(Constants.compressorModule);
    comp.setClosedLoopControl(true);
     
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driveController, Constants.buttonA).whileHeld(new
    // SetAngle(sdrive));
    // new JoystickButton(driveController, Constants.buttonB).toggleWhenPressed(new
    // DistancePID(sdrive)); //TODO: learn how toggle works (doesn't seem to work
    // how i think it does)
    // new JoystickButton(driveController, Constants.rightBumper).whileHeld(new
    // KeepAngle(sdrive)); TODO: make P bigger to help out with this
    new POVButton(driveController, Constants.dpadUp).whenPressed(() -> {
      sdrive.runMotor(.5, .5);
      Timer.delay(.1);
      sdrive.runMotor(0, 0);
    }, sdrive);
    new POVButton(driveController, Constants.dpadDown).whenPressed(() -> {
      sdrive.runMotor(-.5, -.5);
      Timer.delay(.1);
      sdrive.runMotor(0, 0);
    }, sdrive);
    new POVButton(driveController, Constants.dpadLeft).whenPressed(() -> {
      sdrive.runMotor(-.5, .5);
      Timer.delay(.05);
      sdrive.runMotor(0, 0);
    }, sdrive);
    new POVButton(driveController, Constants.dpadRight).whenPressed(() -> {
      sdrive.runMotor(.5, -.5);
      Timer.delay(.05);
      sdrive.runMotor(0, 0);
    }, sdrive);


    //new JoystickButton(weaponsController, Constants.spinPneuToggleBtn).whenPressed(() -> {
    //  
    //})

    //new POVButton(weaponsController, Constants.extakeTogglePOV).whenPressed()

    new JoystickButton(weaponsController, Constants.intakeOutBtn).whileHeld(c_runIntakeOut);
    new POVButton(weaponsController, Constants.intakeInPOV).whileHeld(c_runIntakeIn);
    new POVButton(weaponsController, Constants.extakeOutPOV).whileHeld(c_runExtakeOut);
    new POVButton(weaponsController, Constants.extakeInPOV).whileHeld(c_runExtakeIn);
    new JoystickButton(weaponsController, Constants.armExtendSwitch).whileHeld(c_climbPneumaticControl);
    new JoystickButton(weaponsController, Constants.winchSwitch).whileHeld(c_primeWinch);
    new POVButton(weaponsController, Constants.extakeTogglePOV).whenPressed(() -> {
      sduotake.togglePneumatics();
    }, sduotake);

    // new JoystickButton(driveController, Constants.buttonA).whenPressed(new
    // SetAngle(sdrive, new LimeLight()));
/*
    new JoystickButton(weaponsController, Constants.greenTopButton).whenPressed(new RunIntake(sduotake));
    new JoystickButton(weaponsController, Constants.blueTopButton).whenPressed(new RunExtakeOut(sduotake));
    new JoystickButton(weaponsController, Constants.yellowTopButton).whenPressed(new RunExtakeIn(sduotake));
    new JoystickButton(weaponsController, Constants.greenTopButton).whenPressed(() -> {
      // invert solenoid state
      sduotake.togglePneumatics();
    }, sduotake);
    */
    // TODO: remap button
    //new JoystickButton(weaponsController, Constants.greenTopButton).whileHeld(new SetClimbMotors(sclimb));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    Command command;

    AutoID id = Robot.m_chooser.getSelected();
    /*switch(id){

      case HIGHTIDE:
        System.out.println("hightide");
        command = new SequentialCommandGroup(
          new ParallelRaceGroup(
            generateRamseteCommand("path1", false),
            new RunIntake(sduotake)
          ),  
          generateRamseteCommand("path2", true),
          new ParallelRaceGroup(
            generateRamseteCommand("path3", false),
            new RunIntake(sduotake)
          ),
          generateRamseteCommand("path4", true)
        );
        break;

      case RIGHT:
        System.out.println("right");
        command = new SequentialCommandGroup(
          new ParallelRaceGroup(
            generateRamseteCommand("path1", false),
            new RunIntake(sduotake)
          ),  
          generateRamseteCommand("path2", true),
          new ParallelRaceGroup(
            generateRamseteCommand("path3", false),
            new RunIntake(sduotake)
          ),
          generateRamseteCommand("path4", true)
        );
        break;

      case WIN:
        System.out.println("win");
        command = new SequentialCommandGroup(
          new ParallelRaceGroup(
            generateRamseteCommand("path1", false),
            new RunIntake(sduotake)
          ),  
          generateRamseteCommand("path2", true),
          new ParallelRaceGroup(
            generateRamseteCommand("path3", false),
            new RunIntake(sduotake)
          ),
          generateRamseteCommand("path4", true)
        );
        break;

      case TEST:
        TrajectoryConfig config = new TrajectoryConfig(Constants.maxTrajVelocity, Constants.maxTrajAcceleration);
        config.setKinematics(sdrive.getKinematics());
        // An example trajectory to follow. All units in meters.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
    
        command = new RamseteCommand(trajectory, sdrive::getPose, new RamseteController(2.0, 0.7), sdrive.getFeedforward(),
            sdrive.getKinematics(), sdrive::getSpeeds, sdrive.getLeftTrajPIDController(),
            sdrive.getRightTrajPIDController(), sdrive::setOutput, sdrive);
        break;

      default:
        command = null;
        break;
    }
*/ command = null; // TODO: get rid of

    return command;

  }


  //

  public static RamseteCommand generateRamseteCommand(String pathName, boolean reverse) { //TODO: figure out what permutation of the reversals works to get the wanted behavior

    RamseteCommand command;
    String pathJSON = "paths/"+ pathName +".path";

    try{

      List<Pose2d> waypoints = new ArrayList<>();

      Path pathsPath = Filesystem.getDeployDirectory().toPath().resolve(pathJSON);
      String contents = new String(Files.readAllBytes(pathsPath));
      String[] lines = contents.split("\n");

      for(int i = 1; i < lines.length; i++){ // first line (index 0) is just description
        String[] lineProperties = lines[i].split(",");
        // 0 = x, 1 = y, 2 = xtan, 3 = ytan 
        Pose2d tempPose = new Pose2d(
          Double.parseDouble(lineProperties[0]),
          Double.parseDouble(lineProperties[1]), 
          new Rotation2d(
            Math.atan2(
              Double.parseDouble(lineProperties[3]),
              Double.parseDouble(lineProperties[2])
            )
          )
        );
        /*
        if (reverse){ //TODO: figure out if this is necessary (flipping every heading that we list in pathweaver so that the headings are the direction of the front of the robot)
          tempPose = new Pose2d(tempPose.getTranslation().getX(), tempPose.getTranslation().getY(), new Rotation2d(tempPose.getRotation().getRadians()+Math.PI));
        }
        */
        waypoints.add(tempPose);

      }

      sdrive.setPose(waypoints.get(0)); //TODO: make sure this works with reversals - this placement could also be bad for other reasons (may only want to set on first go around)

      TrajectoryConfig config = new TrajectoryConfig(Constants.maxTrajVelocity, Constants.maxTrajAcceleration);
      config.setKinematics(sdrive.getKinematics());
      config.setReversed(reverse);
      
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

      command = new RamseteCommand(
        trajectory,
        //sdrive::getPose,
        
        () -> {
          Pose2d pose= sdrive.getPose();
          if (reverse){ //TODO: make sure this method provider works & check if i even should be reversing the pose heading that we are getting
            pose = new Pose2d(pose.getTranslation().getX(), pose.getTranslation().getY(), new Rotation2d(pose.getRotation().getRadians()+Math.PI));
          }
          return pose;
        },
        
        new RamseteController(2.0, 0.7),
        sdrive.getFeedforward(),
        sdrive.getKinematics(), 
        sdrive::getSpeeds,
        sdrive.getLeftTrajPIDController(),
        sdrive.getRightTrajPIDController(),
        sdrive::setOutput,
        sdrive
      );

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathJSON, ex.getStackTrace());
      command = null;
    }


    return command;

  }
}