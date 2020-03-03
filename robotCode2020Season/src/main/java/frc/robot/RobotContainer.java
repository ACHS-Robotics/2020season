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
import frc.robot.commands.duotake_commands.RunExtakeAndIntake;
import frc.robot.commands.duotake_commands.RunExtakeIn;
import frc.robot.commands.duotake_commands.RunExtakeOut;
import frc.robot.commands.duotake_commands.RunExtakeOutSlowSolo;
import frc.robot.commands.duotake_commands.RunIntakeIn;
import frc.robot.commands.duotake_commands.RunIntakeOut;
import frc.robot.Constants.AutoID;
import frc.robot.commands.auto_commands.PneumaticDown;
import frc.robot.commands.auto_commands.TimedForward;
import frc.robot.commands.climb_commands.ClimbPneumaticControl;
import frc.robot.commands.climb_commands.PrimeWinch;
import frc.robot.commands.drive_commands.ManualDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final RunExtakeAndIntake c_runExtakeAndIntake = new RunExtakeAndIntake(sduotake);
  // TODO: if we need distance or turn pid just change to having a trajectory?

  // controllers
  public static Joystick driveController = new Joystick(Constants.logitechDriveCont);
  public static XboxController weaponsController = new XboxController(Constants.logitechWeaponsCont); //note that arcade board uses xbox

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

    //drive buttons
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

    // weapons buttons
    new JoystickButton(weaponsController, Constants.intakeOutBtn).whileHeld(c_runIntakeOut);
    new POVButton(weaponsController, Constants.intakeInPOV).whileHeld(c_runIntakeIn);
    new POVButton(weaponsController, Constants.extakeOutPOV).whileHeld(c_runExtakeOut);
    new POVButton(weaponsController, Constants.extakeInPOV).whileHeld(c_runExtakeIn);
    new JoystickButton(weaponsController, Constants.armExtendSwitch).whileHeld(c_climbPneumaticControl);
    new JoystickButton(weaponsController, Constants.winchSwitch).whileHeld(c_primeWinch);
    new POVButton(weaponsController, Constants.extakeTogglePOV).whenPressed(() -> {
      sduotake.togglePneumatics();
    }, sduotake);
    new POVButton(weaponsController, Constants.extakeAndIntakePOV).whileHeld(c_runExtakeAndIntake);
    new JoystickButton(weaponsController, Constants.unmap3).whileHeld(new RunExtakeOutSlowSolo(sduotake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    Command command;

    AutoID id = Robot.m_chooser.getSelected();
    switch(id){

      case NONE:
      command = null;
      break;

      case TIMED:
        command = new TimedForward(sdrive, 2.5);
      break;

      case TIMED_SHOOT:
      command = new SequentialCommandGroup(
        new TimedForward(sdrive, 2.5),
        new PneumaticDown(sduotake, 0.5),
        new RunExtakeOut(sduotake, 2.5)
      );

      break;
/*
      case SIMPLE:
      command = new SequentialCommandGroup(
        generateRamseteCommand("straightSimple", true),
        new InstantCommand(() -> {
          System.out.println("duotake");
          sduotake.setPneumaticsLow();
        }, sduotake),
        c_runExtakeOut
      );
      break;
*/
      case SIMPLE_GTFO:
      command = null; //TODO: update
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
          sdrive.getKinematics(), () -> sdrive.getSpeeds(false), sdrive.getLeftTrajPIDController(false),
          sdrive.getRightTrajPIDController(false), (left,right) -> sdrive.setOutput(left,right,false), sdrive);
      break;
      
      default:
        command = null;
      break;
    }

    return command;

  }


  public static RamseteCommand generateRamseteCommand(String pathName, boolean reverse){ //using trajUtil & reversing 
    RamseteCommand command;
    String trajectoryJSON = "paths/"+ pathName +".wpilib.json";

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      
      command = new RamseteCommand(
        trajectory,
        () -> {
          Pose2d pose= sdrive.getPose();
          if (reverse){
            pose = new Pose2d(pose.getTranslation().getX(), pose.getTranslation().getY(), new Rotation2d(pose.getRotation().getRadians()+Math.PI));
          }
          return pose;
        },
        new RamseteController(2.0, 0.7),
        sdrive.getFeedforward(),
        sdrive.getKinematics(),
        () -> sdrive.getSpeeds(reverse),
        sdrive.getLeftTrajPIDController(reverse),
        sdrive.getRightTrajPIDController(reverse),
        (left, right) -> sdrive.setOutput(left, right, reverse),
        sdrive  
      );


    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      command = null;
    }


    return command;
  }


/*
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
        //if (reverse){ //TODO: figure out if this is necessary (flipping every heading that we list in pathweaver so that the headings are the direction of the front of the robot)
        //  tempPose = new Pose2d(tempPose.getTranslation().getX(), tempPose.getTranslation().getY(), new Rotation2d(tempPose.getRotation().getRadians()+Math.PI));
        //}
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
*/
}