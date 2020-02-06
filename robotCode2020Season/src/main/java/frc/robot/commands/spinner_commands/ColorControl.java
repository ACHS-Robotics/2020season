/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner_commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.S_Spinner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ColorControl extends PIDCommand {
  /**
   * Creates a new ColorControl.
   */

  private S_Spinner sub;
  private static int setpointColor = -1;

  public ColorControl(S_Spinner sub) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0), // since the furthest away the color will ever be is 2 then just go full speed till you get to it? (P just needs to be 1 or greater?)
        // This should return the measurement
        () -> {
          setpointColor = determineColorSetpoint();
          if (setpointColor == -1){
            return 0;
          }
          else {
            return sub.getDistFromColor(setpointColor);
          }
        }, // negation is used to get correct output sign (could also make this 0 and have a positive lambda function in the third argument)
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          // Use the output here
          sub.updateSBColors();
          sub.declareColor();
          sub.runMotor(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    this.sub = sub;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.0);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    sub.resetColorListCounters();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  public static int determineColorSetpoint(){
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    int color;
    if (gameData.length() > 0){
      switch(gameData.charAt(0)){
        case 'B':
          color = Constants.kBlue; 
          break;
        case 'G':
          color = Constants.kGreen;
          break;
        case 'R':
          color = Constants.kRed;
          break;
        case 'Y':
          color = Constants.kYellow;
          break;
        default:
          color = -1;
          break;
      }
    }
    else {
      color = -1;
    }
    return color;
  }

}
