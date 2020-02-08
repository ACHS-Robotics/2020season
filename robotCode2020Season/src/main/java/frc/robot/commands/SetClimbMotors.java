/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.S_Climb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetClimbMotors extends CommandBase {
  /**
   * Runs the talons at 100% speed while held.
   */
  S_Climb sub;
  
  public SetClimbMotors(S_Climb sub) {
   addRequirements(sub);
   this.sub = sub;
  }

  public void execute() {
      sub.setFalcons(1.0);
  } 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
