/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.duotake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.S_Duotake;

public class RunExtakeOut extends CommandBase {
  /**
   * Creates a new RunExtake.
   */
  private final Timer timer = new Timer();
  S_Duotake sub;
  double interval;

  public RunExtakeOut(S_Duotake sub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
    this.sub = sub;
    this.interval = -1;
  }

  public RunExtakeOut(S_Duotake sub, double interval){
    addRequirements(sub);
    this.sub = sub;
    this.interval = interval;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    sub.runExtakeOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("extakeout");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub.stopExtake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (interval < 0){
      return false;
    }
    else {
      return timer.get() >= interval;
    }
  }
}
