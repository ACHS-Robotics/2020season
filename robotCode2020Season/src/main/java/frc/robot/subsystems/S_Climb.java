/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class S_Climb extends SubsystemBase {
  /**
   * Creates a new S_Climb.
   */
  public static DoubleSolenoid leftSol = new DoubleSolenoid(Constants.climbLeftSolenoidPort, Constants.climbLeftSolenoidForward, Constants.climbLeftSolenoidReverse);
  public static DoubleSolenoid rightSol = new DoubleSolenoid(Constants.climbRightSolenoidPort, Constants.climbRightSolenoidForward, Constants.climbRightSolenoidReverse);
  private TalonFX leftFalcon = new TalonFX(Constants.winchLeftFalcon);
  private TalonFX rightFalcon = new TalonFX(Constants.winchRightFalcon);

  public S_Climb() { 
    leftFalcon.set(ControlMode.PercentOutput, 0);
    leftFalcon.configFactoryDefault();
    leftFalcon.setNeutralMode(NeutralMode.Brake);

    rightFalcon.set(ControlMode.PercentOutput, 0);
    rightFalcon.configFactoryDefault();
    rightFalcon.setNeutralMode(NeutralMode.Brake);
  }

  public void setFalcons(double percent){
    leftFalcon.set(ControlMode.PercentOutput, percent);
    rightFalcon.set(ControlMode.PercentOutput, percent);
  }
  public void togglePneumatics(){
    if (rightSol.get() == Value.kForward && leftSol.get() == Value.kForward){
      rightSol.set(Value.kReverse);
      leftSol.set(Value.kReverse);
    }
    else if (rightSol.get() == Value.kReverse && leftSol.get() == Value.kReverse){
      rightSol.set(Value.kForward);
      leftSol.set(Value.kForward);
    }
    else { // fixes if pneumatics are in opposite states
      rightSol.set(Value.kReverse);
      leftSol.set(Value.kReverse);
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
