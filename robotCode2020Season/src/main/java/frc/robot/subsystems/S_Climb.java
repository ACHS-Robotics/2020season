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
  public static DoubleSolenoid leftSol;
  public static DoubleSolenoid rightSol;
  private TalonFX falcon = new TalonFX(Constants.winchFalcon);

  public S_Climb() { 
    leftSol = new DoubleSolenoid(Constants.climbLeftSolenoidPort, Constants.climbLeftSolenoidForward, Constants.climbLeftSolenoidReverse);
    rightSol = new DoubleSolenoid(Constants.climbRightSolenoidPort, Constants.climbRightSolenoidForward, Constants.climbRightSolenoidReverse);

    falcon.set(ControlMode.PercentOutput, 0);
    falcon.configFactoryDefault();
    falcon.setNeutralMode(NeutralMode.Brake);
  }

  public void setFalcons(double percent){
    falcon.set(ControlMode.PercentOutput, percent);
  }

  public void reversePneumatics(){
    rightSol.set(Value.kReverse);
    leftSol.set(Value.kReverse);
  }

  public void forwardPneumatics(){
    rightSol.set(Value.kForward);
    leftSol.set(Value.kForward);
  }

  public void togglePneumatics(){
    if (rightSol.get() == Value.kForward && leftSol.get() == Value.kForward){
      reversePneumatics();
    }
    else if (rightSol.get() == Value.kReverse && leftSol.get() == Value.kReverse){
      forwardPneumatics();
    }
    else { // fixes if pneumatics are in opposite states
      reversePneumatics();
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
