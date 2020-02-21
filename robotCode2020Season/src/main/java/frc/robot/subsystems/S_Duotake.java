/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class S_Duotake extends SubsystemBase {
  /**
   * Creates a new Duotake.
   */

  TalonSRX intake;
  TalonSRX topConveyer;
  TalonSRX bottomConveyer;
  DoubleSolenoid sol;



  public S_Duotake() {
    intake = new TalonSRX(Constants.intakePort);
    topConveyer = new TalonSRX(Constants.topConveyerPort);
    bottomConveyer = new TalonSRX(Constants.bottomConveyerPort);
    sol = new DoubleSolenoid(Constants.duotakeSolenoidPort, Constants.duotakeSolenoidForward, Constants.duotakeSolenoidReverse);
  }

  //TODO: for all motor running methodes may need to change inversions/negate set values
  public void runIntake(){
    intake.set(ControlMode.PercentOutput, 1.0); 
  }

  public void stopIntake(){
    intake.set(ControlMode.PercentOutput, 0.0);
  }

  public void runExtakeOut(){
    topConveyer.set(ControlMode.PercentOutput, 1.0);
    bottomConveyer.set(ControlMode.PercentOutput, -1.0);
  }

  public void runExtakeIn(){
    topConveyer.set(ControlMode.PercentOutput, -1.0);
    bottomConveyer.set(ControlMode.PercentOutput, 1.0);
  }

  public void stopExtake(){
    topConveyer.set(ControlMode.PercentOutput, 0.0);
    bottomConveyer.set(ControlMode.PercentOutput, 0.0);
  }

  //swaps the state of the solenoid for different extake positions
  public void togglePneumatics(){
    if (sol.get() == Value.kForward){
      sol.set(Value.kReverse);
    }
    else {
      sol.set(Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
