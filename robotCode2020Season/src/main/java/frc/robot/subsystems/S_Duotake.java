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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class S_Duotake extends SubsystemBase {
  /**
   * Creates a new Duotake.
   */

  TalonFX intake;
  TalonSRX topConveyor;
  TalonSRX bottomConveyor;
  DoubleSolenoid sol;



  public S_Duotake() {
    intake = new TalonFX(Constants.intakePort);
    intake.set(ControlMode.PercentOutput, 0);
    intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Brake);

    topConveyor = new TalonSRX(Constants.topConveyorPort);
    bottomConveyor = new TalonSRX(Constants.bottomConveyorPort);
    bottomConveyor.setInverted(true);
    sol = new DoubleSolenoid(Constants.duotakeSolenoidPort, Constants.duotakeSolenoidForward, Constants.duotakeSolenoidReverse);
  
    SmartDashboard.putNumber("extakeSpeed", Constants.defualtExtakeSpeed);
    SmartDashboard.putNumber("extakeSpeedSlow", Constants.defualtExtakeSpeedSlow);
  }

  //TODO: for all motor running methodes may need to change inversions/negate set values
  public void runIntake(double percent){
    intake.set(ControlMode.PercentOutput, percent); 
  }



  public void stopIntake(){
    intake.set(ControlMode.PercentOutput, 0.0);
  }

  public void runExtakeOut(){
    double speed = SmartDashboard.getNumber("extakeSpeed", Constants.defualtExtakeSpeed);
    //System.out.println(speed);
    topConveyor.set(ControlMode.PercentOutput, -speed);
    bottomConveyor.set(ControlMode.PercentOutput, -speed);
  }

  public void runExtakeIn(){ //temp out
    double speed = SmartDashboard.getNumber("extakeSpeed", Constants.defualtExtakeSpeed);
    topConveyor.set(ControlMode.PercentOutput, -speed); //TODO: figure out why polarity of the motor mattered (should be oppoiste negative signs)
    bottomConveyor.set(ControlMode.PercentOutput, -speed);
  }

  public void runExtakeOutSlow(){
    double speed = SmartDashboard.getNumber("extakeSpeedSlow", Constants.defualtExtakeSpeedSlow);
    topConveyor.set(ControlMode.PercentOutput, -speed);
    bottomConveyor.set(ControlMode.PercentOutput, -speed); //TODO: figure out why polarity of the motor mattered (should be oppoiste negative signs)
  }

  public void stopExtake(){
    topConveyor.set(ControlMode.PercentOutput, 0.0);
    bottomConveyor.set(ControlMode.PercentOutput, 0.0);
  }

  //swaps the state of the solenoid for different extake positions
  public void togglePneumatics(){
    if (sol.get() == Value.kReverse){
      sol.set(Value.kForward);
    }
    else {
      sol.set(Value.kReverse);
    }
  }

  public void setPneumaticsLow(){
    sol.set(Value.kForward); //TODO: make sure reverse goes down - it should
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
