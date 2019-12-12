/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.RunNeo;

/**
 * Add your docs here.
 */
public class S_Neo extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax lfmoto, lbmoto, rfmoto, rbmoto;

  public S_Neo(){
    lfmoto = new CANSparkMax(RobotMap.NEOlf, MotorType.kBrushless);
    lfmoto.restoreFactoryDefaults();
    lfmoto.setInverted(false);

    lbmoto = new CANSparkMax(RobotMap.NEOlb, MotorType.kBrushless);
    lbmoto.restoreFactoryDefaults();
    lbmoto.setInverted(false);

    rfmoto = new CANSparkMax(RobotMap.NEOrf, MotorType.kBrushless);
    rfmoto.restoreFactoryDefaults();
    rfmoto.setInverted(true);

    rbmoto = new CANSparkMax(RobotMap.NEOrb, MotorType.kBrushless);
    rbmoto.restoreFactoryDefaults();
    rbmoto.setInverted(true);
    


  }
  
  public void runMotor(double left, double right){
   
    lfmoto.set(left);
    lbmoto.set(left);
    
    rfmoto.set(right);
    rbmoto.set(right);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new RunNeo());
  }
}
