/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class S_Spinner extends SubsystemBase {
  /**
   * Creates a new S_Spinner.
   */
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  //private Color detectedColor;
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final ColorMatch colorMatcher = new ColorMatch();
  private CANSparkMax motor;
  private CANEncoder encoder;
  private int previousColor = -1;
  private int[] colorList = new int[5]; 


  public S_Spinner(){
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    

  }

  public Color getColor(){
    return colorSensor.getColor();
  }

  public void updateSBColors(){
    Color detectedColor = colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("IR", IR);
    motor = new CANSparkMax(Constants.spinnerMotoPort, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setInverted(false); // TODO: may need to change inversion
    motor.setIdleMode(IdleMode.kBrake);
    //encoder = new CANEncoder(motor);


  }

  public void updateColorList(){
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    int currentColor;
    if (match.color == kBlueTarget) {
      currentColor = Constants.kBlue;
    } else if (match.color == kRedTarget) {
      currentColor = Constants.kBlue;
    } else if (match.color == kGreenTarget) {
      currentColor = Constants.kBlue;
    } else if (match.color == kYellowTarget) {
      currentColor = Constants.kBlue;
    } else {
      currentColor = Constants.kBlue;
    }

    if (currentColor != previousColor){
      colorList[currentColor]++;
      previousColor = currentColor;
    }
  }

  public void resetColorListCounters(){
    colorList = new int[5];
    previousColor = -1;
  }

  public int[] getColorList(){
    return colorList;
  }

  public void declareColor(){
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    String colorString;
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putString("Current Color", colorString);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateColorList();
  }
}
