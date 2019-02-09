/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Gamepad;

public class ArmPivot extends Subsystem {

  static Constants constants = Constants.getInstance();

  //PIVOT MOTOR TALON
  public WPI_TalonSRX pivotM = new WPI_TalonSRX(constants.pivotMPort);
  public double pivotEncPos = pivotM.getSensorCollection().getQuadraturePosition();
  
  //ARM PIVOT CONSTRUCTOR
  public ArmPivot() {
    pivotM.getSensorCollection().setQuadraturePosition(0,0);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new MySpecialCommand());
  }

  //PIVOTING METHOD
  public void pivotGamepad(Gamepad F310) {
    }
}
