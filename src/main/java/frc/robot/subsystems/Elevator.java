/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Gamepad;

public class Elevator extends Subsystem {
  
  Constants constants = Constants.getInstance();
  
  WPI_TalonSRX elevatorMA = new WPI_TalonSRX(constants.elevatorMPortA);
  WPI_TalonSRX elevatorMB = new WPI_TalonSRX(constants.elevatorMPortB);

  SpeedControllerGroup elevatorMotors = new SpeedControllerGroup(elevatorMA, elevatorMB);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void elevatorGamepad(Gamepad F310) {
  }
}
