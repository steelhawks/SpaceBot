/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Gamepad;
import frc.robot.commands.Elevator.ElevatorGamepad;

public class Elevator extends Subsystem {
  
  static Constants constants = Constants.getInstance();
  
  //ELEVATOR MOTOR TALONS
  public CANSparkMax elevatorM =  new CANSparkMax(constants.elevatorMPort, MotorType.kBrushless);

  //LIMIT SWITCHES
  public DigitalInput topLimit = new DigitalInput(constants.topLimitPort);
  public DigitalInput bottomLimit = new DigitalInput(constants.bottomLimitPort);

  //ELEVATOR CONSTRUCTOR
  public Elevator() {

  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorGamepad());
  }

  //ELEVATOR GAMEPAD METHOD
  public void elevatorGamepad(Gamepad F310) {
    double y = 0;
    if(topLimit.get() == false) {
      if(F310.getLeftY() < 0) {
        y = 0;
      } else {
        y = F310.getLeftY();
      }
    } else if (bottomLimit.get() == false) {
      if(F310.getLeftY() > 0) {
        y = 0;
      } else {
        y = F310.getLeftY();
      }
    } else {
      y = F310.getLeftY();
    }
    elevatorM.set(y);
  }

  //ELEVATOR STOP METHOD
  public void stopElevator() {
    elevatorM.set(0);
  }

}
