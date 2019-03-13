/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  public WPI_TalonSRX elevatorM =  new WPI_TalonSRX(constants.elevatorMPort);

  //LIMIT SWITCHES
  public DigitalInput topLimit = new DigitalInput(constants.topLimitPort);
  public DigitalInput bottomLimit = new DigitalInput(constants.bottomLimitPort);

  //POSTION OF ENCODER
  public double elevatorEncPos = elevatorM.getSensorCollection().getQuadraturePosition();

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

  //ELEVATOR UP BUTTON BOARD
  public void elevatorUpButton() {
    if(topLimit.get() == false) {
      elevatorM.set(0); 
    } else {
      elevatorM.set(0.9);
    }
  }
  //ELEVATOR DOWN BUTTON BOARD
  public void elevatorDownButton() {
    if(bottomLimit.get() == false) {
      elevatorM.set(0);
      elevatorM.getSensorCollection().setQuadraturePosition(0, 0);
    } else {
      elevatorM.set(0.9);
    }
  }

  //ELEVATOR STOP METHOD
  public void stopElevator() {
    elevatorM.set(0);
  }

}
