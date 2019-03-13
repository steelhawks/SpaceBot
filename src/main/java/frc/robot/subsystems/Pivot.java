/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Gamepad;
import frc.robot.commands.Pivot.PivotGamepad;

public class Pivot extends Subsystem {

  static Constants constants = Constants.getInstance();

  //PIVOT MOTOR TALON
  public WPI_TalonSRX pivotM = new WPI_TalonSRX(constants.pivotMPort);

  //PIVOT LIMIT SWITCH
  public DigitalInput pivotLimit = new DigitalInput(constants.pivotLimitPort);
  
  //POSTION OF ENCODER
  public double pivotEncPos = pivotM.getSensorCollection().getQuadraturePosition();
  
  //ARM PIVOT CONSTRUCTOR
  public Pivot() {
    pivotM.getSensorCollection().setQuadraturePosition(0,0);
  }

  //DEFAULT COMMAND
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PivotGamepad());
  }

  //PIVOTING GAMEPAD
  public void pivotGamepad(Gamepad F310) {
    double y = 0;
    if(pivotLimit.get() == false) {
        if(F310.getRightY() < 0) {
          y = 0;
        } else {
          y = F310.getRightY();
        }
      } else if (pivotEncPos < -950) {
        if(F310.getRightY() > 0) {
          y = 0;
        } else {
          y = F310.getRightY();
        }
      } else {
        y = F310.getRightY();
        if(y < 0){
        y = 0.7*y;
      } else if (y > 0) {
        y = 0.4*y;
      } else {
         y = 0;
        }
      pivotM.set(y);

  if(pivotLimit.get() == false) {
    pivotM.getSensorCollection().setQuadraturePosition(0, 0);
  }
  }
}

//PIVOT UP BUTTON BOARD
public void pivotUpButton() {
  if (pivotLimit.get() == false) {
    pivotM.set(0);
    pivotM.getSensorCollection().setQuadraturePosition(0, 0);
  } else {
    pivotM.set(0.5);
  }
}

//PIVOT DOWN BUTTON BOARD
public void pivotDownButton() {
  if (pivotEncPos < -950) {
    pivotM.set(0);
  } else{
    pivotM.set(-0.5);
  }
}

//STOPPING PIVOT MOTOR
public void pivotStop() {
  pivotM.set(0);
}

//PIVOTING AUTONOMOUSLY
public void pivotAuton(double pos, boolean dir) {
  if (dir == true) {
    pivotM.set(-0.5);
      if (pivotEncPos > pos) {
        constants.autoArm = true;
      } if (pivotEncPos <= pos) {
        constants.autoArm = false;
    }
  } else if (dir == false) {
    pivotM.set(0.5);
      if (-pivotEncPos > pos) {
        constants.autoArm = true;
    }  if (-pivotEncPos <= pos) {
        constants.autoArm = false;
    }
  }
}
}