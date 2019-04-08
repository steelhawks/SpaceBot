/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Arms extends Subsystem {

  static Constants constants = Constants.getInstance();

  //ARM MOTOR TALONS
  public WPI_TalonSRX leftArmM = new WPI_TalonSRX(constants.leftArmMPort);
  public WPI_TalonSRX rightArmM = new WPI_TalonSRX(constants.rightArmMPort);

  //ARM SOLENOIDS
  public DoubleSolenoid lockSol = new DoubleSolenoid(constants.lockSolPortOn, constants.lockSolPortOff);
  public DoubleSolenoid hatchSol = new DoubleSolenoid(constants.hatchSolPortOn, constants.hatchSolPortOff);

  //ARMS CONSTRUCTOR
  public Arms() {
    hatchSol.set(DoubleSolenoid.Value.kReverse);
    lockSol.set(DoubleSolenoid.Value.kReverse);
  }

  //DEFAULT COMMAND
  @Override
  public void initDefaultCommand() {
  }

  //INTAKING METHOD
  public void armIntakeButton() {
    if(rightArmM.get() == 0) {
      rightArmM.set(0.8);
    } else {
      rightArmM.set(0);
    }
  }

  //SHOOTING METHOD
  public void armOuttakeButton() {
    if(rightArmM.get() == 0) {
      rightArmM.set(-0.8);
    } else {
      rightArmM.set(0);
    }
  }

  //STOPPING METHOD
  public void armStopMotors() {
    rightArmM.set(0);
  }

  //HATCH PISTON METHOD
  public void hatchPiston() {
    if(hatchSol.get() == DoubleSolenoid.Value.kForward) {
      hatchSol.set(DoubleSolenoid.Value.kReverse);
    } else {
      hatchSol.set(DoubleSolenoid.Value.kForward);
    }
  }

  //LOCK PISTON METHOD
  public void lockPiston() {
    if(lockSol.get() == DoubleSolenoid.Value.kForward) {
      lockSol.set(DoubleSolenoid.Value.kReverse);
    } else {
      lockSol.set(DoubleSolenoid.Value.kForward);
    }
  }
}
