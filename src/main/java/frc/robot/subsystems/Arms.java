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
  public DoubleSolenoid armSol = new DoubleSolenoid(constants.armSolPortOn, constants.armSolPortOff);
  public DoubleSolenoid hatchSol = new DoubleSolenoid(constants.hatchSolPortOn, constants.hatchSolPortOff);

  //IR SENSOR (IF WE USE IT)
  public AnalogInput cargoIR = new AnalogInput(constants.cargoIRPort);

  //ARMS CONSTRUCTOR
  public Arms() {
    armSol.set(DoubleSolenoid.Value.kForward);
    hatchSol.set(DoubleSolenoid.Value.kReverse);
  }

  //DEFAULT COMMAND
  @Override
  public void initDefaultCommand() {
  }

  //INTAKING METHOD
  public void armIntakeButton() {
    if(leftArmM.get() == 0 && rightArmM.get() == 0) {
      leftArmM.set(-0.8);
      rightArmM.set(0.8);
    } else {
      leftArmM.set(0);
      rightArmM.set(0);
    }
  }

  //SHOOTING METHOD
  public void armOuttakeButton() {
    if(leftArmM.get() == 0 && rightArmM.get() == 0) {
      leftArmM.set(0.8);
      rightArmM.set(-0.8);
    } else {
      leftArmM.set(0);
      rightArmM.set(0);
    }
  }

  //STOPPING METHOD
  public void armStopMotors() {
    leftArmM.set(0);
    rightArmM.set(0);
  }

  //ARM PISTON METHOD
  public void armPistons() {
    if(armSol.get() == DoubleSolenoid.Value.kForward) {
      armSol.set(DoubleSolenoid.Value.kReverse);
    } else {
      armSol.set(DoubleSolenoid.Value.kForward);
    }
  }

  //HATCH PISTON METHOD
  public void hatchPiston() {
    if(hatchSol.get() == DoubleSolenoid.Value.kForward) {
      hatchSol.set(DoubleSolenoid.Value.kReverse);
    } else {
      hatchSol.set(DoubleSolenoid.Value.kForward);
    }
  }
}
