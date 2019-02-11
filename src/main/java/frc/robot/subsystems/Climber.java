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

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Climber extends Subsystem {

  static Constants constants = Constants.getInstance();

  /*
  //SPARK MAX CLIMBER MOTORS
  public CANSparkMax actuatorMA = new CANSparkMax(constants.actuatorMPortA, MotorType.kBrushless);
  public CANSparkMax actuatorMB = new CANSparkMax(constants.actuatorMPortB, MotorType.kBrushless);
  public CANSparkMax actuatorMC = new CANSparkMax(constants.actuatorMPortC, MotorType.kBrushless);
  public CANSparkMax actuatorMD = new CANSparkMax(constants.actuatorMPortD, MotorType.kBrushless);*/

  //TALON SRX CLIMBER MOTORS
  public WPI_TalonSRX actuatorMA = new WPI_TalonSRX(constants.actuatorMPortA);
  public WPI_TalonSRX actuatorMB = new WPI_TalonSRX(constants.actuatorMPortB);
  public WPI_TalonSRX actuatorMC = new WPI_TalonSRX(constants.actuatorMPortC);
  public WPI_TalonSRX actuatorMD = new WPI_TalonSRX(constants.actuatorMPortD);


  //TALON SRX DROPDOWN MOTOR
  public WPI_TalonSRX dropdownM = new WPI_TalonSRX(constants.dropdownMPort);

  //CLIMBER CONSTRUCTOR
  public Climber() {
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //EXTEND ALL FOUR ACTUATORS
  public void climberExtend() {
    SpeedControllerGroup allActuators = new SpeedControllerGroup(actuatorMA, actuatorMB, actuatorMC, actuatorMD);
    allActuators.set(1.0);
  }

  //RETRACT FRONT ACTUATORS
  public void frontRetract() {
    SpeedControllerGroup frontActuators = new SpeedControllerGroup(actuatorMA , actuatorMB);
    frontActuators.set(-0.5);
  }

  //RETRACT REAR ACTUATORS
  public void rearRetract() {
    SpeedControllerGroup rearActuators = new SpeedControllerGroup(actuatorMC, actuatorMD);
    rearActuators.set(-0.5);
  }

  //ACTIVATE DROPDOWN MOTOR
  public void dropdownMotor() {
    
  }

  //
}
