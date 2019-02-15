/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

public class Climber extends Subsystem {

  static Constants constants = Constants.getInstance();

  /*
  //SPARK MAX CLIMBER MOTORS - BRUSHLESS
  public CANSparkMax actuatorMA = new CANSparkMax(constants.actuatorMPortA, MotorType.kBrushless);
  public CANSparkMax actuatorMB = new CANSparkMax(constants.actuatorMPortB, MotorType.kBrushless);
  public CANSparkMax actuatorMC = new CANSparkMax(constants.actuatorMPortC, MotorType.kBrushless);
  public CANSparkMax actuatorMD = new CANSparkMax(constants.actuatorMPortD, MotorType.kBrushless);*/

  //SPARK MAX CLIMBER MOTORS - BRUSHED
  public CANSparkMax actuatorMA = new CANSparkMax(constants.actuatorMPortA, MotorType.kBrushed);
  public CANSparkMax actuatorMB = new CANSparkMax(constants.actuatorMPortB, MotorType.kBrushed);
  public CANSparkMax actuatorMC = new CANSparkMax(constants.actuatorMPortC, MotorType.kBrushed);
  public CANSparkMax actuatorMD = new CANSparkMax(constants.actuatorMPortD, MotorType.kBrushed);

  //TALON SRX DROPDOWN MOTOR
  public WPI_TalonSRX dropdownM = new WPI_TalonSRX(constants.dropdownMPort);

  //NEO MOTOR ENCODER
  public CANEncoder actuatorNeoEnc = actuatorMA.getEncoder();

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
    dropdownM.set(0.75);
  }

  //
}
