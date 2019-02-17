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
import frc.robot.Gamepad;
import frc.robot.commands.Climber.FrontClimberGamepad;

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

  //SPEED CONTROLLER GROUPS
  public SpeedControllerGroup allActuators = new SpeedControllerGroup(actuatorMA, actuatorMB, actuatorMC, actuatorMD);
  public SpeedControllerGroup frontActuators = new SpeedControllerGroup(actuatorMA , actuatorMB);
  public SpeedControllerGroup rearActuators = new SpeedControllerGroup(actuatorMC, actuatorMD);

  //NEO MOTOR ENCODER
  public CANEncoder actuatorNeoEnc = actuatorMA.getEncoder();

  //CLIMBER CONSTRUCTOR
  public Climber() {
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new FrontClimberGamepad());
  }

  //FRONT ACTUATORS GAMEPAD
  public void frontGamepad(Gamepad F310) {
    double y = 0.0;
    y = F310.getLeftY();
    frontActuators.set(y);
  }

  //REAR ACTUATORS GAMEPAD
  public void rearGamepad(Gamepad F310) {
    double y = 0.0;
    y = F310.getRightY();
    rearActuators.set(y);
  }

  //EXTEND ALL FOUR ACTUATORS
  public void climberExtend() {
    allActuators.set(1.0);
  }

  //RETRACT FRONT ACTUATORS
  public void frontRetract() {
    frontActuators.set(-0.5);
  }

  //RETRACT REAR ACTUATORS
  public void rearRetract() {
    rearActuators.set(-0.5);
  }

  //ACTIVATE DROPDOWN MOTOR
  public void dropdownMotor() {
    dropdownM.set(0.75);
  }

  //STOP DROPDOWN MOTOR
  public void stopDropdown() {
    dropdownM.set(0);
  }
}
