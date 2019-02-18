/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Gamepad;
import frc.robot.commands.Climber.FrontClimberGamepad;

public class Climber extends Subsystem {

  static Constants constants = Constants.getInstance();

  //SPARK MAX CLIMBER MOTORS
  public CANSparkMax actuatorMA = new CANSparkMax(constants.actuatorMPortA, MotorType.kBrushless);
  public CANSparkMax actuatorMB = new CANSparkMax(constants.actuatorMPortB, MotorType.kBrushless);
  public CANSparkMax actuatorMC = new CANSparkMax(constants.actuatorMPortC, MotorType.kBrushless);
  public CANSparkMax actuatorMD = new CANSparkMax(constants.actuatorMPortD, MotorType.kBrushless);

  //TALON SRX DROPDOWN MOTOR
  public WPI_TalonSRX dropdownM = new WPI_TalonSRX(constants.dropdownMPort);

  //SPEED CONTROLLER GROUPS
  public SpeedControllerGroup allActuators = new SpeedControllerGroup(actuatorMA, actuatorMB, actuatorMC, actuatorMD);
  public SpeedControllerGroup frontActuators = new SpeedControllerGroup(actuatorMA , actuatorMB);
  public SpeedControllerGroup rearActuators = new SpeedControllerGroup(actuatorMC, actuatorMD);

  //NEO MOTOR ENCODER
  public CANEncoder actuatorNeoEncA = actuatorMA.getEncoder();
  public CANEncoder actuatorNeoEncC = actuatorMC.getEncoder();

  //SPARK PID CONTROLLER
  public CANPIDController actuatorPID = actuatorMA.getPIDController();

  //PID CONSTANTS
  public double kP = 0.1;
  public double kI = 1e-4;
  public double kD = 1;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;

  //CLIMBER CONSTRUCTOR
  public Climber() {
    actuatorNeoEncA.setPosition(0);
    actuatorNeoEncC.setPosition(0);
    System.out.println("Climber encoders reset.");
    //SET PID CONSTANTS
    actuatorPID.setP(kP);
    actuatorPID.setI(kI);
    actuatorPID.setD(kD);
    actuatorPID.setIZone(kIz);
    actuatorPID.setFF(kFF);
    actuatorPID.setOutputRange(kMinOutput, kMaxOutput);
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new FrontClimberGamepad());
  }

  //ACTUATOR PID COMMAND
  public void pidActuator() {
    actuatorPID.setReference(10, ControlType.kPosition);
  }

  //COMBINED ACTUATORS JOYSTICK
  public void actuatorsJoystick(Joystick stick) {
    double y = 0;
    if (getNeoPosA() >= 98 && getNeoPosC() >= 108) {
      if(stick.getY() > 0) {
        y = 0;
      } else {
        y = stick.getY();
      }
    } else if(getNeoPosA() <= 7 && getNeoPosC() <= 4) {
      if(stick.getY() < 0) {
        y = 0;
      } else {
        y = stick.getY();
      }
    } else {
      y = stick.getY();
    }
    allActuators.set(y);
  }

  //FRONT ACTUATORS GAMEPAD
  public void frontGamepad(Gamepad F310) {
    double y = 0;
    if (getNeoPosA() >= 98) {
      if(F310.getLeftY() > 0) {
        y = 0;
      } else {
        y = F310.getLeftY();
      }
    } else if(getNeoPosA() <= 7) {
      if(F310.getLeftY() < 0) {
        y = 0;
      } else {
        y = F310.getLeftY();
      }
    } else {
      y = F310.getLeftY();
    }
      frontActuators.set(y);
  }

  /*//REAR ACTUATORS GAMEPAD
  public void rearGamepad(Gamepad F310) {
    double y = 0.0;
    y = F310.getRightY();
    rearActuators.set(y);
  }*/

  //EXTEND ALL FOUR ACTUATORS
  public void climberExtend() {
    if (getNeoPosA() <= 95 || getNeoPosC() <= 105) {
      allActuators.set(0.75);
    } else {
      allActuators.set(0.0);
    }
  }

  //RETRACT FRONT ACTUATORS
  public void frontRetract() {
    if (getNeoPosA() >= 0) {
      frontActuators.set(-0.5);
    } else {
      frontActuators.set(0.0);
    }
  }

  //RETRACT REAR ACTUATORS
  public void rearRetract() {
    if (getNeoPosC() >= 0) {
      rearActuators.set(-0.5);
    } else {
      rearActuators.set(0.0);
    }
  }

  //ACTIVATE DROPDOWN MOTOR
  public void dropdownMotor() {
    dropdownM.set(0.5);
  }

  //STOP DROPDOWN MOTOR
  public void stopDropdown() {
    dropdownM.set(0);
  }

  //STOP CLIMBER MOTORS
  public void stopClimber() {
    allActuators.set(0);
  }

  public double getNeoPosA() {
    return actuatorNeoEncA.getPosition();
  }

  public double getNeoPosC() {
    return actuatorNeoEncC.getPosition();
  }
}
