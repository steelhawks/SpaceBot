/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Gamepad;
import frc.robot.commands.Climber.FrontClimberGamepad;


public class Climber extends Subsystem {

  public enum ActuatorPosition {
    LEVEL_ZERO,
    LEVEL_ONE, 
    LEVEL_THREE,
    DO_NOTHING
  }

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
  public CANEncoder actuatorNeoEncB = actuatorMB.getEncoder();
  public CANEncoder actuatorNeoEncC = actuatorMC.getEncoder();
  public CANEncoder actuatorNeoEncD = actuatorMD.getEncoder();

  //SPARK PID CONTROLLER
  public CANPIDController actuatorPID = actuatorMA.getPIDController();
  public CANPIDController actuatorPIDB = actuatorMB.getPIDController();
  public CANPIDController actuatorPIDC = actuatorMC.getPIDController();
  public CANPIDController actuatorPIDD = actuatorMD.getPIDController();

  //PID CONSTANTS
  public double kP = 0.1;
  public double kI = 1e-4;
  public double kD = 1;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;

  //CURRENT REFERENCE FOR PID
  private int MA_CURRENT_REFERENCE = 0;

  //CONSTANTS FOR CONTROLLING THE ACTUATORS
  public int actuatorPositionLevelZero = 0;
  public int actuatorPositionLevelThree = 3;
  public int actuatorPositionLevelTwo = 2;
  public int actuatorPositionNothing = -1;

  //CLIMBER CONSTRUCTOR
  public Climber() {
    actuatorNeoEncA.setPosition(0);
    actuatorNeoEncC.setPosition(0);
    System.out.println("Climber encoders reset.");
  }

  //INITIALIZE CLIMBER MOTOR AND PID CONTROLLERS
  public void init(){
    //RESTORE FACTORY DEFAULTS
    actuatorMA.restoreFactoryDefaults();
    actuatorMB.restoreFactoryDefaults();
    actuatorMC.restoreFactoryDefaults();
    actuatorMD.restoreFactoryDefaults();

    //SET MOTORS TO COAST
    actuatorMA.setIdleMode(IdleMode.kCoast);
    actuatorMB.setIdleMode(IdleMode.kCoast);
    actuatorMC.setIdleMode(IdleMode.kCoast);
    actuatorMD.setIdleMode(IdleMode.kCoast);

    //SET PID CONSTANTS
    actuatorPID.setP(kP);
    actuatorPID.setI(kI);
    actuatorPID.setD(kD);
    actuatorPID.setIZone(kIz);
    actuatorPID.setFF(kFF);
    actuatorPID.setOutputRange(kMinOutput, kMaxOutput);

    actuatorPIDB.setP(kP);
    actuatorPIDB.setI(kI);
    actuatorPIDB.setD(kD);
    actuatorPIDB.setIZone(kIz);
    actuatorPIDB.setFF(kFF);
    actuatorPIDB.setOutputRange(kMinOutput, kMaxOutput);

    actuatorPIDC.setP(kP);
    actuatorPIDC.setI(kI);
    actuatorPIDC.setD(kD);
    actuatorPIDC.setIZone(kIz);
    actuatorPIDC.setFF(kFF);
    actuatorPIDC.setOutputRange(kMinOutput, kMaxOutput);

    actuatorPIDD.setP(kP);
    actuatorPIDD.setI(kI);
    actuatorPIDD.setD(kD);
    actuatorPIDD.setIZone(kIz);
    actuatorPIDD.setFF(kFF);
    actuatorPIDD.setOutputRange(kMinOutput, kMaxOutput);
    
    //SET FOLLOW MODE
    actuatorMB.follow(actuatorMA);
    actuatorMD.follow(actuatorMC);
  }

  @Override
  public void initDefaultCommand() {
  }

  //ACTUATOR PID COMMAND
  public void pidActuator(int front, int back) {   
  //SET MOTOR CONTROLLERS TO BRAKE 
    actuatorMA.setIdleMode(IdleMode.kBrake);
    actuatorMB.setIdleMode(IdleMode.kBrake);
    actuatorMC.setIdleMode(IdleMode.kBrake);
    actuatorMD.setIdleMode(IdleMode.kBrake);

  // FRONT ACTUATORS
    if( front == actuatorPositionLevelZero){
        System.out.println("Reset Front");
        CANError error = actuatorPID.setReference(0, ControlType.kPosition);
        this.MA_CURRENT_REFERENCE = 0;
    }
    else if( front == actuatorPositionLevelTwo){
        System.out.println("Level Two Front");
        CANError error = actuatorPID.setReference(20, ControlType.kPosition);
        this.MA_CURRENT_REFERENCE = 20;
    }
    else if( front == actuatorPositionLevelThree){
      System.out.println("Level Three Front");
      CANError error = actuatorPID.setReference(40, ControlType.kPosition);
        this.MA_CURRENT_REFERENCE = 40;
    }

 // BACK ACTUATORS
    if( back == actuatorPositionLevelZero ){
        System.out.println("Reset Back"); 
        CANError errorC = actuatorPIDC.setReference(0, ControlType.kPosition);
        this.MA_CURRENT_REFERENCE = 0;
    }
    else if( back == actuatorPositionLevelTwo){
        System.out.println("Level Two Back");
        CANError errorC = actuatorPIDC.setReference(20, ControlType.kPosition);      
        this.MA_CURRENT_REFERENCE = 20;
    }
    else if( back == actuatorPositionLevelThree){
        System.out.println("Level Three Back");
        CANError errorC = actuatorPIDC.setReference(40, ControlType.kPosition);
        this.MA_CURRENT_REFERENCE = 40;
      }
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
    //allActuators.set(y);
  }

  //FRONT ACTUATORS GAMEPAD
  public void frontGamepad(Gamepad F310) {
    double y = 0;

    if (getNeoPosA() >= 160) {
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
      //frontActuators.set(y);
  }

  //ACTIVATE DROPDOWN MOTOR
  public void dropdownButton() {
    dropdownM.set(0.5);
  }

  //STOP DROPDOWN MOTOR
  public void dropdownStop() {
    dropdownM.set(0);
  }

  //STOP CLIMBER MOTORS
  public void climberStop() {
    allActuators.set(0);
  }

  public double getNeoPosA() {
    return actuatorNeoEncA.getPosition();
  }

  public double getNeoPosC() {
    return actuatorNeoEncC.getPosition();
  }
}
