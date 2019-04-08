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

import com.revrobotics.CANSparkMax.ExternalFollower;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

import java.lang.Thread;

public class Climber extends Subsystem {

  //SPARK MAX CLIMBER MOTORS
  public CANSparkMax actuatorMFrontLeft = new CANSparkMax(Constants.actuatorMPortA, MotorType.kBrushless);
  public CANSparkMax actuatorMFrontRight = new CANSparkMax(Constants.actuatorMPortB, MotorType.kBrushless);
  public CANSparkMax actuatorMBackLeft = new CANSparkMax(Constants.actuatorMPortC, MotorType.kBrushless);
  public CANSparkMax actuatorMBackRight = new CANSparkMax(Constants.actuatorMPortD, MotorType.kBrushless);

  //TALON SRX DROPDOWN MOTOR
  public WPI_TalonSRX dropdownM = new WPI_TalonSRX(Constants.dropdownMPort);

  //SPEED CONTROLLER GROUPS
  public SpeedControllerGroup allActuators = 
    new SpeedControllerGroup(actuatorMFrontLeft, 
                            actuatorMFrontRight, 
                            actuatorMBackLeft, 
                            actuatorMBackRight);
  
  public SpeedControllerGroup frontActuators = 
    new SpeedControllerGroup(actuatorMFrontLeft , 
                              actuatorMFrontRight);
  
  public SpeedControllerGroup rearActuators = 
    new SpeedControllerGroup(actuatorMBackLeft, 
                            actuatorMBackRight);

  //NEO MOTOR ENCODER
  public CANEncoder actuatorNeoEncFrontLeft = actuatorMFrontLeft.getEncoder();
  public CANEncoder actuatorNeoEncFrontRight = actuatorMFrontRight.getEncoder();
  public CANEncoder actuatorNeoEncBackLeft = actuatorMBackLeft.getEncoder();
  public CANEncoder actuatorNeoEncBackRight = actuatorMBackRight.getEncoder();

  //SPARK PID CONTROLLER
  public CANPIDController actuatorPID = actuatorMFrontLeft.getPIDController();
  public CANPIDController actuatorPIDB = actuatorMFrontRight.getPIDController();
  public CANPIDController actuatorPIDC = actuatorMBackLeft.getPIDController();
  public CANPIDController actuatorPIDD = actuatorMBackRight.getPIDController();

  //PID CONSTANTS
  public double kP = 1;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 0.25;
  public double kMinOutput = -0.25;

  public String actuator_state = "RETRACTED";
  public String actuator_back_state = "RETRACTED";

  public int target_position = 0;
  public int back_target_position = 0;

  //CONSTANTS FOR CONTROLLING THE ACTUATORS LEVELS
  public int actuatorPositionLevelZero = 0;
  public int actuatorPositionLevelThree = 3;
  public int actuatorPositionLevelTwo = 2;
  public int actuatorPositionNothing = -1;

  //CLIMBER CONSTRUCTOR
  public Climber() {
    actuatorNeoEncFrontLeft.setPosition(0);
    actuatorNeoEncFrontRight.setPosition(0);
    actuatorNeoEncBackLeft.setPosition(0);
    actuatorNeoEncBackRight.setPosition(0);
    System.out.println("Climber encoders reset.");
  }

  //INITIALIZE CLIMBER MOTOR AND PID CONTROLLERS
  public void init(){
    //RESTORE FACTORY DEFAULTS
    actuatorMFrontLeft.restoreFactoryDefaults();
    actuatorMFrontRight.restoreFactoryDefaults();
    actuatorMBackLeft.restoreFactoryDefaults();
    actuatorMBackRight.restoreFactoryDefaults();

    //SET MOTORS TO COAST
    actuatorMFrontLeft.setIdleMode(IdleMode.kCoast);
    actuatorMFrontRight.setIdleMode(IdleMode.kCoast);
    actuatorMBackLeft.setIdleMode(IdleMode.kCoast);
    actuatorMBackRight.setIdleMode(IdleMode.kCoast);

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

    // actuatorMFrontRight.follow(actuatorMFrontLeft);
    // actuatorMBackRight.follow(actuatorMBackLeft);
  
  }

  @Override
  public void initDefaultCommand() {
  }

  //ACTUATOR PID COMMAND
  public void actuatorPIDButton(int front, int back) {   

    //SET MOTOR CONTROLLERS TO BRAKE 
    actuatorMFrontLeft.setIdleMode(IdleMode.kBrake);
    actuatorMFrontRight.setIdleMode(IdleMode.kBrake);
    actuatorMBackLeft.setIdleMode(IdleMode.kBrake);
    actuatorMBackRight.setIdleMode(IdleMode.kBrake);

    // // set the follows
    // ExternalFollower disabled = ExternalFollower.kFollowerDisabled;

    // actuatorMFrontRight.follow(disabled, 0);
    // actuatorMBackLeft.follow(disabled, 0);
    // actuatorMBackRight.follow(disabled, 0);

    if( front == back ){

      if( front == actuatorPositionLevelTwo){

        actuatorPID.setReference(48, ControlType.kPosition);
        actuatorPIDB.setReference(48, ControlType.kPosition);
        actuatorPIDC.setReference(48, ControlType.kPosition);
        actuatorPIDD.setReference(48, ControlType.kPosition);

        return;
      }

      if( front == actuatorPositionLevelThree){
        

        // actuatorMFrontRight.follow(actuatorMFrontLeft);
        // actuatorMBackRight.follow(actuatorMBackLeft);
  
  
        System.out.println("ALL EXTEND TO THREE");

        actuatorPID.setReference(108, ControlType.kPosition);
        actuatorPIDB.setReference(108, ControlType.kPosition);
        actuatorPIDC.setReference(108, ControlType.kPosition);
        actuatorPIDD.setReference(108, ControlType.kPosition);

        return;
      }

    }
    else if( front == actuatorPositionLevelZero ){
      actuatorPID.setReference(0, ControlType.kPosition);
      actuatorPIDB.setReference(0, ControlType.kPosition);
    }
    else if( back == actuatorPositionLevelZero ){
      actuatorPIDC.setReference(0, ControlType.kPosition);
      actuatorPIDD.setReference(0, ControlType.kPosition);
    }

    return;





//     // set the back/front state just in case the Robot.java does not set it for us
//     if( back == actuatorPositionLevelZero ){
//       actuator_back_state = "EXTENDED";
//       back_target_position = 4;
//     }

//     if( front == actuatorPositionLevelZero ){
//       actuator_state = "EXTENDED";
//       target_position = 4;
//     }

//     if( back == actuatorPositionLevelThree ){
//       actuator_back_state = "RETRACTED";
//       back_target_position = 108;
//     }

//     if( front == actuatorPositionLevelThree ){
//       actuator_state = "RETRACTED";
//       target_position = 108;
//     }

//     if( back == actuatorPositionLevelTwo ){
//       actuator_back_state = "RETRACTED";
//       back_target_position = 48;
//     }

//     if( front == actuatorPositionLevelTwo ){
//       actuator_state = "RETRACTED";
//       target_position = 48;
//     }
    
//  // BACK ACTUATORS
//  if( back == actuatorPositionLevelZero ){
//   System.out.println("Reset Back"); 
//   CANError errorC = actuatorPIDC.setReference(0, ControlType.kPosition);
// }
// else if( back == actuatorPositionLevelTwo){
//   System.out.println("Level Two Back");
//   CANError errorC = actuatorPIDC.setReference(48, ControlType.kPosition);      
// }
// else if( back == actuatorPositionLevelThree){
//   System.out.println("Level Three Back");
//   CANError errorC = actuatorPIDC.setReference(108, ControlType.kPosition);
//   System.out.println("finished setting to lvl3");
//   try {
//     Thread.sleep(180);
//   }
//   catch(Exception e){
//     System.out.println("Error");
//   }
// }

//   // FRONT ACTUATORS
//     if( front == actuatorPositionLevelZero){
//         System.out.println("Reset Front");
//         CANError error = actuatorPID.setReference(0, ControlType.kPosition);
//     }
//     else if( front == actuatorPositionLevelTwo){
//         System.out.println("Level Two Front");
//         CANError error = actuatorPID.setReference(48, ControlType.kPosition);
//     }
//     else if( front == actuatorPositionLevelThree){
//       System.out.println("Level Three Front");
//       CANError error = actuatorPID.setReference(108, ControlType.kPosition);
//     }
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
  public void actuatorStop() {
    allActuators.set(0);
  }

  public double getNeoPosA() {
    return actuatorNeoEncFrontLeft.getPosition();
  }

  public double getNeoPosC() {
    return actuatorNeoEncBackLeft.getPosition();
  }
}
