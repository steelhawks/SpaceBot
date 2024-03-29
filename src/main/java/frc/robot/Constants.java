/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Constants {
  
  private static Constants instance = null;

    public static Constants getInstance(){
      if (instance == null){
        instance = new Constants();
      }
      return instance;
    }

    //ROBOT POSITION 
    public static final int autonPos = 1;

    //AUTONOMOUS BOOLEANS
    public static boolean autoArm;

    //JOYSTICK PORTS
    public static final int driveJSPort = 0;
    public static final int buttonBoardPort = 1;
    public static final int gamepadPort = 2;
    
    //RIGHT DRIVETRAIN MOTOR PORTS - SPARK
    public static final int frontRightMPort = 20;
    public static final int midRightMPort = 1;
    public static final int rearRightMPort = 2;

    //LEFT DRIVETRAIN MOTOR PORTS - SPARK
    public static final int frontLeftMPort = 3;
    public static final int midLeftMPort = 4;
    public static final int rearLeftMPort = 5;

    //ELEVATOR MOTOR PORT - TALON
    public static final int elevatorMPort = 6;

    //CLIMBER MOTOR PORTS - FOUR SPARK, ONE TALON
    public static final int actuatorMPortA = 7;
    public static final int actuatorMPortB = 8;
    public static final int actuatorMPortC = 9;
    public static final int actuatorMPortD = 10;
    public static final int dropdownMPort = 11;

    //ARM MOTOR PORTS - TALON
    public static final int leftArmMPort = 12;
    public static final int rightArmMPort = 13;
    public static final int pivotMPort = 14;
    
    //DRIVETRAIN SOLENOID PORTS
    public static final int shiftSolPortOn = 0;
    public static final int shiftSolPortOff = 1;

    //ARM SOLENOID PORTS
    public static final int lockSolPortOn = 2;
    public static final int lockSolPortOff = 3;
    public static final int hatchSolPortOn = 4;
    public static final int hatchSolPortOff = 5;

    //DRIVER JOYSTICK BUTTONS
    public static final int shiftB = 1;
    public static final int dropdownB = 11;
    public static final int alignTape = 3;
    
    //BUTTON BOARD BUTTONS
    public static final int actuatorLevelTwoB = 5;
    public static final int actuatorLevelThreeB = 6;
    public static final int actuatorRetractBackB = 7;
    public static final int actuatorRetractFrontB = 8;
    // public static final int pivotDownB = 9;
    // public static final int pivotUpB = 10;
    // public static final int armIntakeB = 11;
    // public static final int armOuttakeB = 12;
    public static final int elevatorDownB = 1;
    public static final int elevatorUpB = 2;
  

    //OPERATOR GAMEPAD BUTTONS
    public static final int pivotDownB = 1;
    public static final int pivotUpB = 2;
    public static final int lockPistonB = 4;
    public static final int hatchPistonB = 3;
    public static final int armOuttakeB =  5;
    public static final int armIntakeB = 6;
    
    //OPTICAL ENCODER PORTS
    public static final int leftEncPortA = 0;
    public static final int leftEncPortB = 1; 
    public static final int rightEncPortA = 2;
    public static final int rightEncPortB = 3;

    //IR PORT
    public static final int cargoIRPort = 1;

    //LIMIT SWITCH PORTS
    public static final int topLimitPort = 4;
    public static final int bottomLimitPort = 5;
    public static final int pivotLimitPort = 6;
    
    //PATHWEAVER CONSTANTS
    public static final int ticksPerRev = 256;
    public static final double wheelDiameter = 6.0 / 12.0;
    public static final double maxVel = 14.42;
}
