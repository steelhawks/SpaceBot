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

    //RIGHT DRIVETRAIN MOTOR PORTS
    public static final int frontRightMPort = 0;
    public static final int midRightMPort = 1;
    public static final int rearRightMPort = 2;

    //LEFT DRIVETRAIN MOTOR PORTS
    public static final int frontLeftMPort = 3;
    public static final int midLeftMPort = 4;
    public static final int rearLeftMPort = 5;

    //CLIMBER MOTOR PORTS
    public static final int actuatorMPortA = 6;
    public static final int actuatorMPortB = 7;
    public static final int actuatorMPortC = 8;
    public static final int actuatorMPortD = 9;
    public static final int dropdownMPort = 10;
    
    //ARM MOTOR PORTS
    public static final int leftArmMPort = 11;
    public static final int rightArmMPort = 12;
    public static final int pivotMPort = 13;

    //ELEVATOR MOTOR PORTS
    public static final int elevatorMPortA = 14;
    //public static final int elevatorMPortB = 15; -- MAY OR MAY NOT BE NEEDED

    //DRIVETRAIN SOLENOID PORTS
    public static final int shiftSolPortOn = 0;
    public static final int shiftSolPortOff = 1;

    //ARM SOLENOID PORTS
    public static final int armSolPortOn = 2;
    public static final int armSolPortOff = 3;
}
