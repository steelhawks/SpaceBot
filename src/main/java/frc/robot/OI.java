/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Arms.ArmIntakeButton;
import frc.robot.commands.Arms.ArmOuttakeButton;
import frc.robot.commands.Arms.ArmStop;
import frc.robot.commands.Arms.HatchPistonButton;
import frc.robot.commands.Arms.LockPistonButton;
import frc.robot.commands.Climber.ActuatorLevelThreeButton;
import frc.robot.commands.Climber.ActuatorLevelTwoButton;
import frc.robot.commands.Climber.ActuatorRetractBackButton;
import frc.robot.commands.Climber.ActuatorRetractFrontButton;
import frc.robot.commands.Climber.DropdownButton;
import frc.robot.commands.Climber.DropdownStop;
import frc.robot.commands.Drivetrain.ShiftGearButton;
import frc.robot.commands.Elevator.ElevatorDownButton;
import frc.robot.commands.Elevator.ElevatorStop;
import frc.robot.commands.Elevator.ElevatorUpButton;
import frc.robot.commands.Pivot.PivotDownButton;
import frc.robot.commands.Pivot.PivotStop;
import frc.robot.commands.Pivot.PivotUpButton;
import frc.robot.commands.Vision.AlignTape;

/** 
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  Constants constants = Constants.getInstance();

  public Joystick driveJS = new Joystick(constants.driveJSPort);
  public Joystick buttonBoard = new Joystick(constants.buttonBoardPort);
  public Gamepad gamepad = new Gamepad(constants.gamepadPort);

  public OI(){
    //DRIVER JOYSTICK BUTTONS
    Button shift = new JoystickButton(driveJS, constants.shiftB);
    shift.whenPressed(new ShiftGearButton());

    Button dropdown = new JoystickButton(driveJS, constants.dropdownB);
    dropdown.whenActive(new DropdownButton());
    dropdown.whenInactive(new DropdownStop());

    //DRIVER VISION ALIGN BUTTON
    Button alignTape = new JoystickButton(driveJS, constants.alignTape);
    alignTape.whenPressed(new AlignTape());

    //OPERATOR GAMEPAD BUTTONS
    Button hatchPiston = new JoystickButton(gamepad, constants.hatchPistonB);
    hatchPiston.whenPressed(new HatchPistonButton());

    Button armIntake = new JoystickButton(gamepad, constants.armIntakeB);
    armIntake.whenActive(new ArmIntakeButton());
    armIntake.whenInactive(new ArmStop());

    Button armOuttake = new JoystickButton(gamepad, constants.armOuttakeB);
    armOuttake.whenActive(new ArmOuttakeButton());
    armOuttake.whenInactive(new ArmStop());

    Button pivotUp = new JoystickButton(gamepad, constants.pivotUpB);
    pivotUp.whenActive(new PivotUpButton());
    pivotUp.whenInactive(new PivotStop());

    Button pivotDown = new JoystickButton(gamepad, constants.pivotDownB);
    pivotDown.whenActive(new PivotDownButton());
    pivotDown.whenInactive(new PivotStop());


    //OPERATOR BUTTON BOARD BUTTONS
    Button actuatorRetractFront = new JoystickButton(buttonBoard, constants.actuatorRetractFrontB);
    actuatorRetractFront.whenPressed(new ActuatorRetractFrontButton());
    
    Button actuatorRetractBack = new JoystickButton(buttonBoard, constants.actuatorRetractBackB);
    actuatorRetractBack.whenPressed(new ActuatorRetractBackButton());

    Button actuatorLevelTwo = new JoystickButton(buttonBoard, constants.actuatorLevelTwoB);
    actuatorLevelTwo.whenPressed(new ActuatorLevelTwoButton());

    Button actuatorLevelThree = new JoystickButton(buttonBoard, constants.actuatorLevelThreeB);
    actuatorLevelThree.whenPressed(new ActuatorLevelThreeButton());

    Button lockPiston = new JoystickButton(buttonBoard, constants.lockPistonB);
    lockPiston.whenPressed(new LockPistonButton());

    Button elevatorUp = new JoystickButton(buttonBoard, constants.elevatorUpB);
    elevatorUp.whenActive(new ElevatorUpButton());
    elevatorUp.whenInactive(new ElevatorStop());

    Button elevatorDown = new JoystickButton(buttonBoard, constants.elevatorDownB);
    elevatorDown.whenActive(new ElevatorDownButton());
    elevatorDown.whenInactive(new ElevatorStop());

    //Closing all command calls. Makes the code run faster (less resources present)

    shift.close();
    dropdown.close();
    alignTape.close();
    hatchPiston.close();
    armIntake.close();
    armOuttake.close();
    pivotUp.close();
    pivotDown.close();
    actuatorRetractFront.close();
    actuatorRetractBack.close();
    actuatorLevelTwo.close();
    actuatorLevelThree.close();
    lockPiston.close();
    elevatorUp.close();
    elevatorDown.close();
  }
}
