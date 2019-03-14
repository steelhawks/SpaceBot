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
import frc.robot.commands.Arms.ArmPistonButton;
import frc.robot.commands.Arms.ArmStop;
import frc.robot.commands.Arms.HatchPistonButton;
import frc.robot.commands.Climber.DropdownButton;
import frc.robot.commands.Climber.DropdownStop;
import frc.robot.commands.Climber.FrontRetract;
import frc.robot.commands.Drivetrain.ShiftGearButton;
import frc.robot.commands.Elevator.ElevatorDownButton;
import frc.robot.commands.Elevator.ElevatorUpButton;
import frc.robot.commands.Pivot.PivotDownButton;
import frc.robot.commands.Pivot.PivotStop;
import frc.robot.commands.Pivot.PivotUpButton;

/** 
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  Constants constants = Constants.getInstance();

  public Joystick driveJS = new Joystick(constants.driveJSPort);
  public Gamepad gamepad = new Gamepad(constants.gamepadPort);

  public OI(){
    //DRIVER JOYSTICK BUTTONS
    Button shift = new JoystickButton(driveJS, constants.shiftB);
    shift.whenPressed(new ShiftGearButton());

    Button dropdown = new JoystickButton(driveJS, constants.dropdownB);
    dropdown.whenActive(new DropdownButton());
    dropdown.whenInactive(new DropdownStop());

    //OPERATOR BUTTON BOARD BUTTONS
    Button armPiston = new JoystickButton(gamepad, constants.armPistonB);
    armPiston.whenPressed(new ArmPistonButton());

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

    Button elevatorUp = new JoystickButton(gamepad, constants.elevatorUpB);
    elevatorUp.whenPressed(new ElevatorUpButton());

    Button elevatorDown = new JoystickButton(gamepad, constants.elevatorDownB);
    elevatorDown.whenPressed(new ElevatorDownButton());

    /*Button frontActuatorRetract = new JoystickButton(gamepad, constants.frontActuatorRetractB);
    frontActuatorRetract.whenPressed(new FrontRetract());
    
    Button rearActuatorRetract = new JoystickButton(gamepad, constants.rearActuatorRetractB);
    rearRetractButton.whenPressed(new StopClimber());
    */
  }
}
