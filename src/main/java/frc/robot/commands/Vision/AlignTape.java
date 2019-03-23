package frc.robot.commands.Vision;

import frc.robot.Robot;
import frc.robot.subsystems.Ultra;
import frc.robot.subsystems.Tape;

import edu.wpi.first.wpilibj.command.Command;

public class AlignTape extends Command
{    
    Tape TAPE = new Tape();

    public AlignTape() {}

    //Sets the gyro and angle at zero, both the right and left coordinates to default values, and ultrasound is enabled
    public void initialize()
    {
        TAPE.resetGyro();
        TAPE.setAngle(TAPE.getNTAngle());
        TAPE.setXPosLeftLimit(157.5);
        TAPE.setXPosRightLimit(162.5);
        Ultra.enable();
    }

    //Executes the vision code
    public void execute()
    {
        TAPE.align();
    }

    //Returns whether the vision code has finished
    public boolean isFinished()
    {
        return TAPE.isAligned();
    }

    //Terminates the vision code, motors are turned off, and the ultrasound is disabled
    public void end()
    {
        Robot.drivetrain.leftGroup.set(0);
        Robot.drivetrain.rightGroup.set(0);
        Ultra.disable();
    }

    //Suspends robot activity, motors are turned off, and ultrasound is disabled
    public void interupted()
    {
        Robot.drivetrain.leftGroup.set(0);
        Robot.drivetrain.rightGroup.set(0);
        Ultra.disable();
    }
}


