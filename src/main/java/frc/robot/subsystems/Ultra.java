package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;

public class Ultra
{
    private static Ultrasonic ultra = new Ultrasonic(8, 9);
    public static void enable()
    {
        ultra.setEnabled(true);
        ultra.setAutomaticMode(true);
    }

    public static void disable()
    {
        ultra.setEnabled(false);
    }

    public static double getRange()
    {
        System.out.println(ultra.getRangeInches());
        return ultra.getRangeInches();
    }

    public static double getRangeMM()
    {
        return ultra.getRangeMM();
    }

    public static boolean isCloseShift()
    {
        if(ultra.getRangeInches() <= 70)
        {
            return true;
        }
        return false;
    }

    public static boolean isClose()
    {
        if(ultra.getRangeInches() <= 40)
        {
            return true;
        }
        return false;
    }

    public static boolean isRangeValid()
    {
        return ultra.isRangeValid();
    }
}