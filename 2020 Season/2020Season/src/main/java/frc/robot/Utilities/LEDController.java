package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Spark;

public class LEDController {
    private static Spark _blinkin;
    public static LEDController _instance;

    public final double kTrackingTarget = -0.31;
    public final double kOnTarget = -0.23;
    
    public LEDController()
    {
        _blinkin = new Spark(0);
    }
    public static LEDController GetInstance()
    {
        if (_instance == null)
        {
            _instance = new LEDController();
        }
        return _instance;
    }
    public void setLED(double value)
    {
        _blinkin.set(value);
    }
    public void setTrackingTargetState()
    {
        _blinkin.set(kTrackingTarget);
    }
    public void setOnTargetState()
    {
        _blinkin.set(kOnTarget);
    }
}