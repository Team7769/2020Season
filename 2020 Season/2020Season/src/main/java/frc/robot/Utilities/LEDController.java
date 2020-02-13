package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Spark;

public class LEDController {
    private static Spark _blinkin;
    public static LEDController _instance;
    
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
}