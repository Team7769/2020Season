package frc.robot.Subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Collector implements ISubsystem {

    
    private static Collector _instance;

    public Collector() {
    }
    public static Collector GetInstance(){
        if (_instance == null) 
        {
            _instance = new Collector();
        }
        return _instance;
    }
    

    @Override
    public void LogTelemetry() {
    }

    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub

    }

}