package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Collector implements ISubsystem {

    private CANSparkMax _LeftCollector;
    private CANSparkMax _RightCollector;

    private static Collector _instance;
    private double _collectorSpeed;

    public Collector() {
        _LeftCollector = new CANSparkMax(Constants.kLeftCollectorId, MotorType.kBrushless);
        _RightCollector = new CANSparkMax(Constants.kRightCollectorId, MotorType.kBrushless);

        _LeftCollector.follow(_RightCollector);

        _collectorSpeed = 0;
        SmartDashboard.putNumber("manualCollectorSpeed", _collectorSpeed);
    }
    public static Collector GetInstance(){
        if (_instance == null) 
        {
            _instance = new Collector();
        }
        return _instance;
    }

    public void ManualCollect(){
        //_leftMotor.set(_shooterSpeed);
        _RightCollector.set(_collectorSpeed);
    }
    

    @Override
    public void LogTelemetry() {
        SmartDashboard.putNumber("collectorRPM", _RightCollector.getOpenLoopRampRate());
    }

    @Override
    public void ReadDashboardData() {
        _collectorSpeed = SmartDashboard.getNumber("manualShooterSpeed", 0);

    }

}