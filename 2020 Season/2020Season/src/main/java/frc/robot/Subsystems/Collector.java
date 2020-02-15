package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Collector implements ISubsystem {

    private CANSparkMax _leftCollector;
    private CANSparkMax _rightCollector;
    private AnalogInput _intakeProximitySensor;

    private static Collector _instance;
    private double _collectorSpeed;

    public Collector() {
        ///_leftCollector = new CANSparkMax(Constants.kLeftCollectorId, MotorType.kBrushless);
        //_rightCollector = new CANSparkMax(Constants.kRightCollectorId, MotorType.kBrushless);

        //_leftCollector.follow(_rightCollector);

        _intakeProximitySensor = new AnalogInput(Constants.kIntakeSensorPort);

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
        _rightCollector.set(_collectorSpeed);
    }
    public void spit()
    {
        _rightCollector.set(-_collectorSpeed);
    }
    public void succ()
    {
        _rightCollector.set(_collectorSpeed);
    }
    public void stop()
    {
        _rightCollector.set(0);
    }
    

    @Override
    public void LogTelemetry() {
        //Marvin code :)
        //SmartDashboard.putNumber("collectorRPM", _rightCollector.getOpenLoopRampRate());
        SmartDashboard.putNumber("intakeSensor", _intakeProximitySensor.getVoltage());
    }

    @Override
    public void ReadDashboardData() {
        _collectorSpeed = SmartDashboard.getNumber("manualCollectorSpeed", 0);

    }

}