package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.CrewLib.ProximitySensor;

public class Collector implements ISubsystem {

    private CANSparkMax _innerCollector;
    private CANSparkMax _outerCollector;
    private CANSparkMax _frontConveyor;
    private CANSparkMax _backConveyor;
    private DoubleSolenoid _collectorSolenoid;
    private Solenoid _ballStop;
    private ProximitySensor _intakeProximitySensor;
    private ProximitySensor _indexSensor;
    private ProximitySensor _outIndexSensor;

    private static Collector _instance;
    private double _innerCollectorSpeed;
    private double _outerCollectorSpeed;
    private double _conveyorSpeed;
    private boolean _indexing;
    private boolean _hasCounted;
    private boolean _hasLeft;
    private int _ballCount;

    public Collector() {
        
        _innerCollector = new CANSparkMax(Constants.kInnerCollectorId, MotorType.kBrushless);
        _outerCollector = new CANSparkMax(Constants.kOuterCollectorId, MotorType.kBrushless);
        _frontConveyor = new CANSparkMax(Constants.kFrontConveyorId, MotorType.kBrushless);
        _backConveyor = new CANSparkMax(Constants.kBackConveyorId, MotorType.kBrushless);
        _intakeProximitySensor = new ProximitySensor(Constants.kIntakeSensorPort);
        _indexSensor = new ProximitySensor(Constants.kIndexSensorPort);
        _outIndexSensor = new ProximitySensor(Constants.kOutIndexSensorPort);
        _collectorSolenoid = new DoubleSolenoid(Constants.kCollectorChannelForward, Constants.kCollectorChannelReverse);
        _ballStop = new Solenoid(Constants.kConveyorChannel);

        _backConveyor.follow(_frontConveyor, true);

        _innerCollectorSpeed = .8;
        _outerCollectorSpeed = .8;
        _conveyorSpeed = .3;
        _indexing = true;
        _ballCount = 0;
        _hasCounted = false;
        _hasLeft = false;

        SmartDashboard.putNumber("innerCollectorSpeed", _innerCollectorSpeed);
        SmartDashboard.putNumber("outerCollectorSpeed", _outerCollectorSpeed);
        SmartDashboard.putNumber("conveyorSpeed", _conveyorSpeed);

        setInitialBallCount(3);
    }
    public static Collector GetInstance(){
        if (_instance == null) 
        {
            _instance = new Collector();
        }
        return _instance;
    }

    public void ManualCollect(){
        _innerCollector.set(_innerCollectorSpeed);
        _outerCollector.set(-_outerCollectorSpeed);
    }
    public void spit()
    {
        _collectorSolenoid.set(Value.kReverse);
        _innerCollector.set(-_innerCollectorSpeed);
        _outerCollector.set(_outerCollectorSpeed);
    }
    public void succ()
    {
        _collectorSolenoid.set(Value.kReverse);
        _innerCollector.set(_innerCollectorSpeed);
        _outerCollector.set(-_outerCollectorSpeed);
    }
    public void stop()
    {
        //_frontConveyor.set(0);
        _innerCollector.set(0);
        _outerCollector.set(0);
    }
    public void retractCollector()
    {
        _collectorSolenoid.set(Value.kForward);
    }
    public void index()
    {
        _ballStop.set(true);
        if (_intakeProximitySensor.isBlocked())
        {
            _frontConveyor.set(_conveyorSpeed);
        } else if (_indexing) {
            _frontConveyor.set(0);
        }

        //if (_indexSensor.isBlocked() && !_hasCounted)
        //{
        //    _hasCounted = true;
        //    _ballCount++;
        //} else if (!_indexSensor.isBlocked())
        //{
        //    _hasCounted = false;
        //}
        
    }
    public void goUp()
    {
        _frontConveyor.set(_conveyorSpeed);
    }
    public void empty()
    {
        _frontConveyor.set(-_conveyorSpeed);
        _innerCollector.set(-_innerCollectorSpeed);
        _outerCollector.set(_outerCollectorSpeed);
    }
    public void feed()
    {
        _ballStop.set(false);
        _frontConveyor.set(_conveyorSpeed);
        _indexing = false;

        //if (_outIndexSensor.isBlocked() && !_hasLeft)
        //{
        //    _hasLeft = true;
        //    _ballCount--;
        //} else if (!_outIndexSensor.isBlocked())
        //{
        //    _hasLeft = false;
        //}
    }
    public void stopConveyor()
    {
        _ballStop.set(true);
        _frontConveyor.set(0);
    }
    public void stopFeed() {
        _ballStop.set(true);
        if (!_indexing)
        {
            _frontConveyor.set(0);
            _indexing = true;
        }
    }
    public void setInitialBallCount(int number)
    {
        _ballCount = number;
    }

    @Override
    public void LogTelemetry() {
        //Marvin code :)
        //SmartDashboard.putNumber("collectorRPM", _rightCollector.getOpenLoopRampRate());
        SmartDashboard.putNumber("intakeSensor", _intakeProximitySensor.getVoltage());
        SmartDashboard.putNumber("ballCount", _ballCount);
        SmartDashboard.putBoolean("ballStop", _ballStop.get());
        
        switch (_collectorSolenoid.get())
        {
            case kReverse:
                SmartDashboard.putString("collectorSolenoidState", "Reverse");
            break;
            case kForward:
                SmartDashboard.putString("collectorSolenoidState", "Forward");
            break;
            default:
                SmartDashboard.putString("collectorSolenoidState", "Off");
            break;

        }
    }

    @Override
    public void ReadDashboardData() {
        _innerCollectorSpeed = SmartDashboard.getNumber("innerCollectorSpeed", 0);
        _outerCollectorSpeed = SmartDashboard.getNumber("outerCollectorSpeed", 0);
    }

}