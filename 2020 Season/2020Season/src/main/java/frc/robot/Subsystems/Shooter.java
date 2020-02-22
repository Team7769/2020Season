package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Shooter implements ISubsystem {

    private CANSparkMax _hoodMotor;
    private Encoder _hoodEncoder;
    private TalonFX _leftMotor;
    private TalonFX _rightMotor;

    private Solenoid _leftCooler;
    private Solenoid _rightCooler;

    private PIDController _hoodPositionPID;

    private static Shooter _instance;
    private double _shooterSpeed;
    private double _hoodPosition;

    public Shooter() {
        _leftMotor = new TalonFX(Constants.kLeftShooterId);
        _rightMotor = new TalonFX(Constants.kRightShooterId);
        _hoodMotor = new CANSparkMax(Constants.kHoodId, MotorType.kBrushless);
        _hoodEncoder = new Encoder(Constants.kHoodEncoderPortA, Constants.kHoodEncoderPortB);
        _leftCooler = new Solenoid(Constants.kLeftShooterChannel);
        _rightCooler = new Solenoid(Constants.kRightShooterChannel);

	    /** Config Objects for motor controllers */
	    TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
        
        _leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorDeviceID = _leftMotor.getDeviceID(); //Device ID of Remote Source
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

        _leftMotor.setInverted(true);
        _rightMotor.follow(_leftMotor, FollowerType.PercentOutput);

        _leftMotor.configAllSettings(_leftConfig);
        _rightMotor.configAllSettings(_rightConfig);

        _hoodPositionPID = new PIDController(Constants.kHoodPositionkP, Constants.kHoodPositionkI, Constants.kHoodPositionkD);

        _shooterSpeed = 0;
        _hoodPosition = 0;
        SmartDashboard.putNumber("manualShooterSpeed", 0);

    }
    public static Shooter GetInstance() 
    {
        if (_instance == null)
        {
            _instance = new Shooter();
        }
        return _instance;
    }
    public void resetSensors()
    {
        _hoodEncoder.reset();
    }
    public void goShoot()
    {
        setSpeed(_shooterSpeed);
        setHoodPosition(_hoodPosition);
    }
    private void setSpeed(double speed)
    {
        _leftMotor.set(TalonFXControlMode.Velocity, speed);
    }
    private void setHoodPosition(double position)
    {
        if (position != _hoodPositionPID.getSetpoint())
        {
            _hoodPositionPID.reset();
        }
        var output = _hoodPositionPID.calculate(_hoodEncoder.getDistance(), position);
        _hoodMotor.set(output);
    }
    public void setLineShot()
    {
        _shooterSpeed = Constants.kLineShotVelocity;
        _hoodPosition = Constants.kLineShotHoodPosition;
    }
    public void setFarShot()
    {
        _shooterSpeed = Constants.kFarShotVelocity;
        _hoodPosition = Constants.kFarShotHoodPosition;
    }
    public void setPopShot()
    {
        _shooterSpeed = Constants.kPopShotVelocity;
        _hoodPosition = Constants.kPopShotHoodPosition;
    }
    public void setTrenchShot()
    {
        _shooterSpeed = Constants.kTrenchShotVelocity;
        _hoodPosition = Constants.kTrenchShotHoodPosition;
    }
    public void Shoot(double speed){
        //_leftMotor.set(speed);
        _leftMotor.set(ControlMode.PercentOutput, speed);
    }
    public void ManualShoot(){
        //_leftMotor.set(_shooterSpeed);
        _leftMotor.set(ControlMode.PercentOutput, _shooterSpeed);
    }
    public void monitorTemperature()
    {
        if (_leftMotor.getTemperature() >= Constants.kMotorTemperatureThreshold)
        {
            _leftCooler.set(true);
        } else {
            _leftCooler.set(false);
        }
        if (_rightMotor.getTemperature() >= Constants.kMotorTemperatureThreshold)
        {
            _rightCooler.set(true);
        } else {
            _rightCooler.set(false);
        }
    }

    @Override
    public void LogTelemetry() {
        SmartDashboard.putNumber("shooterRPM", _leftMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooterInputCurrent", _leftMotor.getSupplyCurrent());
        SmartDashboard.putNumber("shooterOutputCurrent", _leftMotor.getStatorCurrent());

        SmartDashboard.putNumber("leftShooterTemperature", _leftMotor.getTemperature());
        SmartDashboard.putNumber("rightShooterTemperature", _rightMotor.getTemperature());
        SmartDashboard.putBoolean("leftCoolerEngaged", _leftCooler.get());
        SmartDashboard.putBoolean("rightCoolerEngaged", _rightCooler.get());

        SmartDashboard.putNumber("hoodPosition", _hoodEncoder.getDistance());
    }

    @Override
    public void ReadDashboardData() {
        _shooterSpeed = SmartDashboard.getNumber("manualShooterSpeed", 0);
    }

}