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

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Shooter implements ISubsystem {

    private CANSparkMax _hoodMotor;
    private DutyCycleEncoder _hoodEncoder;
    private TalonFX _leftMotor;
    private TalonFX _rightMotor;

    private Solenoid _leftCooler;
    private Solenoid _rightCooler;

    private PIDController _hoodPositionPID;


    private static Shooter _instance;
    private double _shooterSpeed;
    private double _hoodPosition;
    private String _currentShot;

    public Shooter() {
        _leftMotor = new TalonFX(Constants.kLeftShooterId);
        _rightMotor = new TalonFX(Constants.kRightShooterId);
        _hoodMotor = new CANSparkMax(Constants.kHoodId, MotorType.kBrushless);
        _hoodMotor.setInverted(true);
        _hoodEncoder = new DutyCycleEncoder(Constants.kHoodEncoderPortA);
        
        
        
        _leftCooler = new Solenoid(Constants.kLeftShooterChannel);
        _rightCooler = new Solenoid(Constants.kRightShooterChannel);

	    /** Config Objects for motor controllers */
	    TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
        TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
        
        _leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source
        _leftConfig.slot0.kP = Constants.kShootP;
        _leftConfig.slot0.kF = Constants.kShootF;

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorDeviceID = _leftMotor.getDeviceID(); //Device ID of Remote Source
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

        _leftMotor.setInverted(true);
        _rightMotor.follow(_leftMotor, FollowerType.PercentOutput);

        _leftMotor.configAllSettings(_leftConfig);
        _rightMotor.configAllSettings(_rightConfig);

        _hoodPositionPID = new PIDController(Constants.kHoodPositionkP, Constants.kHoodPositionkI, Constants.kHoodPositionkD);
        _hoodPositionPID.setTolerance(0.05);

        _shooterSpeed = 0;
        _hoodPosition = 0;

        _currentShot = "None";
        SmartDashboard.putNumber("manualShooterSpeed", 0);
        setLineShot();

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
    public void readyShot()
    {
        setSpeed(_shooterSpeed);
        setHoodPosition(_hoodPosition);
    }
    public boolean goShoot()
    {
        boolean shooterAtSpeed = false;
        if (_currentShot == "Far Shot") {
            shooterAtSpeed = (Math.abs(_leftMotor.getClosedLoopError()) < 1000);
        } else {
            shooterAtSpeed = (Math.abs(_leftMotor.getClosedLoopError()) < 500);
        }
        
        return shooterAtSpeed && _hoodPositionPID.atSetpoint();
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
        var output = _hoodPositionPID.calculate(_hoodEncoder.get(), position);
        if (Math.abs(output) >= 0.25)
        {
            if (output > 0)
            {
                output = .25;
            } else {
                output = -.25;
            }
        }
        SmartDashboard.putNumber("hoodOutput", output);

        _hoodMotor.set(output);
    }
    public void setLineShot()
    {
        _shooterSpeed = Constants.kLineShotVelocity;
        _hoodPosition = Constants.kLineShotHoodPosition;
        _currentShot = "Line Shot";
    }
    public void setFarShot()
    {
        _shooterSpeed = Constants.kFarShotVelocity;
        _hoodPosition = Constants.kFarShotHoodPosition;
        _currentShot = "Far Shot";
    }
    public void setPopShot()
    {
        _shooterSpeed = Constants.kPopShotVelocity;
        _hoodPosition = Constants.kPopShotHoodPosition;
        _currentShot = "Pop Shot";
    }
    public void setTrenchShot()
    {
        _shooterSpeed = Constants.kTrenchShotVelocity;
        _hoodPosition = Constants.kTrenchShotHoodPosition;
        _currentShot = "Trench Shot";
    }
    public void moveHood(double speed)
    {
        _hoodMotor.set(speed);
    }
    public void Shoot(double speed){
        _leftMotor.set(ControlMode.PercentOutput, speed);
    }
    public void ManualShoot(){
        _leftMotor.set(ControlMode.PercentOutput, _shooterSpeed);
    }
    public void stopHood(){
        _hoodMotor.set(0);
    }
    public void stop()
    {
        _leftMotor.set(ControlMode.PercentOutput, 0);
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
        SmartDashboard.putNumber("hoodPower", _hoodMotor.get());

        SmartDashboard.putNumber("hoodPosition", _hoodEncoder.get());
        SmartDashboard.putNumber("hoodOffset", _hoodEncoder.getPositionOffset());
        SmartDashboard.putNumber("hoodFrequency", _hoodEncoder.getFrequency());
        SmartDashboard.putString("currentShot", _currentShot);
        
        SmartDashboard.putNumber("shooterError", _leftMotor.getClosedLoopError());
        SmartDashboard.putBoolean("hoodPositionAtSetpoint", _hoodPositionPID.atSetpoint());
    }

    @Override
    public void ReadDashboardData() {
        //_shooterSpeed = SmartDashboard.getNumber("manualShooterSpeed", 0);
    }

    public boolean isCloseShot()
    {
        return _currentShot == "Pop Shot";
    }

}