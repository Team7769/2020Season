package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Shooter implements ISubsystem {

    //private CANSparkMax _leftMotor;
    //private CANSparkMax _rightMotor;
    //private CANEncoder _shooterEncoder;

    private TalonFX _leftMotor;
    private TalonFX _rightMotor;

    private static Shooter _instance;
    private double _shooterSpeed;

    public Shooter() {
        _leftMotor = new TalonFX(Constants.kLeftShooterId);
        _rightMotor = new TalonFX(Constants.kRightShooterId);
        //_leftMotor = new CANSparkMax(Constants.kLeftShooterId, MotorType.kBrushless);
        //_rightMotor = new CANSparkMax(Constants.kRightShooterId, MotorType.kBrushless);

        //_shooterEncoder = _leftMotor.getEncoder();

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

        _shooterSpeed = 0;
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
    public void Shoot(double speed){
        //_leftMotor.set(speed);
        _leftMotor.set(ControlMode.PercentOutput, speed);
    }
    public void ManualShoot(){
        //_leftMotor.set(_shooterSpeed);
        _leftMotor.set(ControlMode.PercentOutput, _shooterSpeed);
    }

    @Override
    public void LogTelemetry() {
        SmartDashboard.putNumber("shooterRPM", _leftMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooterInputCurrent", _leftMotor.getSupplyCurrent());
        SmartDashboard.putNumber("shooterOutputCurrent", _leftMotor.getStatorCurrent());
    }

    @Override
    public void ReadDashboardData() {
        _shooterSpeed = SmartDashboard.getNumber("manualShooterSpeed", 0);
    }

}