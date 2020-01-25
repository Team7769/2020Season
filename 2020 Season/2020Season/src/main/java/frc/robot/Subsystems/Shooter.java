package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Shooter implements ISubsystem {

    private CANSparkMax _leftMotor;
    private CANSparkMax _rightMotor;
    private CANEncoder _shooterEncoder;

    private static Shooter _instance;
    private double _shooterSpeed;

    public Shooter() {
        _leftMotor = new CANSparkMax(Constants.kLeftShooterId, MotorType.kBrushless);
        _rightMotor = new CANSparkMax(Constants.kRightShooterId, MotorType.kBrushless);

        _shooterEncoder = _leftMotor.getEncoder();

        _leftMotor.setInverted(true);
        _rightMotor.follow(_leftMotor, true);

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
        _leftMotor.set(speed);
    }
    public void ManualShoot(){
        _leftMotor.set(_shooterSpeed);
    }

    @Override
    public void LogTelemetry() {
        SmartDashboard.putNumber("shooterRPM", _shooterEncoder.getVelocity());
        SmartDashboard.putNumber("shooterCurrent", _leftMotor.getOutputCurrent());
    }

    @Override
    public void ReadDashboardData() {
        _shooterSpeed = SmartDashboard.getNumber("manualShooterSpeed", 0);
    }

}