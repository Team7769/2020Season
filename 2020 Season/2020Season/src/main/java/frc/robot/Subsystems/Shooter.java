package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Configuration.Constants;

public class Shooter implements ISubsystem {

    private CANSparkMax _leftMotor;
    private CANSparkMax _rightMotor;

    private static Shooter _instance;

    public Shooter() {
        _leftMotor = new CANSparkMax(Constants.kLeftShooterId, MotorType.kBrushless);
        _rightMotor = new CANSparkMax(Constants.kRightShooterId, MotorType.kBrushless);

    }
    public static Shooter GetInstance() 
    {
        if (_instance == null)
        {
            _instance = new Shooter();
        }
        return _instance;
    }

    @Override
    public void LogTelemetry() {
        // TODO Auto-generated method stub

    }

}