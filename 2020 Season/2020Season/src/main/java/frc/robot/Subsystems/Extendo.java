package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Extendo implements ISubsystem {
    
    private static Extendo _instance;
    private CANSparkMax _leftMotor;
    private CANSparkMax _rightMotor;

    private Encoder _extendoEncoder;

    private double _extendoSpeed;

    public Extendo()
    {
        _leftMotor = new CANSparkMax(Constants.kLeftExtendoId, MotorType.kBrushless);
        _rightMotor = new CANSparkMax(Constants.kRightExtendoId, MotorType.kBrushless);
        _rightMotor.setInverted(true);

        _rightMotor.follow(_leftMotor);

        _extendoEncoder = new Encoder(Constants.kExtendoEncoderPortA, Constants.kExtendoEncoderPortB);
        _extendoSpeed = 0;

        SmartDashboard.putNumber("extendoSpeed", _extendoSpeed);
    }
    public static Extendo GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Extendo();
        }
        return _instance;
    }
    private void setSpeed(double speed)
    {
        _leftMotor.set(speed);
    }
    public void manualExtendo()
    {
        setSpeed(_extendoSpeed);
    }

    @Override
    public void LogTelemetry() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("extendoPosition", _extendoEncoder.getDistance());
    }

    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        _extendoSpeed = SmartDashboard.getNumber("extendoSpeed", 0);
    }
}