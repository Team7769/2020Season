package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Extendo implements ISubsystem {
    
    private static Extendo _instance;
    private CANSparkMax _leftMotor;
    private CANSparkMax _rightMotor;

    private Encoder _extendoEncoder;

    private double _extendoSpeed;

    private DoubleSolenoid _extendoSolenoid;

    public Extendo()
    {
        _leftMotor = new CANSparkMax(Constants.kLeftExtendoId, MotorType.kBrushless);
        _rightMotor = new CANSparkMax(Constants.kRightExtendoId, MotorType.kBrushless);
        _rightMotor.setInverted(true);

        _rightMotor.follow(_leftMotor);

        _extendoEncoder = new Encoder(Constants.kExtendoEncoderPortA, Constants.kExtendoEncoderPortB);
        _extendoSpeed = .5;

        _extendoSolenoid = new DoubleSolenoid(Constants.kExtendoLockChannelForward, Constants.kExtendoLockChannelReverse );
       
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
    public void extend() 
    {
        setSpeed(1.0);
    }
    public void unextend()
    {
        setSpeed(-_extendoSpeed);
    }
    public void stop()
    {
        setSpeed(0);
    }
    public void extendoLock()
    {
        _extendoSolenoid.set(Value.kReverse);
    }
    public void extendoRelease()
    {
        _extendoSolenoid.set(Value.kForward);
    }
    
    @Override
    public void LogTelemetry() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("extendoPosition", _extendoEncoder.getDistance());
        switch (_extendoSolenoid.get())
        {
            case kForward:
              SmartDashboard.putString("extendoSolenoidState", "Forward");
            break;
            case kReverse:
            SmartDashboard.putString("extendoSolenoidState", "Reverse");
            break;
            default:
            SmartDashboard.putString("extendoSolenoidState","Off");
            break;

        }
    }

    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        _extendoSpeed = SmartDashboard.getNumber("extendoSpeed", 0);
    }
}