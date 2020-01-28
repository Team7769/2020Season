package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Drivetrain implements ISubsystem{

    private DifferentialDrive _robotDrive;

    private CANSparkMax _leftFrontMotor;
    private CANSparkMax _leftRearMotor;
    private CANSparkMax _rightFrontMotor;
    private CANSparkMax _rightRearMotor;


    private static Drivetrain _instance;

    public Drivetrain() {
        _leftFrontMotor = new CANSparkMax(Constants.kLeftFrontDriveId, MotorType.kBrushless);
        _leftRearMotor = new CANSparkMax(Constants.kLeftRearDriveId, MotorType.kBrushless);
        _rightFrontMotor = new CANSparkMax(Constants.kRightFrontDriveId, MotorType.kBrushless);
        _rightRearMotor = new CANSparkMax(Constants.kRightRearDriveId, MotorType.kBrushless);

        _leftFrontMotor.setOpenLoopRampRate(Constants.kDriveRampRateSeconds);
        _leftFrontMotor.setSmartCurrentLimit(Constants.kDriveSmartCurrentLimitAmps);
        _leftFrontMotor.setInverted(true);

        _rightFrontMotor.setOpenLoopRampRate(Constants.kDriveRampRateSeconds);
        _rightFrontMotor.setSmartCurrentLimit(Constants.kDriveSmartCurrentLimitAmps);
        _rightFrontMotor.setInverted(true);

        _leftRearMotor.follow(_leftFrontMotor);
        _rightRearMotor.follow(_rightFrontMotor);

        _robotDrive = new DifferentialDrive(_leftFrontMotor, _rightFrontMotor);
    }
    public static Drivetrain GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }
        return _instance;
    }
    public void FunnyDrive(double throttle, double turn){
        _robotDrive.arcadeDrive(throttle, -turn);
    }
    public void LogTelemetry()
    {
        SmartDashboard.putNumber("distance",  1 * .37);
    }

    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub

    }
}