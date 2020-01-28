package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Utilities.PathFollower;

public class Drivetrain implements ISubsystem{

    private DifferentialDrive _robotDrive;
    private PathFollower _pathFollower;

    private CANSparkMax _leftFrontMotor;
    private CANSparkMax _leftRearMotor;
    private CANSparkMax _rightFrontMotor;
    private CANSparkMax _rightRearMotor;

    private Encoder _leftEncoder;
    private Encoder _rightEncoder;
    private AHRS _gyro;


    private static Drivetrain _instance;
    private Pose2d _currentPose;
    private DifferentialDriveOdometry _odometry;

    public Drivetrain() {
        _leftFrontMotor = new CANSparkMax(Constants.kLeftFrontDriveId, MotorType.kBrushless);
        _leftRearMotor = new CANSparkMax(Constants.kLeftRearDriveId, MotorType.kBrushless);
        _rightFrontMotor = new CANSparkMax(Constants.kRightFrontDriveId, MotorType.kBrushless);
        _rightRearMotor = new CANSparkMax(Constants.kRightRearDriveId, MotorType.kBrushless);

        _gyro = new AHRS(Port.kMXP);
        _leftEncoder = new Encoder(1, 2);
        _rightEncoder = new Encoder(3, 4);

        _leftFrontMotor.setOpenLoopRampRate(Constants.kDriveRampRateSeconds);
        _leftFrontMotor.setSmartCurrentLimit(Constants.kDriveSmartCurrentLimitAmps);
        _leftFrontMotor.setInverted(true);

        _rightFrontMotor.setOpenLoopRampRate(Constants.kDriveRampRateSeconds);
        _rightFrontMotor.setSmartCurrentLimit(Constants.kDriveSmartCurrentLimitAmps);
        _rightFrontMotor.setInverted(true);

        _leftRearMotor.follow(_leftFrontMotor);
        _rightRearMotor.follow(_rightFrontMotor);

        _robotDrive = new DifferentialDrive(_leftFrontMotor, _rightFrontMotor);
        _pathFollower = new PathFollower();
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
    
    public void SetPath()
    {
        _pathFollower.setPath();
        _currentPose = _pathFollower.getStartingPose();
        
        var angle = Rotation2d.fromDegrees(-_gyro.getAngle());
        if (_odometry == null) 
        {
            _odometry = new DifferentialDriveOdometry(angle, _currentPose);
        } else {
            _odometry.update(_gyro, leftDistanceMeters, rightDistanceMeters);
        }
    }
    public void followPath(double timestamp)
    {
        var pose = _odometry.update(_gyro, leftDistanceMeters, rightDistanceMeters);
        _pathFollower.getPathTarget(timestamp, pose);
    }

    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub

    }
}