package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Configuration.Constants;
import frc.robot.Utilities.Limelight;
import frc.robot.Utilities.PathFollower;

public class Drivetrain implements ISubsystem{

    private DifferentialDrive _robotDrive;
    private PathFollower _pathFollower;
    private Limelight _limelight;

    private CANSparkMax _leftFrontMotor;
    private CANSparkMax _leftRearMotor;
    private CANSparkMax _rightFrontMotor;
    private CANSparkMax _rightRearMotor;

    private Encoder _leftEncoder;
    private Encoder _rightEncoder;
    private AHRS _gyro;

    private PIDController _leftDriveVelocityPID;
    private PIDController _rightDriveVelocityPID;
    private PIDController _turnPID;


    private static Drivetrain _instance;
    private DifferentialDriveOdometry _odometry;
    private SimpleMotorFeedforward _feedForward;

    public Drivetrain() {
        _leftFrontMotor = new CANSparkMax(Constants.kLeftFrontDriveId, MotorType.kBrushless);
        _leftRearMotor = new CANSparkMax(Constants.kLeftRearDriveId, MotorType.kBrushless);
        _rightFrontMotor = new CANSparkMax(Constants.kRightFrontDriveId, MotorType.kBrushless);
        _rightRearMotor = new CANSparkMax(Constants.kRightRearDriveId, MotorType.kBrushless);

        _gyro = new AHRS(Port.kMXP);
        _leftEncoder = new Encoder(Constants.kLeftEncoderPortA, Constants.kLeftEncoderPortB);
        _leftEncoder.setDistancePerPulse(Constants.kDriveDistancePerPulse);

        _rightEncoder = new Encoder(Constants.kRightEncoderPortA, Constants.kRightEncoderPortB, true);
        _rightEncoder.setDistancePerPulse(Constants.kDriveDistancePerPulse);

        _leftRearMotor.follow(_leftFrontMotor);
        _rightRearMotor.follow(_rightFrontMotor);

        _robotDrive = new DifferentialDrive(_leftFrontMotor, _rightFrontMotor);
        _pathFollower = new PathFollower();

        _odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        _leftDriveVelocityPID = new PIDController(Constants.kPathFollowingkP, 0.0, 0.0);
        _rightDriveVelocityPID = new PIDController(Constants.kPathFollowingkP, 0.0, 0.0);
        _turnPID = new PIDController(Constants.kTurnkP, Constants.kTurnkI, Constants.kTurnkD);
        _turnPID.setTolerance(0.5);

        _feedForward = new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter);

        
        SmartDashboard.putNumber("turnP", _turnPID.getP());
        SmartDashboard.putNumber("turnI", _turnPID.getI());
        SmartDashboard.putNumber("turnD", _turnPID.getD());

        _limelight = Limelight.GetInstance();
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
        _rightFrontMotor.setIdleMode(IdleMode.kCoast);
        _leftFrontMotor.setIdleMode(IdleMode.kCoast);
        _robotDrive.arcadeDrive(throttle, turn);
    }
    public void LogTelemetry()
    {
        SmartDashboard.putNumber("leftDriveDistance", _leftEncoder.getDistance());
        SmartDashboard.putNumber("leftDriveVelocity", _leftEncoder.getRate());
        SmartDashboard.putNumber("rightDriveDistance", _rightEncoder.getDistance());
        SmartDashboard.putNumber("rightDriveVelocity", _rightEncoder.getRate());
        SmartDashboard.putNumber("gyroAngle", getHeading());

        
    }
     /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }
  public void updatePose()
  {
      _odometry.update(Rotation2d.fromDegrees(getHeading()), _leftEncoder.getDistance(), _rightEncoder.getDistance());
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_leftEncoder.getRate(), _rightEncoder.getRate());
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
/**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _rightFrontMotor.setIdleMode(IdleMode.kBrake);
    _leftFrontMotor.setIdleMode(IdleMode.kBrake);
    _leftFrontMotor.setVoltage(leftVolts);
    _rightFrontMotor.setVoltage(-rightVolts);
    _robotDrive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    _leftEncoder.reset();
    _rightEncoder.reset();
  }
  public void resetGyro() {
    _gyro.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (_leftEncoder.getDistance() + _rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return _leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return _rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    _robotDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    _gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(_gyro.getAngle(), 360) * (-1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return _gyro.getRate() * (1.0);
  }
    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub
    }
    public void setLineToTrenchPath()
    {
        _pathFollower.setLineToTrenchPath(getTrajectoryConfig(false));
    }
    public TrajectoryConfig getTrajectoryConfig(boolean isReverse)
    {
      var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);
        TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
            config.setReversed(isReverse);
            return config;
    }
    public void setTrenchToLinePath()
    {
        _pathFollower.setTrenchToLinePath(getTrajectoryConfig(true));
    }
    public void setLineToLeftDiamondPath()
    {
      _pathFollower.setLineToLeftDiamondPath(getTrajectoryConfig(false));
    }
    public void setLeftDiamondToLinePath()
    {
      _pathFollower.setLeftDiamondToLinePath(getTrajectoryConfig(true));
    }
    public void setLeftDiamondToTrenchPath()
    {
      _pathFollower.setLeftDiamondToTrenchPath(getTrajectoryConfig(true));
    }
    public void setAfterLeftDiamondToTrenchPath()
    {
      _pathFollower.setAfterLeftDiamondToTrenchPath(getTrajectoryConfig(false));
    }
    public void setLineToStealPath()
    {
      _pathFollower.setLineToStealPath(getTrajectoryConfig(false));
    }
    public void setStealToGoalPath()
    {
      _pathFollower.setStealToGoalPath(getTrajectoryConfig(true));
    }

    public void startPath()
    {
        _leftDriveVelocityPID.reset();
        _rightDriveVelocityPID.reset();
        _pathFollower.startPath();
    }
    public void followPath()
    {
        var target = _pathFollower.getPathTarget(getPose());

        SmartDashboard.putNumber("leftSpeed", getWheelSpeeds().leftMetersPerSecond);
        SmartDashboard.putNumber("rightSpeed", getWheelSpeeds().rightMetersPerSecond);
        SmartDashboard.putNumber("leftTargetSpeed", target.leftMetersPerSecond);
        SmartDashboard.putNumber("rightTargetSpeed", target.rightMetersPerSecond);

        var leftOutputTarget = _leftDriveVelocityPID.calculate(getWheelSpeeds().leftMetersPerSecond, target.leftMetersPerSecond);
        var rightOutputTarget = _rightDriveVelocityPID.calculate(getWheelSpeeds().rightMetersPerSecond, target.rightMetersPerSecond);

        var leftFeedForward = _feedForward.calculate(target.leftMetersPerSecond);
        var rightFeedForward = _feedForward.calculate(target.rightMetersPerSecond);

        tankDriveVolts(leftOutputTarget + leftFeedForward, rightOutputTarget + rightFeedForward);

    }
    public boolean isPathFinished()
    {
      return _pathFollower.isPathFinished();
    }
    public void turnToAngle(double angle)
    {
      _turnPID.setP(SmartDashboard.getNumber("turnP", 0));
      _turnPID.setI(SmartDashboard.getNumber("turnI", 0));
      _turnPID.setD(SmartDashboard.getNumber("turnD", 0));
      var output = _turnPID.calculate(getHeading(), angle);
      SmartDashboard.putNumber("turnOutput", output);
      if (Math.abs(output) > 0.5)
      {
        if (output > 0)
        {
          output = 0.5;
        } else {
          output = -0.5;
        }
      }
      arcadeDriveBrake(0, -output);
    }
    public boolean isTurnFinished()
    {
      return _turnPID.atSetpoint();
    }
    public void arcadeDriveBrake(double throttle, double turn)
    {
        _rightFrontMotor.setIdleMode(IdleMode.kBrake);
        _leftFrontMotor.setIdleMode(IdleMode.kBrake);
        _robotDrive.arcadeDrive(throttle, turn);
    }
    public void resetPIDControllers()
    {
      _turnPID.reset();
    }
    public boolean trackTarget()
    {
      if (!_limelight.hasTarget())
      {
        return false;
      }
      var limelightTargetAngle = _limelight.getAngleToTarget();

      var targetAngle = getHeading() - limelightTargetAngle;

      turnToAngle(targetAngle);

      return isTurnFinished();

    }
    public double followTarget()
    {
      if (!_limelight.hasTarget())
      {
        return 0;
      }
      var limelightTargetAngle = _limelight.getAngleToTarget();

      var targetAngle = getHeading() - limelightTargetAngle;

      
      _turnPID.setP(SmartDashboard.getNumber("turnP", 0));
      _turnPID.setI(SmartDashboard.getNumber("turnI", 0));
      _turnPID.setD(SmartDashboard.getNumber("turnD", 0));
      var output = _turnPID.calculate(getHeading(), targetAngle);
      SmartDashboard.putNumber("turnOutput", output);
      if (Math.abs(output) > 0.5)
      {
        if (output > 0)
        {
          output = 0.5;
        } else {
          output = -0.5;
        }
      }
      return output;
    }
}