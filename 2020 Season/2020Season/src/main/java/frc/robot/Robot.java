/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ISubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SpinnyThingy;
import frc.robot.Utilities.LEDController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  private XboxController _driverController;
  private Drivetrain _drivetrain;
  private Shooter _shooter;
  private Collector _collector;
  private SpinnyThingy _spinnyThingy;
  private ArrayList<ISubsystem> _subsystems;

  private LEDController _ledController;

  private int _autonomousLoops;
  private int _autonomousCase;
  private int _aimLoops;
  private double _ledValue;

  @Override
  public void robotInit() {
    _driverController = new XboxController(Constants.kDriverUsbSlot);
    
    _drivetrain = Drivetrain.GetInstance();
    _ledController = LEDController.GetInstance();
    //_shooter = Shooter.GetInstance();
    //_collector = Collector.GetInstance();
    //_spinnyThingy = SpinnyThingy.GetInstance();

    _subsystems = new ArrayList<ISubsystem>();

    _subsystems.add(_drivetrain);
    //_subsystems.add(_shooter);
    //_subsystems.add(_collector);
    //_subsystems.add(_spinnyThingy);
    _autonomousCase = 0;
    _autonomousLoops = 0;
    _aimLoops = 0;
    _ledValue = -0.99;
    SmartDashboard.putNumber("ledValue", _ledValue);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    _subsystems.forEach(s -> s.LogTelemetry());
    _subsystems.forEach(s -> s.ReadDashboardData());

    _drivetrain.updatePose();
  }

  @Override
  public void disabledPeriodic() {
    if (_driverController.getBackButtonPressed())
    {
      autonomousInit();
      
    }
  }

  @Override
  public void autonomousInit() {
    //_drivetrain.setLineToTrenchPath();
    _drivetrain.setLineToLeftDiamondPath();
    _drivetrain.resetEncoders();
    _drivetrain.resetGyro();
    _drivetrain.updatePose();
    _drivetrain.resetPIDControllers();
    _autonomousLoops = 0;
    _autonomousCase = 0;
    _aimLoops = 0;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Timestamp", Timer.getMatchTime());
    diamondFirstTrenchAuto();
    //trenchAuto();
    //reverseTrenchTest(timestamp);
    //turnTestAuto();

    _autonomousLoops++;
  }
  public void diamondFirstTrenchAuto()
  {
    switch (_autonomousCase) {
      case 0:
        _drivetrain.setLineToLeftDiamondPath();
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _drivetrain.setLeftDiamondToTrenchPath();
          _autonomousLoops = 0;
          _autonomousCase++;
          //_autonomousCase = 7769;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops > 0) {
          _autonomousLoops = 0;
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 3:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _drivetrain.setAfterLeftDiamondToTrenchPath();
          _autonomousCase++;
        }
        break;
      case 4:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 5:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      case 6:
        if (turnToAngle(10.1))
        {
          _autonomousCase++;
        } else {
          _aimLoops = 0;
        }
        break;
      case 7:
        _drivetrain.FunnyDrive(0, 0);
        break;
    }
  }
  public void trenchAuto()
  {
    switch (_autonomousCase) {
      case 0:
        _drivetrain.setLineToTrenchPath();
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _drivetrain.setTrenchToLinePath();
          _autonomousLoops = 0;
          _autonomousCase++;
          //_autonomousCase = 7769;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        if (_autonomousLoops > 0) {
          _autonomousLoops = 0;
          _drivetrain.startPath();
          _autonomousCase++;
        }
        break;
      case 3:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _drivetrain.setLineToLeftDiamondPath();
          _autonomousCase++;
        }
        break;
      case 4:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.startPath();
        _autonomousCase++;

        break;
      case 5:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _drivetrain.setLeftDiamondToLinePath();
          _autonomousCase++;
        }
        break;
      case 6:
        _drivetrain.tankDriveVolts(0, 0);
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 7:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _autonomousCase++;
        }
        break;
      case 8:
        _drivetrain.FunnyDrive(0, 0);
        break;
      case 7769:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }
  public void reverseTrenchTest()
  {
      switch (_autonomousCase) {
      case 0:
        _drivetrain.setTrenchToLinePath();
        _drivetrain.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followPath();
        if (_drivetrain.isPathFinished())
        {
          _drivetrain.setTrenchToLinePath();
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }

  public void turnTestAuto()
  {
    switch (_autonomousCase)
    {
      case 0:
        if (turnToAngle(135))
        {
          _autonomousCase++;
        } else {
          _autonomousLoops = 0;
        }
        break;
      case 1:
        _drivetrain.tankDriveVolts(0, 0);
        break;
    }
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double throttle = -_driverController.getY(Hand.kLeft);
    double turn = _driverController.getX(Hand.kRight);

    _drivetrain.FunnyDrive(throttle, turn);

    if (_driverController.getBumper(Hand.kLeft)){
      _shooter.ManualShoot();
    }

    if (_driverController.getBackButtonPressed())
    {
      _ledValue -= 0.02;
      if (_ledValue <= -1)
      {
        _ledValue = -0.99;
      }
    } else if (_driverController.getStartButtonPressed()) {
      _ledValue += 0.02;
      if (_ledValue >= 1)
      {
        _ledValue = 0.99;
      }
    }
    SmartDashboard.putNumber("ledValue", _ledValue);
    _ledController.setLED(_ledValue);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public boolean turnToAngle(double angle)
  {
    _drivetrain.turnToAngle(angle);
    if (_drivetrain.isTurnFinished())
    {
      _aimLoops++;
    }
    return _drivetrain.isTurnFinished() && _aimLoops > 50;
  }
}
