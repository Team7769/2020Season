/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ISubsystem;
import frc.robot.Subsystems.Shooter;

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
  private ArrayList<ISubsystem> _subsystems;

  @Override
  public void robotInit() {
    _driverController = new XboxController(Constants.kDriverUsbSlot);
    
    _drivetrain = Drivetrain.GetInstance();
    _shooter = Shooter.GetInstance();

    _subsystems = new ArrayList<ISubsystem>();

    _subsystems.add(_drivetrain);
    _subsystems.add(_shooter);
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
  }

  @Override
  public void autonomousInit() {
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    double throttle = _driverController.getY(Hand.kLeft);
    double turn = _driverController.getX(Hand.kRight);

    _drivetrain.FunnyDrive(throttle, turn);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
