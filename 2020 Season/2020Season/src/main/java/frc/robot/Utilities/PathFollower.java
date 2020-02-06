package frc.robot.Utilities;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Configuration.Constants;

public class PathFollower {
    private RamseteController _controller;
    private Trajectory _currentPath;
    private Timer _timer;

    public PathFollower()
    {
        _controller = new RamseteController();
        _timer = new Timer();
    }
    public void startPath()
    {
        _timer.reset();
        _timer.start();
    }
    public boolean isPathFinished()
    {
        return _timer.get() > _currentPath.getTotalTimeSeconds();
    }
    public void setLineToTrenchPath(TrajectoryConfig config)
    {
        _currentPath = getLineToTrenchTrajectory(config);
    }
    public void setTrenchToLinePath(TrajectoryConfig config)
    {
        _currentPath = getTrenchToLineTrajectory(config);
    }
    public void setLineToLeftDiamondPath(TrajectoryConfig config)
    {
        _currentPath = getLineToLeftDiamondTrajectory(config);
    }
    public void setLeftDiamondToLinePath(TrajectoryConfig config)
    {
        _currentPath = getLeftDiamondToLineTrajectory(config);
    }
    public Trajectory getCurrentTrajectory()
    {
        return _currentPath;
    }
    public Pose2d getStartingPose()
    {
        return _currentPath.getInitialPose();
    }
    public DifferentialDriveWheelSpeeds getPathTarget(Pose2d currentPose)
    {
        var time = _timer.get();
        var goal = _currentPath.sample(time);
        var targetSpeeds = _controller.calculate(currentPose, goal);
        SmartDashboard.putNumber("goalDegrees", goal.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("totalPathTime", _currentPath.getTotalTimeSeconds());
        SmartDashboard.putNumber("goal", goal.poseMeters.getTranslation().getX());

        return Constants.kDriveKinematics.toWheelSpeeds(targetSpeeds);
    }
    /*private Trajectory generateTrajectory(TrajectoryConfig config) {

        var start = new Pose2d(3.129, 2.402, Rotation2d.fromDegrees(0));
        var end = new Pose2d(8.106, .75, Rotation2d.fromDegrees(0));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(5.708, .75));
    
        var trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
        return trajectory;
      }*/
      private Trajectory generateTestTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
      }
      private Trajectory getLineToTrenchTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2.6, 1.50)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5.1, 1.50, new Rotation2d(0)),
        // Pass config
        config);
      }
      private Trajectory getTrenchToLineTrajectory(TrajectoryConfig config)
      {
        return TrajectoryGenerator.generateTrajectory(new Pose2d(5.1, 1.50, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2.6, 1.50)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass config
        config);
      }
      private Trajectory getLineToLeftDiamondTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1.84, 0.8)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3.05, -0.307, Rotation2d.fromDegrees(-45)),
        // Pass config
        config);
      }
      private Trajectory getLeftDiamondToLineTrajectory(TrajectoryConfig config)
      {
        return TrajectoryGenerator.generateTrajectory(new Pose2d(3.05, -0.307, Rotation2d.fromDegrees(-45)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1.84, 0.8)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass config
        config);
      }
    }