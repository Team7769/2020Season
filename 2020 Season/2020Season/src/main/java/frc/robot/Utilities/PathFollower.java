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
    public void setLeftDiamondToTrenchPath(TrajectoryConfig config)
    {
        _currentPath = getLeftDiamondToTrenchTrajectory(config);
    }
    public void setAfterLeftDiamondToTrenchPath(TrajectoryConfig config)
    {
        _currentPath = getAfterLeftDiamondToTrenchTrajectory(config);
    }
    public void setLineToStealPath(TrajectoryConfig config)
    {
        _currentPath = getLineToStealTrajectory(config);
    }
    public void setStealToGoalPath(TrajectoryConfig config)
    {
        _currentPath = getStealToGoalTrajectory(config);
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

        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kCenterGoalLineX, Constants.kCenterGoalLineY, new Rotation2d(Constants.kCenterGoalLineStartAngle)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kTrenchPathMidpointX,Constants.kTrenchPathMidpointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kTrenchPathEndX, Constants.kTrenchPathEndY, new Rotation2d(Constants.kTrenchPathEndAngle)),
        // Pass config
        config);
      }
      private Trajectory getTrenchToLineTrajectory(TrajectoryConfig config)
      {
        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kTrenchPathEndX,Constants.kTrenchPathEndY, new Rotation2d(Constants.kTrenchPathEndAngle)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kTrenchPathMidpointX, Constants.kTrenchPathMidpointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kCenterGoalLineX, Constants.kCenterGoalLineY, new Rotation2d(Constants.kCenterGoalLineStartAngle)),
        // Pass config
        config);
      }
      private Trajectory getLineToLeftDiamondTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kCenterGoalLineX, Constants.kCenterGoalLineY, new Rotation2d(Constants.kCenterGoalLineStartAngle)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kLeftDiamondPathMidpointX, Constants.kLeftDiamondPathMidpointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kLeftDiamondPathEndX, Constants.kLeftDiamondPathEndY, Rotation2d.fromDegrees(Constants.kLeftDiamondPathEndAngle)),
        // Pass config
        config);
      }
      private Trajectory getLeftDiamondToLineTrajectory(TrajectoryConfig config)
      {
        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kLeftDiamondPathEndX, Constants.kLeftDiamondPathEndY, Rotation2d.fromDegrees(Constants.kLeftDiamondPathEndAngle)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kLeftDiamondPathMidpointX, Constants.kLeftDiamondPathMidpointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kCenterGoalLineX, Constants.kCenterGoalLineY, new Rotation2d(Constants.kCenterGoalLineStartAngle)),
        // Pass config
        config);
      }
      private Trajectory getLeftDiamondToTrenchTrajectory(TrajectoryConfig config)
      {
        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kLeftDiamondPathEndX, Constants.kLeftDiamondPathEndY, Rotation2d.fromDegrees(Constants.kLeftDiamondPathEndAngle)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kLeftDiamondTrenchPathMidpointX, Constants.kLeftDiamondTrenchPathMidpointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kLeftDiamondTrenchPathEndX, Constants.kLeftDiamondTrenchPathEndY, new Rotation2d(Constants.kLeftDiamondTrenchPathEndAngle)),
        // Pass config
        config);
      }
      private Trajectory getAfterLeftDiamondToTrenchTrajectory(TrajectoryConfig config)
      {
        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kLeftDiamondTrenchPathEndX, Constants.kLeftDiamondTrenchPathEndY, new Rotation2d(Constants.kLeftDiamondTrenchPathEndAngle)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kAfterLeftDiamondPathMidpointX, Constants.kAfterLeftDiamondPathMidpointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kAfterLeftDiamondPathEndX, Constants.kAfterLeftDiamondPathEndY, new Rotation2d(Constants.kAfterLeftDiamondPathEndAngle)),
        // Pass config
        config);
      }
      private Trajectory getLineToStealTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1.68, 0.0)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2.65, 0.0, new Rotation2d(0)),
        // Pass config
        config);
      }
      private Trajectory getStealToGoalTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(2.65, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(0.82, 1.04)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.52, 5.0, Rotation2d.fromDegrees(-90)),
        // Pass config
        config);
      }
    }