package frc.robot.Utilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Configuration.Constants;

public class PathFollower {
    private RamseteController _controller;
    private Trajectory _currentPath;
    private DifferentialDriveKinematics _kinematics;

    public PathFollower()
    {
        _controller = new RamseteController();
    }
    public void setPath(TrajectoryConfig config)
    {
        _currentPath = generateTrajectory(config);
    }
    public Trajectory getCurrentTrajectory()
    {
        return _currentPath;
    }
    public Pose2d getStartingPose()
    {
        return _currentPath.getInitialPose();
    }
    public DifferentialDriveWheelSpeeds getPathTarget(double time, Pose2d currentPose)
    {
        var goal = _currentPath.sample(time);
        var targetSpeeds = _controller.calculate(currentPose, goal);
        SmartDashboard.putNumber("goalDegrees", goal.poseMeters.getRotation().getDegrees());

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
      private Trajectory generateTrajectory(TrajectoryConfig config) {

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
}