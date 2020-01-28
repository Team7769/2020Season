package frc.robot.Utilities;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

public class PathFollower {
    private RamseteController _controller;
    private Trajectory _currentPath;
    private DifferentialDriveKinematics _kinematics;

    public PathFollower()
    {
        _controller = new RamseteController();
        _kinematics = new DifferentialDriveKinematics(.5);
    }
    public void setPath()
    {
        _currentPath = generateTrajectory();
    }
    public Pose2d getStartingPose()
    {
        return _currentPath.getInitialPose();
    }
    public DifferentialDriveWheelSpeeds getPathTarget(double time, Pose2d currentPose)
    {
        var goal = _currentPath.sample(time);
        var targetSpeeds = _controller.calculate(currentPose, goal);

        return _kinematics.toWheelSpeeds(targetSpeeds);
    }
    private Trajectory generateTrajectory() {

        var start = new Pose2d(3.129, 2.402, Rotation2d.fromDegrees(0));
        var end = new Pose2d(8.106, .75, Rotation2d.fromDegrees(0));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(5.708, .75));
    
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(3), Units.feetToMeters(3));
        //config.setReversed(true);
    
        var trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);

        return trajectory;
      }
}