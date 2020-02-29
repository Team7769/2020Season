package frc.robot.Configuration;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {

    //CAN Device IDs
    public static final int kLeftFrontDriveId = 2;
    public static final int kLeftRearDriveId = 3;
    public static final int kRightFrontDriveId = 4;
    public static final int kRightRearDriveId = 5;
    public static final int kLeftShooterId = 6;
    public static final int kRightShooterId = 7;
    public static final int kInnerCollectorId = 8;
    public static final int kOuterCollectorId = 9;
    public static final int kHoodId = 10;
    public static final int kLeftExtendoId = 11;
    public static final int kRightExtendoId = 12;
    public static final int kFrontConveyorId = 13;
    public static final int kBackConveyorId = 14;

    //Usb Controllers
    public static final int kDriverUsbSlot = 0;
    public static final int kOperatorUsbSlot = 1;

    //Spark MAX Configuration
    public static final double kDriveRampRateSeconds = 0.1;
    public static final int kDriveSmartCurrentLimitAmps = 40;

    //DIO
    public static final int kLeftEncoderPortA = 0;
    public static final int kLeftEncoderPortB = 1;
    public static final int kRightEncoderPortA = 2;
    public static final int kRightEncoderPortB = 3;
    public static final int kHoodEncoderPortA = 4;
    public static final int kHoodEncoderPortB = 5;
    public static final int kExtendoEncoderPortA = 6;
    public static final int kExtendoEncoderPortB = 7;

    //Analog
    public static final int kIntakeSensorPort = 0;
    public static final int kIndexSensorPort = 1;
    public static final int kOutIndexSensorPort = 2;

    //Solenoids
    public static final int kCollectorChannelForward = 0;
    public static final int kCollectorChannelReverse = 1;
    public static final int kExtendoLockChannelForward = 2;
    public static final int kExtendoLockChannelReverse = 3;
    public static final int kLeftShooterChannel = 4;
    public static final int kRightShooterChannel = 5;
    public static final int kConveyorChannel = 6;

    //Test Robot Configuration

    public static final int kDriveEncoderTicksPerRevolution = 2048;
    public static final double kDriveWheelDiameter = 0.15;
    public static final double kDriveDistancePerPulse = (kDriveWheelDiameter * Math.PI) / kDriveEncoderTicksPerRevolution;

    //Drive Characteristics
    public static final double ksVolts = 0.149;
    public static final double kvVoltSecondsPerMeter = 2.86;
    public static final double kaVoltSecondsSquaredPerMeter = 0.252;
    
    //
    public static final double kTrackwidthMeters = 0.6223;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    //public static final double kMaxSpeedMetersPerSecond = 1.25;
    //public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    //Path Following
    public static final double kPathFollowingkP = 4.5;

    //Starting Point- Center Line
    public static final int kCenterGoalLineX = 0;
    public static final int kCenterGoalLineY = 0;
    public static final int kCenterGoalLineStartAngle = 0;
    //Trench Path Points
    public static final double kTrenchPathMidpointX = 2.2;
    public static final double kTrenchPathMidpointY = 1.40;
    public static final double kTrenchPathEndX = 5.1;
    public static final double kTrenchPathEndY = 1.50;
    public static final double kTrenchPathEndAngle = 0.0;

    //To-the-Rendevous(from the left)
    public static final double kLeftDiamondPathMidpointX = 1.6;
    public static final double kLeftDiamondPathMidpointY = 0.6;
    public static final double kLeftDiamondPathEndX = 2.75;
    public static final double kLeftDiamondPathEndY = -0.4;
    public static final double kLeftDiamondPathEndAngle = -60;

    //To-the-Trench(from the rendevous)
    public static final double kLeftDiamondTrenchPathMidpointX = 2.358;
    public static final double kLeftDiamondTrenchPathMidpointY = 0.778;
    public static final double kLeftDiamondTrenchPathEndX = 1.29;
    public static final double kLeftDiamondTrenchPathEndY = 1.5;
    public static final int kLeftDiamondTrenchPathEndAngle = 0;
    
    //Line Up
    public static final double kAfterLeftDiamondPathMidpointX = 3.25;
    public static final double kAfterLeftDiamondPathMidpointY = 1.5;
    public static final double kAfterLeftDiamondPathEndX = 5.1;
    public static final double kAfterLeftDiamondPathEndY = 1.5;
    public static final int kAfterLeftDiamondPathEndAngle = 0;


    public static final double kTurnkP = 0.067;
    public static final double kTurnkI = 0.0;
    public static final double kTurnkD = 0.0035;

    public static final double kHoodPositionkP = 1.25;
    public static final double kHoodPositionkI = 0.0;
    public static final double kHoodPositionkD = 0.0;

    public static final double kGoalHeight = 4.26; //feet

    public static final double kPopShotVelocity = 15000;
    public static final double kLineShotVelocity = 12000;
    public static final double kTrenchShotVelocity = 16000;
    public static final double kFarShotVelocity = 23000;

    public static final double kPopShotHoodPosition = 2.4;
    public static final double kLineShotHoodPosition = 1.6;
    public static final double kTrenchShotHoodPosition = 1.0;
    public static final double kFarShotHoodPosition = .517;

    public static final double kBallProximity = 1.0;
    
    public static final double kMotorTemperatureThreshold = 32; //Degrees Celsius

    /*
    //Competition Robot Settings

    public static final int kDriveEncoderTicksPerRevolution = 2048;
    public static final double kDriveWheelDiameter = 0.15;
    public static final double kDriveDistancePerPulse = (kDriveWheelDiameter * Math.PI) / kDriveEncoderTicksPerRevolution;

    //Drive Characteristics
    public static final double ksVolts = 0.149;
    public static final double kvVoltSecondsPerMeter = 2.86;
    public static final double kaVoltSecondsSquaredPerMeter = 0.252;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 9.77;

    //
    public static final double kTrackwidthMeters = 0.6223;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    //public static final double kMaxSpeedMetersPerSecond = 1.25;
    //public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kPathFollowingkP = 4.5;

    public static final double kTurnkP = 0.067;
    public static final double kTurnkI = 0.0;
    public static final double kTurnkD = 0.0035;

    public static final double kHoodPositionkP = 0.25;
    public static final double kHoodPositionkI = 0.0;
    public static final double kHoodPositionkD = 0.0;

    public static final double kGoalHeight = 4.26; //feet

    public static final double kPopShotVelocity = 15000;
    public static final double kLineShotVelocity = 12000;
    public static final double kTrenchShotVelocity = 16000;
    public static final double kFarShotVelocity = 23000;

    public static final double kPopShotHoodPosition = -.3;
    public static final double kLineShotHoodPosition = -.5;
    public static final double kTrenchShotHoodPosition = -.8;
    public static final double kFarShotHoodPosition = -1.0;

    public static final double kBallProximity = 1.0;
    
    public static final double kMotorTemperatureThreshold = 32; //Degrees Celsius
    */
}