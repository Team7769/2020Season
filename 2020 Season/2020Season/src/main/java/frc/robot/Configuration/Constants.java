package frc.robot.Configuration;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    //Test Robot Settings
    
    //CAN Device IDs
    public static final int kLeftFrontDriveId = 2;
    public static final int kLeftRearDriveId = 3;
    public static final int kRightFrontDriveId = 4;
    public static final int kRightRearDriveId = 5;
    public static final int kLeftShooterId = 6;
    public static final int kRightShooterId = 7;
    public static final int kLeftCollectorId = 8;
    public static final int kRightCollectorId = 9;

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

    public static final double kPathFollowingkP = 3.6;

    /*
    //Competition Robot Settings
    public static final int kLeftDriveId = 0;
    public static final int kRightDriveId = 1;

    public static final int kDriverUsbSlot = 0;
    public static final int kOperatorUsbSlot = 1;
    */
}