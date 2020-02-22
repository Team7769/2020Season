package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private NetworkTableEntry _targetX;
    private NetworkTableEntry _targetY;
    private NetworkTableEntry _validTargets;
    private NetworkTableEntry _camMode;

    private static Limelight _instance;

    public Limelight()
    {
        _targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
        _targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
        _validTargets = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
        _camMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode");

    }
    public static Limelight GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Limelight();
        }
        return _instance;
    }
    public double getAngleToTarget()
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
    public double getYAngle()
    {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
    public boolean hasTarget()
    {
        return _validTargets.getDouble(0) == 1;
    }
    public boolean isAimbot()
    {
        return _camMode.getDouble(0) == 0;
    }
    public void setDashcam()
    {
        _camMode.setDouble(1);
    }
    public void setAimbot()
    {
        _camMode.setDouble(0);
    }
}