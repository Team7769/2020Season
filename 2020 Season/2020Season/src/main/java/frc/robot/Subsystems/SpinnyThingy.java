package frc.robot.Subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class SpinnyThingy implements ISubsystem {
    private final I2C.Port _colorSensorPort;

    private ColorSensorV3 _colorSensor;
    private ColorMatch _colorMatcher;
    
    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private static SpinnyThingy _instance;

    public SpinnyThingy()
    {
        _colorSensorPort = I2C.Port.kOnboard;
        _colorSensor = new ColorSensorV3(_colorSensorPort);
        _colorMatcher = new ColorMatch();

        _colorMatcher.addColorMatch(kBlueTarget);
        _colorMatcher.addColorMatch(kGreenTarget);
        _colorMatcher.addColorMatch(kRedTarget);
        _colorMatcher.addColorMatch(kYellowTarget);  
        
    }
    public static SpinnyThingy GetInstance()
    {
        if (_instance == null)
        {
            _instance = new SpinnyThingy();
        }
        return _instance;
    }

    @Override
    public void LogTelemetry() {
        Color color = _colorSensor.getColor();

        SmartDashboard.putNumber("colorSensorProximity", _colorSensor.getProximity());
        SmartDashboard.putNumber("colorSensorRed", color.red);
        SmartDashboard.putNumber("colorSensorBlue", color.blue);
        SmartDashboard.putNumber("colorSensorGreen", color.green);
        SmartDashboard.putNumber("colorSensorIR", _colorSensor.getIR());

    }
    public void DetectColor(Color color){
        ColorMatchResult match = _colorMatcher.matchClosestColor(color);
        String colorString = "";

        if (match.color == kBlueTarget) {
            colorString = "Blue";
          } else if (match.color == kRedTarget) {
            colorString = "Red";
          } else if (match.color == kGreenTarget) {
            colorString = "Green";
          } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
          } else {
            colorString = "Unknown";
          }
          SmartDashboard.putString("matchedColor", colorString);
          SmartDashboard.putNumber("matchedColorConfidence", match.confidence);

    }

    @Override
    public void ReadDashboardData() {
        // TODO Auto-generated method stub

    }

}