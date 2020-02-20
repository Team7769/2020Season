package frc.robot.CrewLib;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Configuration.Constants;

public class ProximitySensor extends AnalogInput{

    public ProximitySensor(int port)
    {
        super(port);
    }
    public boolean isBlocked() {
        return this.getVoltage() > Constants.kBallProximity;
    }

}