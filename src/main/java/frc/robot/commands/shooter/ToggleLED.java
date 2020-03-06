package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.vision.Limelight;
import frc.robot.vision.ControlMode.LedMode;

public class ToggleLED extends InstantCommand {

    private Limelight m_limelight;

    private boolean toggle = false;

    public ToggleLED(Limelight limelight) {
        m_limelight = limelight;
    }

    public void initialize() {
        toggle = !toggle;

        if (toggle) {
            m_limelight.setLEDMode(LedMode.kforceOn);
        }

        else if (!toggle) {
            m_limelight.setLEDMode(LedMode.kforceOff);
        }
    }
}