package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.vision.Limelight;
import frc.robot.vision.ControlMode.LedMode;

public class TurnToTarget extends CommandBase {

    private Drivebase m_drive;
    private Limelight m_limelight;

    private int m_pipeline;

    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    public TurnToTarget(Drivebase drive, Limelight limelight, int pipeline) {
        m_drive = drive;
        m_limelight = limelight;
        m_pipeline = pipeline;

        addRequirements(m_drive, m_limelight);
    }

    @Override
    public void initialize() {

        m_limelight.setPipeline(m_pipeline);
        m_limelight.setLEDMode(LedMode.kforceOn);
        System.out.println("TurnToTarget Initialized");

        if (m_LimelightHasValidTarget) {
            m_drive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);
        } else {
            m_drive.arcadeDrive(0.0, 0.0);
        }
    }

    @Override
    public void execute() {

        Update_Limelight_Tracking();

        if (m_LimelightHasValidTarget) {
            m_drive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);
        } else {
            m_drive.arcadeDrive(0.0, 0.0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void Update_Limelight_Tracking() {
        // These numbers must be tuned for your Robot! Be careful!
        final double STEER_K = 0.01; // how hard to turn toward the target
        final double DRIVE_K = 0.1; // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.5; // Simple speed limit so we don't drive too fast

        boolean tv = m_limelight.getIsTargetFound();
        double tx = m_limelight.getDegRotationToTarget();
        double ty = m_limelight.getDegVerticalToTarget();
        double ta = m_limelight.getTargetArea();

        if (!tv) {
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }
}