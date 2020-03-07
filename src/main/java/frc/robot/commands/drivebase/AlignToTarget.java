package frc.robot.commands.drivebase;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.vision.Limelight;

public class AlignToTarget extends CommandBase {

    private Drivebase m_drive;
    private Limelight m_limelight;
    
    double kPAim = -0.1f;
    double kPDist = -0.1f;
    double kMinAimPower = 0.05f;
    //double kTargetDist;

    public AlignToTarget(Drivebase drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;

        //kTargetDist = targetDist;

        addRequirements(drive, limelight);
    }

    @Override
    public void initialize() {
        m_limelight.setPipeline(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double headingError = m_limelight.getDegRotationToTarget();
        //double distError = kTargetDist - m_shooter.getDistToTarget(m_limelight.getDegVerticalToTarget());
        double distError = m_limelight.getDegVerticalToTarget();

        SmartDashboard.putNumber("Heading Error", headingError);
        SmartDashboard.putNumber("Dist Error", distError);

        double rotCorrect = 0;

        if (headingError > 1) {
            rotCorrect = kPAim * headingError - kMinAimPower;
        }

        else if (headingError < 1) {
            rotCorrect = kPAim * headingError + kMinAimPower;
        }

        double distCorrect = kPDist * distError;

        double leftPow =+ rotCorrect + distCorrect;
        double rightPow =- rotCorrect + distCorrect;

        m_drive.getLeftMaster().set(ControlMode.PercentOutput, leftPow);
        m_drive.getRightMaster().set(ControlMode.PercentOutput, rightPow);
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
}