package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class TurnPID extends CommandBase {

    private Drivebase m_drive = new Drivebase();

    private double targetAngle = 0;
    private double tolerance = 0;

    private PIDController controller = new PIDController(Constants.Drive.kTurnGains.kP,
            Constants.Drive.kTurnGains.kI, Constants.Drive.kTurnGains.kD);

    public TurnPID(Drivebase drive, double targetAngle, double tolerance) {
        this.m_drive = drive;

        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        m_drive.resetHeading();
        m_drive.configMotors(ControlMode.PercentOutput, 0);

        SendableRegistry.add(controller, "Turn Controller");
        controller.reset();

        controller.setTolerance(tolerance);
        controller.enableContinuousInput(-180, 180);
        controller.setSetpoint(targetAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.leftMaster.set(ControlMode.PercentOutput, controller.calculate(m_drive.getHeadingDegrees()));
        m_drive.rightMaster.set(ControlMode.PercentOutput, controller.calculate(-m_drive.getHeadingDegrees()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(targetAngle - m_drive.getHeadingDegrees()) < 1) {
            return true;
          }
          return false;
    }

}