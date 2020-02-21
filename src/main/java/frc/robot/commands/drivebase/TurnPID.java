package frc.robot.commands.drivebase;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class TurnPID extends CommandBase {

    private DriveBase m_drive = new DriveBase();

    private double targetAngle = 0;
    private double tolerance = 0;

    private PIDController turnController = new PIDController(Constants.DriveConstants.kTurnGains.kP, 
                                                             Constants.DriveConstants.kTurnGains.kI, 
                                                             Constants.DriveConstants.kTurnGains.kD);

    public TurnPID(DriveBase drive, double targetAngle, double tolerance){
        this.m_drive = drive;

        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        m_drive.resetHeading();
        m_drive.configMotors(ControlMode.PercentOutput, 0);

        SendableRegistry.add(turnController, "Turn Controller");
        turnController.reset();

        turnController.setTolerance(tolerance);
        turnController.enableContinuousInput(-180, 180);
        turnController.setSetpoint(targetAngle);
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_drive.leftMaster.set(ControlMode.PercentOutput, turnController.calculate(m_drive.getHeadingDegrees()));
      m_drive.rightMaster.set(ControlMode.PercentOutput, turnController.calculate(-m_drive.getHeadingDegrees()));
    }
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }
    
}