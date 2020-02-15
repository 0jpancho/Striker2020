package frc.robot.commands.drivebase;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class TurnPID extends CommandBase {

    private DriveBase drive = new DriveBase();

    private double targetAngle = 0;
    private double tolerance = 0;

    PIDController turnController = new PIDController(Constants.DriveConstants.kTurnGains.kP, 
                                                     Constants.DriveConstants.kTurnGains.kP, 
                                                     Constants.DriveConstants.kTurnGains.kP);


    public TurnPID(DriveBase drive, double targetAngle, double tolerance){
        this.drive = drive;

        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        drive.resetHeading();
        drive.configMotors(ControlMode.PercentOutput, 0);

        turnController.setSetpoint(targetAngle);
        turnController.setTolerance(tolerance);
        turnController.enableContinuousInput(-180, 180);
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      drive.leftMaster.set(ControlMode.PercentOutput, turnController.calculate(drive.getHeadingDegrees()));
      drive.rightMaster.set(ControlMode.PercentOutput, turnController.calculate(-drive.getHeadingDegrees()));
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