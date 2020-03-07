package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class OldDifferential extends CommandBase {

    private Drivebase m_drive = new Drivebase();
  
  private DoubleSupplier kForward;
  private DoubleSupplier kRot;
  
  public OldDifferential(Drivebase drive, DoubleSupplier forward, DoubleSupplier rot) {
    m_drive = drive;
    this.kForward = forward;
    this.kRot = rot;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_drive.resetHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double inputForward = kForward.getAsDouble();
    double inputRot = kRot.getAsDouble();

    if (Math.abs(inputForward) < 0.1){
      inputForward = 0;
    } 

    if (Math.abs(inputRot) < 0.1){
      inputRot = 0;
    }

    m_drive.updateOdometry();

    // m_drive.differentialDrive(-inputForward * Constants.Drive.kRawMaxSpeed,
    //                               -inputRot * Constants.Drive.kRawAngularSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}