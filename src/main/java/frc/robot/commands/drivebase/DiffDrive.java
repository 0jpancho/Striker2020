/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DiffDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private Drivebase m_drive = new Drivebase();

  private DoubleSupplier forward;
  private DoubleSupplier rot;

  public DiffDrive(Drivebase drive, DoubleSupplier forward, DoubleSupplier rot) {
    m_drive = drive;
    this.forward = forward;
    this.rot = rot;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetHeading();
    m_drive.resetOdometry();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double inputForward = forward.getAsDouble();
    double inputRot = rot.getAsDouble();

    if (Math.abs(inputForward) < 0.05) {
      inputForward = 0;
    }

    if (Math.abs(inputRot) < 0.05) {
      inputRot = 0;
    }

    //m_drive.updateOdometry();

    m_drive.differentialDrive(-inputForward * Constants.Drive.kAdjustedMaxSpeed,
        -inputRot * Constants.Drive.kAdjustedAngularSpeed);
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
