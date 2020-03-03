/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivebase;

import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DiffDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private Drivebase m_drive;
  private XboxController m_controller;

  private SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(1);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  public DiffDrive(Drivebase drive, XboxController controller) {
    m_drive = drive;
    this.m_controller = controller;

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

    double inputForward = m_controller.getY(Hand.kLeft);
    double inputRot = m_controller.getX(Hand.kRight);
   
    if (Math.abs(inputForward) < 0.05) {
      inputForward = 0;
    }

    if (Math.abs(inputRot) < 0.05) {
      inputRot = 0;
    }

    //m_drive.updateOdometry();

    m_drive.differentialDrive(m_forwardLimiter.calculate(-inputForward) * Constants.Drive.kAdjustedMaxSpeed,
                              m_rotLimiter.calculate(-inputRot) * Constants.Drive.kAdjustedAngularSpeed);
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
