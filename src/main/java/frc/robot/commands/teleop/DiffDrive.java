/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DiffDrive implements Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private DriveBase m_DriveBase = new DriveBase();

  private final DifferentialDriveKinematics m_kinematics = 
    new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);

  private final DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(m_DriveBase.getHeading());

  
  private final PIDController m_LPID = new PIDController(0.1, 0, 0);
  private final PIDController m_RPID = new PIDController(0.1, 0, 0);
  
  double forward = 0;
  double rot = 0;
  
  public DiffDrive(DriveBase subsystem, double forward, double rot) {
    m_DriveBase = subsystem;
    this.forward = forward;
    this.rot = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveBase.resetEncoders();
    m_DriveBase.resetHeading();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateOdometry();

    drive(forward, rot);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    double leftOutput = m_LPID.calculate(m_DriveBase.getLeftEncVelo(), 
      speeds.leftMetersPerSecond);
    double rightOutput = m_RPID.calculate(m_DriveBase.getRightEncVelo(),
      speeds.rightMetersPerSecond);

    m_DriveBase.leftMaster.set(ControlMode.PercentOutput, leftOutput);
    m_DriveBase.rightMaster.set(ControlMode.PercentOutput, rightOutput);
  }

  public void drive(double forward, double rot){
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(forward, 0.0, rot));
    setSpeeds(wheelSpeeds);

  }

  public void updateOdometry(){
    m_odometry.update(m_DriveBase.getHeading(), m_DriveBase.getLeftEncVelo(), m_DriveBase.getRightEncVelo());
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

  @Override
  public Set<Subsystem> getRequirements() {
    return null;
  }
}
