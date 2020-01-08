/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class DriveTrain implements Subsystem {
  
  private final WPI_TalonSRX leftMaster = 
    new WPI_TalonSRX(Constants.DriveConstants.kLeftMasterID);

  private final WPI_TalonSRX leftFollower = 
    new WPI_TalonSRX(Constants.DriveConstants.kRightFollowerID);
      
  private final WPI_TalonSRX rightMaster = 
    new WPI_TalonSRX(Constants.DriveConstants.kRightMasterID);

  private final WPI_TalonSRX rightFollower =
    new WPI_TalonSRX(Constants.DriveConstants.kRightFollowerID);

  private final AHRS navx = new AHRS(Port.kMXP);
  
  public DriveTrain() {

    //Configure motor inversions/sensor phase
    leftMaster.setInverted(false);
    leftMaster.setSensorPhase(true);

    leftFollower.setInverted(false);

    rightMaster.setInverted(true);
    leftMaster.setSensorPhase(true);

    rightFollower.setInverted(true);

    //Set neutral mode
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    //Set followers to masters
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    //Config encoders
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
  }

  @Override
  public void periodic() {

  }

  public void setPIDConfigIDx(int config){

  }
}
