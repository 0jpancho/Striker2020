/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {

  public final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.DriveConstants.kLeftMasterID);

  public final WPI_TalonSRX leftFollower = new WPI_TalonSRX(Constants.DriveConstants.kLeftFollowerID);

  public final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.DriveConstants.kRightMasterID);

  public final WPI_TalonSRX rightFollower = new WPI_TalonSRX(Constants.DriveConstants.kRightFollowerID);

  private final AHRS navx = new AHRS(Port.kMXP);

  public DriveBase() {

    //Rest configs back to default - prevents conflicts
    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();

    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    //Set followers to masters
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    //Configure motor inversions/sensor phase
    leftMaster.setInverted(false);
    leftFollower.setInverted(InvertType.FollowMaster);
    leftMaster.setSensorPhase(true);

    rightMaster.setInverted(true);
    rightFollower.setInverted(InvertType.FollowMaster);
    rightMaster.setSensorPhase(false);

    //Set neutral mode
    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    //Config encoders
    // TODO Config Talons per command use. Drive base difference between auton/driving
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

    leftMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    //Reset sensors
    resetEncoders();
    resetHeading();

    navx.enableLogging(false);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Left Enc Velo", getLeftVelo());
      SmartDashboard.putNumber("Right Enc Velo", getRightVelo());

      SmartDashboard.putNumber("Left Enc Pos", getLeftPos());
      SmartDashboard.putNumber("Right Enc Pos", getRightPos());
      
      SmartDashboard.putNumber("Heading", navx.getYaw());
      SmartDashboard.putBoolean("NavX Cal?", navx.isCalibrating());
      SmartDashboard.putBoolean("NavX Alive?", navx.isConnected());
  }

  public void ArcadeDrive(DoubleSupplier forward, DoubleSupplier rotation){
      leftMaster.set(ControlMode.PercentOutput, -forward.getAsDouble() - rotation.getAsDouble());
      rightMaster.set(ControlMode.PercentOutput, -forward.getAsDouble() + rotation.getAsDouble());
  }

  public void tankDrive(DoubleSupplier left, DoubleSupplier right){
    leftMaster.set(ControlMode.PercentOutput, -left.getAsDouble());
    rightMaster.set(ControlMode.PercentOutput, -right.getAsDouble());
  }
  
  public Rotation2d getHeading() {
    float angle = navx.getYaw();
    return Rotation2d.fromDegrees((double)angle);
  }
  
  public void resetHeading(){
    navx.reset();
  }
  
  public int getLeftVelo(){
    return leftMaster.getSensorCollection().getQuadratureVelocity();
  }

  public int getRightVelo(){
    return rightMaster.getSensorCollection().getQuadratureVelocity();
  }

  public int getLeftPos(){
    return leftMaster.getSensorCollection().getQuadraturePosition();
  }

  public int getRightPos(){
    return rightMaster.getSensorCollection().getQuadraturePosition();
  }

  public void resetEncoders(){
    leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
    rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
  }

  public void setPIDConfig(int config, double kP, double kI, double kD, double timeout){
    
  }

  public void testMotors(boolean leftM, boolean leftS, boolean rightM, boolean rightS){
    if(leftM){
      leftMaster.set(ControlMode.PercentOutput, 1);
    }
    else{
      leftMaster.set(ControlMode.PercentOutput, 0);
    }

    if(leftS){
      leftFollower.set(ControlMode.PercentOutput, 1);
    }
    else{
      leftFollower.set(ControlMode.PercentOutput, 0);
    }

    if(rightM){
      rightMaster.set(ControlMode.PercentOutput, 1);
    }
    else{
      rightMaster.set(ControlMode.PercentOutput, 0);
    }

    if(rightS){
      rightFollower.set(ControlMode.PercentOutput, 1);
    }
    else{
      rightFollower.set(ControlMode.PercentOutput, 0);
    }
  }
}
