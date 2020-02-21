package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Shooter implements Subsystem{

    public final WPI_TalonSRX left = new WPI_TalonSRX(Constants.ShooterConstants.kLeftShooter);
    public final WPI_TalonSRX right = new WPI_TalonSRX(Constants.ShooterConstants.kRightShooter);

    private TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

    private ControlMode controlMode = ControlMode.PercentOutput;
    private double sendableMotorVal;

    public Shooter(){
        
        left.configFactoryDefault();
        right.configFactoryDefault();

        motorConfig.slot0.kP = Constants.ShooterConstants.kGains.kP;
        motorConfig.slot0.kI = Constants.ShooterConstants.kGains.kI;
        motorConfig.slot0.kD = Constants.ShooterConstants.kGains.kD;
        motorConfig.slot0.kF = Constants.ShooterConstants.kGains.kF;

        motorConfig.nominalOutputForward = 0;
        motorConfig.nominalOutputReverse = 0;
        motorConfig.peakOutputForward = 1;
        motorConfig.peakOutputReverse = -1;
       
        motorConfig.continuousCurrentLimit = 40;
        motorConfig.peakCurrentDuration = 0;

        left.configAllSettings(motorConfig);
        right.configAllSettings(motorConfig);
        
        left.setSensorPhase(false);
        right.setSensorPhase(true);
        
        left.setInverted(false);
        right.setInverted(true);    
        

        left.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 
                                          Constants.ShooterConstants.kPIDLoopIdx, 
                                          Constants.ShooterConstants.kTimeoutMs);
        right.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 
                                           Constants.ShooterConstants.kPIDLoopIdx, 
                                           Constants.ShooterConstants.kTimeoutMs);

        left.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        left.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

        right.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        right.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        right.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

        resetEncoders();

        System.out.println("Shooter Initialized");
    }

    @Override
    public void periodic(){
        left.set(controlMode, sendableMotorVal);
        right.set(controlMode, sendableMotorVal);

        SmartDashboard.putNumber("Left Enc Pos", left.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Enc Velo", right.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Right Enc Pos", right.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Enc Velo", right.getSelectedSensorVelocity());
    }

    public void configMotors(ControlMode mode, double value){
        this.controlMode = mode;
        this.sendableMotorVal = value;
    }

    public void resetEncoders(){
        left.getSensorCollection().setQuadraturePosition(0, Constants.ShooterConstants.kTimeoutMs);
        right.getSensorCollection().setQuadraturePosition(0, Constants.ShooterConstants.kTimeoutMs);
    }

    public void setBrake(boolean isEnabled){
        left.setInverted(isEnabled);
        right.setInverted(isEnabled);
    }
}