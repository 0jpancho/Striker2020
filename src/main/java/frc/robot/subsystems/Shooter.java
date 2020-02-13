package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Shooter implements Subsystem{

    public final WPI_TalonSRX left = new WPI_TalonSRX(Constants.ShooterConstants.kLeftShooter);
    public final WPI_TalonSRX right = new WPI_TalonSRX(Constants.ShooterConstants.kRightShooter);

    private TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

    private ControlMode controlMode;
    private double sendableMotorVal;

    public Shooter(){
        
        left.configFactoryDefault();
        right.configFactoryDefault();

        motorConfig.slot0.kP = Constants.ShooterConstants.kGains.kP;
        motorConfig.slot0.kI = Constants.ShooterConstants.kGains.kI;
        motorConfig.slot0.kD = Constants.ShooterConstants.kGains.kD;
        motorConfig.slot0.kF = Constants.ShooterConstants.kGains.kF;
       
        motorConfig.continuousCurrentLimit = 40;
        motorConfig.peakCurrentDuration = 0;
      
        motorConfig.slot0.kP = 0;
        motorConfig.slot0.kI = 0;
        motorConfig.slot0.kD = 0;
        motorConfig.slot0.kF = 0;

        left.configAllSettings(motorConfig);
        right.configAllSettings(motorConfig);
        
        left.setSensorPhase(false);
        right.setSensorPhase(true);
        
        left.setInverted(false);
        right.setInverted(true);    
        

        left.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
                                            Constants.ShooterConstants.kPIDLoopIdx, 
                                            Constants.ShooterConstants.kTimeoutMs);
        right.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
                                            Constants.ShooterConstants.kPIDLoopIdx, 
                                            Constants.ShooterConstants.kTimeoutMs);

        left.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        left.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

        right.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        right.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        right.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

        left.configNominalOutputForward(0, Constants.ShooterConstants.kTimeoutMs);
		left.configNominalOutputReverse(0, Constants.ShooterConstants.kTimeoutMs);
		left.configPeakOutputForward(1, Constants.ShooterConstants.kTimeoutMs);
        left.configPeakOutputReverse(-1, Constants.ShooterConstants.kTimeoutMs);
        
        right.configNominalOutputForward(0, Constants.ShooterConstants.kTimeoutMs);
		right.configNominalOutputReverse(0, Constants.ShooterConstants.kTimeoutMs);
		right.configPeakOutputForward(1, Constants.ShooterConstants.kTimeoutMs);
        right.configPeakOutputReverse(-1, Constants.ShooterConstants.kTimeoutMs);
    }

    @Override
    public void periodic(){
        left.set(controlMode, sendableMotorVal);
        right.set(controlMode, sendableMotorVal);
    }

    public void setShooterConfig(ControlMode mode, double value){
        this.controlMode = mode;
        this.sendableMotorVal = value;
    }

    public void setBrake(boolean isEnabled){
        left.setInverted(isEnabled);
        right.setInverted(isEnabled);
    }
}