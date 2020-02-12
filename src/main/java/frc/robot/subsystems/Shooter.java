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

    private TalonSRXConfiguration leftConfig = new TalonSRXConfiguration();
    private TalonSRXConfiguration rightConfig = new TalonSRXConfiguration();
    
    public Shooter(){
        

        left.configFactoryDefault();
        right.configFactoryDefault();
        
        left.setSensorPhase(false);
        right.setSensorPhase(true);
        
        left.setInverted(false);
        left.configContinuousCurrentLimit(40);
        left.configPeakCurrentDuration(0);
        
        right.setInverted(true);    
        right.configContinuousCurrentLimit(40);
        right.configPeakCurrentDuration(0);

        left.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
                                            Constants.ShooterConstants.kPIDLoopIdx, 
                                            Constants.ShooterConstants.kTimeoutMs);
        right.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
                                            Constants.ShooterConstants.kPIDLoopIdx, 
                                            Constants.ShooterConstants.kTimeoutMs);

        left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        right.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        left.configNominalOutputForward(0, Constants.ShooterConstants.kTimeoutMs);
		left.configNominalOutputReverse(0, Constants.ShooterConstants.kTimeoutMs);
		left.configPeakOutputForward(1, Constants.ShooterConstants.kTimeoutMs);
        left.configPeakOutputReverse(-1, Constants.ShooterConstants.kTimeoutMs);
        
        right.configNominalOutputForward(0, Constants.ShooterConstants.kTimeoutMs);
		right.configNominalOutputReverse(0, Constants.ShooterConstants.kTimeoutMs);
		right.configPeakOutputForward(1, Constants.ShooterConstants.kTimeoutMs);
        right.configPeakOutputReverse(-1, Constants.ShooterConstants.kTimeoutMs);
        
        leftConfig.slot0.kP = Constants.ShooterConstants.kGains.kP;
        leftConfig.slot0.kI = Constants.ShooterConstants.kGains.kI;
        leftConfig.slot0.kD = Constants.ShooterConstants.kGains.kD;
        leftConfig.slot0.kF = Constants.ShooterConstants.kGains.kF;

        rightConfig.slot0.kP = Constants.ShooterConstants.kGains.kP;
        rightConfig.slot0.kI = Constants.ShooterConstants.kGains.kI;
        rightConfig.slot0.kD = Constants.ShooterConstants.kGains.kD;
        rightConfig.slot0.kF = Constants.ShooterConstants.kGains.kF;

        left.configAllSettings(leftConfig);
        right.configAllSettings(rightConfig);
    }

    @Override
    public void periodic(){

    }

    public void setConfig(ControlMode controlMode, double targetVelocity){
        left.set(controlMode, targetVelocity);
        right.set(controlMode, targetVelocity);
    }
}