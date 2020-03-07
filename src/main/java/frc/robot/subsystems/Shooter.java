package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Shooter implements Subsystem {

    public final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.Shooter.kLShooterID);
    public final WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.Shooter.kRShooterID);

    private TalonSRXConfiguration motorConfig = new TalonSRXConfiguration();

    private double motorVal;

    public Shooter() {
        leftMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();

        motorConfig.slot0.kP = Constants.Shooter.kGains.kP;
        motorConfig.slot0.kI = Constants.Shooter.kGains.kI;
        motorConfig.slot0.kD = Constants.Shooter.kGains.kD;
        motorConfig.slot0.kF = Constants.Shooter.kGains.kF;

        motorConfig.nominalOutputForward = 0;
        motorConfig.nominalOutputReverse = 0;
        motorConfig.peakOutputForward = 1;
        motorConfig.peakOutputReverse = -1;

        motorConfig.continuousCurrentLimit = 35;
        motorConfig.peakCurrentDuration = 0;

        leftMaster.configAllSettings(motorConfig);
        rightSlave.configAllSettings(motorConfig);

        leftMaster.setInverted(false);
        rightSlave.setInverted(true);

        // SET TO TRUE AFTER QUAL 71
        leftMaster.setSensorPhase(false);
        rightSlave.setSensorPhase(false);

        rightSlave.follow(leftMaster);

        leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);
        rightSlave.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.kPIDLoopIdx,
                Constants.Shooter.kTimeoutMs);

        leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        rightSlave.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        rightSlave.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        rightSlave.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        rightSlave.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        rightSlave.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 20);

        resetEncoders();

        System.out.println("Shooter Initialized");
    }

    @Override
    public void periodic() {
        
    }
    
    public double getMotorVal() {
        return motorVal;
    }

    public void resetEncoders() {
        leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.Drive.kTimeoutMs);
        rightSlave.getSensorCollection().setQuadraturePosition(0, Constants.Drive.kTimeoutMs);
    }

    public void setBrake(NeutralMode mode) {
        leftMaster.setNeutralMode(mode);
        rightSlave.setNeutralMode(mode);
    }

    public void stop() {
        leftMaster.set(0);
        rightSlave.set(0);
    }

    public WPI_TalonSRX getLeftMotor() {
        return leftMaster;
    }

    public WPI_TalonSRX getRightMotor() {
        return rightSlave;
    }

    public int getLeftVeloTicks() {
        return leftMaster.getSelectedSensorVelocity();
    }

    public int getRightVeloTicks() {
        return rightSlave.getSelectedSensorVelocity();
    }

    public double getDistToTarget(double targetY) {
        return (Constants.Limelight.kTargetHeight - Constants.Limelight.kAngleDegrees)
                / Math.tan(Units.degreesToRadians((Constants.Limelight.kAngleDegrees) - targetY));
    }
}