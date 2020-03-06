package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private WPI_TalonSRX m_master = new WPI_TalonSRX(Constants.Indexer.kMasterID);
    private WPI_VictorSPX m_slave = new WPI_VictorSPX(Constants.Indexer.kSlaveID);

    private ControlMode mode = ControlMode.Disabled;
    private double motorVal;

    public Indexer() {
        m_master.configFactoryDefault();
        m_slave.configFactoryDefault();

        m_master.configContinuousCurrentLimit(35);
        m_master.configPeakCurrentDuration(0);

        m_master.configNominalOutputForward(0);
        m_master.configNominalOutputReverse(0);
        m_master.configPeakOutputForward(1);
        m_master.configPeakOutputReverse(-1);

        m_slave.configNominalOutputForward(0);
        m_slave.configNominalOutputReverse(0);
        m_slave.configPeakOutputForward(1);
        m_slave.configPeakOutputReverse(-1);

        m_master.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
                Constants.Indexer.kPIDLoopIdx, Constants.Indexer.kTimeoutMs);

        m_master.setInverted(true);
        m_slave.setInverted(true);

        m_master.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        m_master.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        m_master.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

        m_slave.follow(m_master);

        System.out.println("Shooter Initialized");
    }

    public void periodic() {
        m_master.set(this.mode, this.motorVal);
    }

    public WPI_TalonSRX getMaster() {
        return m_master;
    }

    public WPI_VictorSPX getSlave() {
        return m_slave;
    }


    public void setMotors(ControlMode mode, double motorVal) {
        m_master.set(mode, motorVal);
        m_slave.set(mode, motorVal);
    }

    public void setBrake(boolean isEnabled) {
        m_master.setInverted(isEnabled);
        m_slave.setInverted(isEnabled);
    }

    public void stop() {
        m_master.set(0);
        m_slave.set(0);
    }
}