package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private WPI_TalonSRX master = new WPI_TalonSRX(Constants.Indexer.kMasterID);
    private WPI_VictorSPX slave = new WPI_VictorSPX(Constants.Indexer.kSlaveID);

    private ControlMode mode = ControlMode.Disabled;
    private double motorVal;

    public Indexer() {
        master.configFactoryDefault();
        slave.configFactoryDefault();

        master.configContinuousCurrentLimit(35);
        master.configPeakCurrentDuration(0);

        master.configNominalOutputForward(0);
        master.configNominalOutputReverse(0);
        master.configPeakOutputForward(1);
        master.configPeakOutputReverse(-1);

        slave.configNominalOutputForward(0);
        slave.configNominalOutputReverse(0);
        slave.configPeakOutputForward(1);
        slave.configPeakOutputReverse(-1);

        master.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
                Constants.Indexer.kPIDLoopIdx, Constants.Indexer.kTimeoutMs);

        master.setInverted(true);
        slave.setInverted(true);

        master.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
        master.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        master.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);

        System.out.println("Shooter Initialized");
    }

    public void periodic() {
        master.set(this.mode, this.motorVal);
    }

    public void setMotors(ControlMode mode, double motorVal) {
        master.set(mode, motorVal);
        slave.set(mode, motorVal);
    }

    public void setBrake(boolean isEnabled) {
        master.setInverted(isEnabled);
        slave.setInverted(isEnabled);
    }
}