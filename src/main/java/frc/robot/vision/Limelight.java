package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.ControlMode.CamMode;
import frc.robot.vision.ControlMode.LedMode;
import frc.robot.vision.ControlMode.Snapshot;
import frc.robot.vision.ControlMode.StreamType;

public class Limelight extends SubsystemBase {

    private NetworkTable m_table;
    private String m_tableName;

    public Limelight() {
        m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);

        setStream(ControlMode.StreamType.kPiPMain);
    }

    public boolean getIsTargetFound() {
        double v = m_table.getEntry("tv").getDouble(0.0);
        if (v == 0.0f){
            return false;
        } else {
            return true;
        }
    }

    public double getDegRotationToTarget() {
        return m_table.getEntry("tx").getDouble(0.0);
    }

    public double getDegVerticalToTarget() {
        return m_table.getEntry("ty").getDouble(0.0);
    }

    public double getTargetArea() {
        return m_table.getEntry("ta").getDouble(0.0);
    }

    public double getSkewRot() {
        return m_table.getEntry("ts").getDouble(0.0);
    }

    public double getPipelineLatency() {
        return m_table.getEntry("tl").getDouble(0.0);
    }

    public void setLEDMode(LedMode ledMode) {
        m_table.getEntry("ledMode").setValue(ledMode.getValue());
    }

    public LedMode getLEDMode() {
        NetworkTableEntry ledMode = m_table.getEntry("ledMode");
        double led = ledMode.getDouble(0);
        LedMode mode = LedMode.getByValue(led);
        return mode;
    }

    public void setCamMode(CamMode camMode) {
        m_table.getEntry("camMode").setValue(camMode.getValue());
    }

    public CamMode getCamMode() {
        NetworkTableEntry camMode = m_table.getEntry("camMode");
        double cam = camMode.getDouble(0.0);
        CamMode mode = CamMode.getByValue(cam);
        return mode;
    }

    public void setPipeline(Integer pipeline) {
        if(pipeline<0){
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        }else if(pipeline>9){
            pipeline = 9;
            throw new IllegalArgumentException("Pipeline can not be greater than nine");
        }
        m_table.getEntry("pipeline").setValue(pipeline);
    }

    public double getPipeline(){
        return m_table.getEntry("pipeline").getDouble(0.0);
    }
     /**
         * stream   Sets limelight’s streaming mode
         * 
         * kStandard - Side-by-side streams if a webcam is attached to Limelight
         * kPiPMain - The secondary camera stream is placed in the lower-right corner of the primary camera stream
         * kPiPSecondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
         * 
         * @param stream
         */ 
        public void setStream(StreamType stream) {
            m_table.getEntry("stream").setValue(stream.getValue());
        }
    
        public StreamType getStream() {
            NetworkTableEntry stream = m_table.getEntry("stream");
            double st = stream.getDouble(0.0);
            StreamType mode = StreamType.getByValue(st);
            return mode;
        }

         /**
         * snapshot Allows users to take snapshots during a match
         * 
         * kon - Stop taking snapshots
         * koff - Take two snapshots per second
         * @param snapshot
         */
        public void setSnapshot(Snapshot snapshot) {
            m_table.getEntry("snapshot").setValue(snapshot.getValue());
        }
    
        public Snapshot getSnapshot() {
            NetworkTableEntry snapshot = m_table.getEntry("snapshot");
            double snshot = snapshot.getDouble(0.0);
            Snapshot mode = Snapshot.getByValue(snshot );        
            return mode;
        }
}