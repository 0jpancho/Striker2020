package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
*
*/
public class ControlMode extends SubsystemBase {

    public enum LedMode {
        kpipeLine(0),   //0	use the LED Mode set in the current pipeline
        kforceOff(1),   //1	force off
        kforceBlink(2), //2	force blink
        kforceOn(3);    //3	force on 

        private static final Map<Double, LedMode> MY_MAP = new HashMap<Double, LedMode>();
        
        static {
            for (LedMode LedMode : values()) {
            MY_MAP.put(LedMode.getValue(), LedMode);
            }
        }
        
        private double value;

        private LedMode(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public static LedMode getByValue(double value) {
            return MY_MAP.get(value);
        }

        public String toString() {
            return name();
        }
}

    public enum CamMode {
        kvision(0),
        klightsOff(1);
    
        private static final Map<Double, CamMode> MY_MAP = new HashMap<Double, CamMode>();
  
        static {
            for (CamMode CamMode : values()) {
            MY_MAP.put(CamMode.getValue(), CamMode);
            }
        }
        
        private int value;

        private CamMode(int value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public static CamMode getByValue(double value) {
            return MY_MAP.get(value);
        }

        public String toString() {
            return name();
        }
    }

    public enum StreamType {
        kStandard(0),
        kPiPMain(1),
        kPiPSecondary(2);
    
        private static final Map<Double,  StreamType> MY_MAP = new HashMap<Double,  StreamType>();
        
        static {
            for ( StreamType  StreamType : values()) {
            MY_MAP.put( StreamType.getValue(),  StreamType);
            }
        }
        
        private double value;

        private  StreamType(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public static  StreamType getByValue(double value) {
            return MY_MAP.get(value);
        }

        public String toString() {
            return name();
        }

    }

    public enum Snapshot {

        kon(1), koff(0);
      
        private static final Map<Double,  Snapshot> MY_MAP = new HashMap<Double,  Snapshot>();
        
        static {
          for (Snapshot Snapshot : values()) {
            MY_MAP.put( Snapshot.getValue(),  Snapshot);
          }
        }
        
        private double value;
      
        private  Snapshot(double value) {
          this.value = value;
        }
      
        public double getValue() {
          return value;
        }
      
        public static  Snapshot getByValue(double value) {
          return MY_MAP.get(value);
        }
      
        public String toString() {
          return name();
        }
      }
}