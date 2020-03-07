package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;

/*
 * Based on team 1388
 * https://github.com/AGHSEagleRobotics/frc1388-2020/blob/master/frc1388-2020/src/main/java/frc/robot/CompDashBoard.java 
 */

public class Dashboard {

        /**
         * 
         * Competition Tab
         * 
         */
        /*
         * // cam dimensions private final int cam2Height = 4; private final int
         * cam2Width = 4;
         * 
         * // auton chooser private final int autonChooserWidth = 3; private final int
         * autonChooserHeight = 1; private final int autonChooserColumnIndex = 0;
         * private final int autonChooserRowIndex = 0;
         * 
         * // max capacity private final int maxCapacityWidth = 3; private final int
         * maxCapacityHeight = 3; private final int maxCapacityColumnIndex = 0; private
         * final int maxCapacityRowIndex = 6; // cam private final int camWidth = 10;
         * private final int camHeight = 10; private final int camColumnIndex = 8;
         * private final int camRowIndex = 0;
         * 
         * // color spinner grid private final int colorSpinnerGridWidth = 5; private
         * final int colorSpinnerGridHeight = 7; private final int
         * colorSpinnerGridColumnIndex = 21; private final int colorSpinnerGridRowIndex
         * = 0;
         * 
         * // desired color private final int desiredColorWidth = 5; private final int
         * desiredColorHeight = 2; private final int desiredColorColumnIndex = 21;
         * private final int desiredColorRowIndex = 7;
         */

        private ShuffleboardTab shuffleboard;

        private ComplexWidget complexWidgetCam;
        private ComplexWidget complexWidgetAuton;
        private SendableChooser<AutonomousMode> autonChooser = new SendableChooser<>();
        private NetworkTableEntry maxCapacityBox;

        /*
         * private ShuffleboardLayout colorSpinnerGrid; private NetworkTableEntry
         * colorGridRed; private NetworkTableEntry colorGridGreen; private
         * NetworkTableEntry colorGridYellow; private NetworkTableEntry colorGridBlue;
         */

        // Cam
        private UsbCamera m_driverCam;
        private HttpCamera m_limeLight;
        private int m_currVideoSourceIndex = 0;
        private VideoSink m_videoSink;
        private VideoSource[] m_videoSources;

        /**
         * 
         * Diagnostics Tab
         * 
         */

        private Drivebase m_drivebase;
        private Shooter m_shooter;

        private SuppliedValueWidget<Double> driveLPos;
        private SuppliedValueWidget<Double> driveLVelo;
        private SuppliedValueWidget<Double> driveRPos;
        private SuppliedValueWidget<Double> driveRVelo;

        private SuppliedValueWidget<Double> LMetersPerSec;
        private SuppliedValueWidget<Double> LMetersTraveled;
        private SuppliedValueWidget<Double> RMetersPerSec;
        private SuppliedValueWidget<Double> RMetersTraveled;

        private SuppliedValueWidget<Double> shooterLVelo;
        private SuppliedValueWidget<Double> shooterRVelo;

        private SuppliedValueWidget<Double> heading;
        private SuppliedValueWidget<Boolean> navxAlive;
        private SuppliedValueWidget<Boolean> navxCalibrating;

        private DoubleSupplier driveLPosSup = () -> m_drivebase.getLPosTicks();
        private DoubleSupplier driveLVeloSup = () -> m_drivebase.getLVeloTicks();
        private DoubleSupplier driveRPosSup = () -> m_drivebase.getRPosTicks();
        private DoubleSupplier driveRVeloSup = () -> m_drivebase.getRVeloTicks();

        private DoubleSupplier LMetersPerSecSup = () -> m_drivebase.getLeftMetersPerSec();
        private DoubleSupplier LMetersTraveledSup = () -> m_drivebase.getLeftMetersTraveled();
        private DoubleSupplier RMetersPerSecSup = () -> m_drivebase.getRightMetersPerSec();
        private DoubleSupplier RMetersTraveledSup = () -> m_drivebase.getRightMetersTraveled();

        private DoubleSupplier shooterLVeloSup = () -> m_shooter.getLeftVeloTicks();
        private DoubleSupplier shooterRVeloSup = () -> m_shooter.getRightVeloTicks();

        private DoubleSupplier headingSup = () -> m_drivebase.getHeadingDegrees();
        private BooleanSupplier navxAliveSup = () -> m_drivebase.navxAlive();
        private BooleanSupplier navxCalibratingSup = () -> m_drivebase.navxCalibrating();

        public Dashboard(Drivebase drivebase, Shooter shooter) {

                m_drivebase = drivebase;
                m_shooter = shooter;

                //m_driverCam = CameraServer.getInstance().startAutomaticCapture();

                constructCompetitionLayout();
                constructDiagnosticsLayout();

                //camStuff();
        }

        public enum AutonomousMode {
                SHOOT_MOVE("Shoot + Move"), MOVE("Move"), SHOOT("Shoot"), ENCODERS_ARE_BAD("Encoders are Bad"), NOTHING("Nothing");

                public static final AutonomousMode DEFAULT = MOVE;

                private String name;

                private AutonomousMode(String setName) {
                        name = setName;
                }

                public String getName() {
                        return name;
                }
        }

        
        private void camStuff() {
                // m_cameraIntake = CameraServer.getInstance().startAutomaticCapture();

                // m_videoSources = new VideoSource[] { m_cameraIntake };
                //m_driverCam = new UsbCamera("Driver Cam", 1);

                //CameraServer.getInstance().addCamera(m_driverCam);
                m_limeLight = new HttpCamera("limelight", "http://10.1.1.11:5801");

                m_videoSources = new VideoSource[] { m_driverCam, };

                m_videoSink = CameraServer.getInstance().getServer();

                /*
                if (m_driverCam != null) {
                        m_videoSink.setSource(m_driverCam);
                }
                */

        }
        

        public void constructCompetitionLayout() {
                shuffleboard = Shuffleboard.getTab("Competition");

                /*
                 * complexWidgetCam = shuffleboard.add("Cams",
                 * m_videoSink.getSource()).withWidget(BuiltInWidgets.kCameraStream)
                 * .withSize(camHeight, camWidth).withPosition(camColumnIndex, camRowIndex)
                 * .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
                 */

                // complexWidgetCam2 = shuffleboard.add("LimeLight", m_limeLight)
                // .withWidget(BuiltInWidgets.kCameraStream)
                // .withSize(cam2Height, cam2Width)
                // .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

                for (Dashboard.AutonomousMode o : AutonomousMode.values()) {
                        autonChooser.addOption(o.getName(), o);
                }
                autonChooser.setDefaultOption(AutonomousMode.DEFAULT.getName(), AutonomousMode.DEFAULT);

                complexWidgetAuton = shuffleboard.add("AutonChooser", autonChooser)
                                .withWidget(BuiltInWidgets.kSplitButtonChooser).withSize(3, 1).withPosition(0, 0);

                /*
                complexWidgetCam = shuffleboard.add("Cams", m_videoSink.getSource())
                                .withWidget(BuiltInWidgets.kCameraStream).withSize(4, 4).withPosition(3, 0)
                                .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

                */
        }

        public void constructDiagnosticsLayout() {
                shuffleboard = Shuffleboard.getTab("Diagnostics");

                driveLPos = shuffleboard.addNumber("DriveLPos", driveLPosSup).withWidget(BuiltInWidgets.kTextView)
                                .withSize(1, 1).withPosition(0, 0);
                driveLVelo = shuffleboard.addNumber("DriveLVelo", driveLVeloSup).withWidget(BuiltInWidgets.kTextView)
                                .withSize(1, 1).withPosition(0, 1);
                driveRPos = shuffleboard.addNumber("DriveRPos", driveRPosSup).withWidget(BuiltInWidgets.kTextView)
                                .withSize(1, 1).withPosition(1, 0);
                driveRVelo = shuffleboard.addNumber("DriveRVelo", driveRVeloSup).withWidget(BuiltInWidgets.kTextView)
                                .withSize(1, 1).withPosition(1, 1);

                LMetersPerSec = shuffleboard.addNumber("LMetersPerSec", LMetersPerSecSup)
                                .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(2, 0);
                LMetersTraveled = shuffleboard.addNumber("LMetersTraveled", LMetersTraveledSup)
                                .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(3, 0);
                RMetersPerSec = shuffleboard.addNumber("RMetersPerSec", RMetersPerSecSup)
                                .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(2, 1);
                RMetersTraveled = shuffleboard.addNumber("RMetersTraveled", RMetersTraveledSup)
                                .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(3, 1);

                heading = shuffleboard.addNumber("Heading", headingSup).withWidget(BuiltInWidgets.kTextView)
                                .withSize(2, 1).withPosition(4, 0);
                navxAlive = shuffleboard.addBoolean("Navx Alive", navxAliveSup).withWidget(BuiltInWidgets.kBooleanBox)
                                .withSize(2, 1).withPosition(4, 1);
                navxCalibrating = shuffleboard.addBoolean("Navx Calibrating", navxCalibratingSup)
                                .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1).withPosition(4, 2);

                shooterLVelo = shuffleboard.addNumber("ShooterLVelo", shooterLVeloSup)
                                .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(6, 0);
                shooterRVelo = shuffleboard.addNumber("ShooterRVelo", shooterRVeloSup)
                                .withWidget(BuiltInWidgets.kTextView).withSize(1, 1).withPosition(7, 0);

        }

        /*
         * public void switchVideoSource() { m_currVideoSourceIndex =
         * (m_currVideoSourceIndex + 1) % m_videoSources.length; if
         * (m_videoSources[m_currVideoSourceIndex] != null) {
         * m_videoSink.setSource(m_videoSources[m_currVideoSourceIndex]); } }
         */

        public AutonomousMode getSelectedObjective() {
                return autonChooser.getSelected();
        }
}