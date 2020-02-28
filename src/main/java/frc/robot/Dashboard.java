package frc.robot;

import java.util.Map;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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

    // cam dimensions
    private final int cam2Height = 4;
    private final int cam2Width = 4;

    // auton chooser
    private final int autonChooserWidth = 8;
    private final int autonChooserHeight = 2;
    private final int autonChooserColumnIndex = 0;
    private final int autonChooserRowIndex = 0;

    // max capacity
    private final int maxCapacityWidth = 3;
    private final int maxCapacityHeight = 3;
    private final int maxCapacityColumnIndex = 0;
    private final int maxCapacityRowIndex = 6;
    // cam
    private final int camWidth = 10;
    private final int camHeight = 10;
    private final int camColumnIndex = 8;
    private final int camRowIndex = 0;

    // color spinner grid
    private final int colorSpinnerGridWidth = 5;
    private final int colorSpinnerGridHeight = 7;
    private final int colorSpinnerGridColumnIndex = 21;
    private final int colorSpinnerGridRowIndex = 0;

    // desired color
    private final int desiredColorWidth = 5;
    private final int desiredColorHeight = 2;
    private final int desiredColorColumnIndex = 21;
    private final int desiredColorRowIndex = 7;

    private ShuffleboardTab shuffleboard;

    private ComplexWidget complexWidgetCam;
    private ComplexWidget complexWidgetAuton;
    private SendableChooser<AutonomousMode> autonChooser = new SendableChooser<>();
    private NetworkTableEntry maxCapacityBox;
    private ShuffleboardLayout colorSpinnerGrid;
    private NetworkTableEntry colorGridRed;
    private NetworkTableEntry colorGridGreen;
    private NetworkTableEntry colorGridYellow;
    private NetworkTableEntry colorGridBlue;

    // Cam
    private UsbCamera m_cameraIntake;
    private UsbCamera m_cameraClimber;
    private int m_currVideoSourceIndex = 0;
    private VideoSink m_videoSink;
    private VideoSource[] m_videoSources;

    /**
     * 
     * Diagnostics Tab
     * 
     */

    private SimpleWidget driveLPos;
    private SimpleWidget driveLVelo;
    private SimpleWidget driveRPos;
    private SimpleWidget driveRVelo;

    private SimpleWidget LMetersPerSec;
    private SimpleWidget LMetersTraveled;
    private SimpleWidget RMetersPerSec;
    private SimpleWidget RMetersTraveled;

    private SimpleWidget shooterLVelo;
    private SimpleWidget shooterRVelo;

    private SimpleWidget heading;
    private SimpleWidget navxAlive;
    private SimpleWidget navxCalibrating;

    private int graphHeight = 2;
    private int graphWidth = 2;

    private Drivebase m_drivebase;
    private Shooter m_shooter;

    public Dashboard(Drivebase drivebase, Shooter shooter) {

        m_drivebase = drivebase;
        m_shooter = shooter;

        constructCompetitionLayout();
        constructDiagnosticsLayout();

        // camStuff();
    }

    public enum AutonomousMode {
        SHOOTMOVE("ShootMove"), MOVE("Move"), SHOOT("Shoot"), NOTHING("Nothing");

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

        m_videoSources = new VideoSource[] { m_cameraIntake, m_cameraClimber };

        // m_videoSources = new VideoSource[] {
        // m_cameraIntake,
        // m_cameraClimber
        // };

        m_videoSink = CameraServer.getInstance().getServer();

        if (m_cameraIntake != null) {
            m_videoSink.setSource(m_cameraIntake);
        }

    }

    public void constructCompetitionLayout() {
        shuffleboard = Shuffleboard.getTab("Competition");

        /*
         * complexWidgetCam = shuffleboard.add("Cams",
         * m_videoSink.getSource()).withWidget(BuiltInWidgets.kCameraStream)
         * .withSize(camHeight, camWidth).withPosition(camColumnIndex, camRowIndex)
         * .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
         */

        // complexWidgetCam2 = shuffleboard.add( "LimeLight", m_limeLight)
        // .withWidget(BuiltInWidgets.kCameraStream)
        // .withSize(cam2Height, cam2Width)
        // .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

        for (Dashboard.AutonomousMode o : AutonomousMode.values()) {
            autonChooser.addOption(o.getName(), o);
        }
        autonChooser.setDefaultOption(AutonomousMode.DEFAULT.getName(), AutonomousMode.DEFAULT);

        complexWidgetAuton = shuffleboard.add("AutonChooser", autonChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser).withSize(autonChooserWidth, autonChooserHeight)
                .withPosition(autonChooserColumnIndex, autonChooserRowIndex);

        colorSpinnerGrid = shuffleboard.getLayout("Color Spinner", BuiltInLayouts.kGrid)
                .withSize(colorSpinnerGridWidth, colorSpinnerGridHeight)
                .withPosition(colorSpinnerGridColumnIndex, colorSpinnerGridRowIndex)
                .withProperties(Map.of("Number of columns", 1, "Number of Rows", 4));

        colorGridBlue = colorSpinnerGrid.add("Blue", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "blue", "colorWhenFalse", "grey")).getEntry();

        colorGridYellow = colorSpinnerGrid.add("Yellow", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "yellow", "colorWhenFalse", "grey")).getEntry();

        colorGridRed = colorSpinnerGrid.add("Red", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "grey")).getEntry();

        colorGridGreen = colorSpinnerGrid.add("Green", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "grey")).getEntry();

        shuffleboard.add("DesiredColor | FMSColor", "No Game Message yet").withWidget(BuiltInWidgets.kTextView)
                .withSize(desiredColorWidth, desiredColorHeight)
                .withPosition(desiredColorColumnIndex, desiredColorRowIndex).getEntry();
    }

    public void constructDiagnosticsLayout() {
        shuffleboard = Shuffleboard.getTab("Diagnostics");

        driveLPos = shuffleboard.add("DriveLPos", m_drivebase.getLPosTicks()).withWidget(BuiltInWidgets.kGraph)
                .withSize(graphWidth, graphHeight).withPosition(6, 2);
        driveLVelo = shuffleboard.add("DriveLVelo", m_drivebase.getLVeloTicks()).withWidget(BuiltInWidgets.kGraph)
                .withSize(graphWidth, graphHeight).withPosition(6, 3);
        driveRPos = shuffleboard.add("DriveRPos", m_drivebase.getRPosTicks()).withWidget(BuiltInWidgets.kGraph)
                .withSize(graphWidth, graphHeight).withPosition(7, 2);
        driveRVelo = shuffleboard.add("DriveRVelo", m_drivebase.getRVeloTicks()).withWidget(BuiltInWidgets.kGraph)
                .withSize(graphWidth, graphHeight).withPosition(7, 3);

        LMetersPerSec = shuffleboard.add("LMetersPerSec", m_drivebase.getLeftMetersPerSec())
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(0, 0);
        LMetersTraveled = shuffleboard.add("LMetersTraveled", m_drivebase.getLeftMetersTraveled())
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(0, 2);
        RMetersPerSec = shuffleboard.add("RMetersPerSec", m_drivebase.getRightMetersPerSec())
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(2, 0);
        RMetersTraveled = shuffleboard.add("RMetersTraveled", m_drivebase.getRightMetersTraveled())
                .withWidget(BuiltInWidgets.kTextView).withSize(2, 1).withPosition(2, 2);

        heading = shuffleboard.add("Heading", m_drivebase.getHeadingDegrees()).withWidget(BuiltInWidgets.kGyro)
                .withSize(graphWidth, graphHeight).withPosition(4, 0);
        navxAlive = shuffleboard.add("Navx Alive", m_drivebase.navxAlive()).withWidget(BuiltInWidgets.kBooleanBox)
                .withSize(2, 1).withPosition(4, 2);
        navxCalibrating = shuffleboard.add("Navx Calibrating", m_drivebase.navxCalibrating())
                .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1).withPosition(4, 4);

        shooterLVelo = shuffleboard.add("ShooterLVelo", m_shooter.getRightVeloTicks()).withWidget(BuiltInWidgets.kGraph)
                .withSize(graphWidth, graphHeight).withPosition(6, 0);
        shooterRVelo = shuffleboard.add("ShooterRVelo", m_shooter.getRightVeloTicks()).withWidget(BuiltInWidgets.kGraph)
                .withSize(graphWidth, graphHeight).withPosition(8, 0);
    }

    public void switchVideoSource() {
        m_currVideoSourceIndex = (m_currVideoSourceIndex + 1) % m_videoSources.length;
        if (m_videoSources[m_currVideoSourceIndex] != null) {
            m_videoSink.setSource(m_videoSources[m_currVideoSourceIndex]);
        }
    }

    public String getFMSColor() {
        String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
        if (gameMessage.length() > 0) {
            switch (gameMessage.charAt(0)) {
            case 'R':
                // desiredColorDisplay.setString( "Red" );
                // break;
                return "Red";
            case 'B':
                // desiredColorDisplay.setString( "Blue" );
                // break;
                return "Blue";
            case 'G':
                // desiredColorDisplay.setString( "Green" );
                // break;
                return "Green";
            case 'Y':
                // desiredColorDisplay.setString( "Yellow" );
                // break;
                return "Yellow";
            default:
                // desiredColorDisplay.setString( "No Game Data" );
                return "DataNotKnown";
            }
        }
        return "NoData";
    }

    public void setMaxCapacity(boolean isFull) {
        maxCapacityBox.setBoolean(isFull);
    }

    public void setRed(boolean colorIsPresent) {
        colorGridRed.setBoolean(colorIsPresent);
    }

    public void setBlue(boolean colorIsPresent) {
        colorGridBlue.setBoolean(colorIsPresent);
    }

    public void setYellow(boolean colorIsPresent) {
        colorGridYellow.setBoolean(colorIsPresent);
    }

    public void setGreen(boolean colorIsPresent) {
        colorGridGreen.setBoolean(colorIsPresent);
    }

    public AutonomousMode getSelectedObjective() {
        return autonChooser.getSelected();
    }
}