package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final int autonChooserWidth = 3;
    private final int autonChooserHeight = 1;
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

    private int graphHeight = 2;
    private int graphWidth = 2;

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

        // complexWidgetCam2 = shuffleboard.add("LimeLight", m_limeLight)
        // .withWidget(BuiltInWidgets.kCameraStream)
        // .withSize(cam2Height, cam2Width)
        // .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

        for (Dashboard.AutonomousMode o : AutonomousMode.values()) {
            autonChooser.addOption(o.getName(), o);
        }
        autonChooser.setDefaultOption(AutonomousMode.DEFAULT.getName(), AutonomousMode.DEFAULT);

        complexWidgetAuton = shuffleboard.add("AutonChooser", autonChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser).withSize(3, 1)
                .withPosition(0, 0);

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

        shooterLVelo = shuffleboard.addNumber("ShooterLVelo", shooterLVeloSup).withWidget(BuiltInWidgets.kTextView)
                .withSize(1, 1).withPosition(5, 0);
        shooterRVelo = shuffleboard.addNumber("ShooterRVelo", shooterRVeloSup).withWidget(BuiltInWidgets.kTextView)
                .withSize(1, 1).withPosition(6, 0);
        
        /*
        driveLPos = shuffleboard.addNumber("DriveLPos", driveLPosSup);
        driveLVelo = shuffleboard.addNumber("DriveLVelo", driveLVeloSup);
        driveRPos = shuffleboard.addNumber("DriveRPos", driveRPosSup);
        driveRVelo = shuffleboard.addNumber("DriveRVelo", driveRVeloSup);

        LMetersPerSec = shuffleboard.addNumber("LMetersPerSec", LMetersPerSecSup);
        LMetersTraveled = shuffleboard.addNumber("LMetersTraveled", LMetersTraveledSup);
        RMetersPerSec = shuffleboard.addNumber("RMetersPerSec", RMetersPerSecSup);
        RMetersTraveled = shuffleboard.addNumber("RMetersTraveled", RMetersTraveledSup);

        heading = shuffleboard.addNumber("Heading", headingSup);
        navxAlive = shuffleboard.addBoolean("Navx Alive", navxAliveSup);
        navxCalibrating = shuffleboard.addBoolean("Navx Calibrating", navxCalibratingSup);

        shooterLVelo = shuffleboard.addNumber("ShooterLVelo", shooterLVeloSup);
        shooterRVelo = shuffleboard.addNumber("ShooterRVelo", shooterRVeloSup);
        */
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