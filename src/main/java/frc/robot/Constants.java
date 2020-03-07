/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.util.Gains;

public abstract class Constants {
    public final static class Drive {

        public static final int kLeftMasterID = 10;
        public static final int kLeftFollowerID = 11;

        public static final int kRightMasterID = 12;
        public static final int kRightFollowerID = 13;

        // inches
        public static final double kRadiusInches = 3;
        public static final double kDiameterInches = 6;

        public static final double kCircumferenceInches = Math.PI * kDiameterInches;
        public static final double kCircumferenceMeters = Units.inchesToMeters(kCircumferenceInches);

        public static final double kTrackWidth = Units.inchesToMeters(19.5); // meters
        public final static double kRadiusMeters = Units.inchesToMeters(kRadiusInches); // meters

        // SRX Mag Encoder
        public static final int kEncoderResolution = 4096;

        public static final double kInchesPerCount = kCircumferenceInches / kEncoderResolution;
        public static final double kMetersPerCount = kCircumferenceMeters / kEncoderResolution;

        public static final double kRawMaxSpeed = 3.956304; // 12.98 ft/s to m/s (AndyMark 10.71:1 Toughbox Mini w/ 6in wheels)
        public static final double kRawAngularSpeed = 2 * Math.PI;
        public static final double kForwardAdjust = 1;
        public static final double kRotAdjust = .8;

        public static final double kAdjustedMaxSpeed = kRawMaxSpeed * kForwardAdjust;
        public static final double kAdjustedAngularSpeed = kRawAngularSpeed * kRotAdjust;

        //public static final Gains kVeloGains = new Gains(0.0303, 0, 0.35, 0, 0, 1.00);
        public static final Gains kVeloGains = new Gains(0.6, 0, 0, 0, 0, 1.00);

        public static final Gains kPosGains = new Gains(0.4, 0, 0, 0, 0, 1.00);
        
        public static final Gains kTurnGains = new Gains(1, 0, 0, 0, 0, 1.00);

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 20;
    }

    public static final class Shooter {

        public static final int kLShooterID = 21;
        public static final int kRShooterID = 22;

        public static final int kRadiusInches = 2;
        public static final int kDiameterInches = 4;

        public static final double kCircumferenceInches = Math.PI * kDiameterInches;

        // Hi-Res CIMcoder
        public static final int kEncoderResolution = 256;

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 20;
        // kP kI kD kF Iz PeakOut
        public static final Gains kGains = new Gains(1.47, 0, 0, 0, 0, 1.00); // CHANGE TO 0.6 TO TEST AFTER QUAL 61

        public static final double kRPM = 200;
    }

    public final static class Indexer {

        public static final int kMasterID = 31;
        public static final int kSlaveID = 32;

        // SRX Mag Encoder
        public static final int kEncoderResolution = 4096;

        public static final double kRadiusInches = 1.5;
        public static final double kDiameterInches = 3;

        public static final double kCircumferenceInches = Math.PI * kDiameterInches;

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 20;

        public static final double kPower = 0.75;
    }

    public final static class Intake {
        public static final int kIntakeID = 0;

        // Increased power from .25 to .5 to compensate for balls going under the robot
        public static final double kPower = -0.5;
    }

    public final static class Climber {
        public static final int kWinchID = 1;
        public static final int kLiftID = 2;

        public static final double kWinchPower = -1;
        public static final double kLiftPower = 1;
    }
 
    // Logitech F310
    public final static class GamepadIDs {

        // Gamepad axis
        public static final int kGamepadAxisLeftStickX = 1;
        public static final int kGamepadAxisLeftStickY = 2;
        public static final int kGamepadAxisShoulder = 3;
        public static final int kGamepadAxisRightStickX = 4;
        public static final int kGamepadAxisRightStickY = 5;
        public static final int kGamepadAxisDpad = 6;

        // Gamepad buttons
        public static final int kGamepadButtonA = 1; // Bottom Button
        public static final int kGamepadButtonB = 2; // Right Button
        public static final int kGamepadButtonX = 3; // Left Button
        public static final int kGamepadButtonY = 4; // Top Button
        public static final int kGamepadButtonShoulderL = 5;
        public static final int kGamepadButtonShoulderR = 6;
        public static final int kGamepadButtonBack = 7;
        public static final int kGamepadButtonStart = 8;
        public static final int kGamepadButtonLeftStick = 9;
        public static final int kGamepadButtonRightStick = 10;
        public static final int kGamepadButtonMode = -1;
        public static final int kGamepadButtonLogitech = -1;
    }

    public static final class Limelight {
        
        public static final double kHeightInches = 18; 
        public static final double kAngleDegrees = 33.69; 

        public static final double kTargetHeight = 98.25; // inches to center hexagon
    }
}
