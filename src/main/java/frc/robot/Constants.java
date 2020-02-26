/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

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

        public static final double kMaxSpeed = 3.956304; // 12.98 ft/s to m/s (AndyMark 10.71:1 Toughbox Mini)
        public static final double kMaxAngularSpeed = kMaxSpeed / kTrackWidth;

        public final static Gains kDriveGains = new Gains(0, 0, 0, 0, 0, 1.00);
        public final static Gains kTurnGains = new Gains(1, 0, 0, 0, 0, 1.00);

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 20;
    }

    public final static class Shooter {

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
        public final static Gains kGains = new Gains(0.75, 0, 0, 0, 0, 1.00);
    }

    public final static class Indexer {

        public static final int kMasterID = 31;
        public static final int kSlaveID = 32;

        //  SRX Mag Encoder
        public static final int kEncoderResolution = 4096;

        public static final double kRadiusInches = 1.5;
        public static final double kDiameterInches = 3;

        public static final double kCircumferenceInches = Math.PI * kDiameterInches;

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 20;
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
}
