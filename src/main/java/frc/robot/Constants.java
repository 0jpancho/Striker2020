/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static class DriveConstants {

        // inches
        public static final double kWheelDiameter = 6;
        public static final double kWheelCircumferenceInches = Math.PI * kWheelDiameter;
        public static final double kWheelCircumferenceMeters = Units.inchesToMeters(kWheelCircumferenceInches);

        public static final double kTrackWidth = 0.51; // meters
        public final static double kWheelRadius = Units.inchesToMeters(kWheelDiameter); // meters
        public static final int kEncoderResolution = 4096;

        public static final double kInchesPerCount = (kWheelDiameter * Math.PI) / kEncoderResolution;

        public static final int kLeftMasterID = 10;
        public static final int kLeftFollowerID = 11;

        public static final int kRightMasterID = 12;
        public static final int kRightFollowerID = 13;

        public static final double kMaxSpeed = 3.956304; // 12.98 ft/s to m/s (AndyMark 10.71:1 Toughbox Mini)
        public static final double kMaxAngularSpeed = kMaxSpeed / kTrackWidth; // 1 rad/s (test)
    }

    public final static class GamepadVals{
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
