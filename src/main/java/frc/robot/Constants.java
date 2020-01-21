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

    public final class DriveConstants{

        // inches
        public static final double kWheelDiameter = 6;

        public static final double kTrackWidth = 0.51; // meters
        public final double kWheelRadius = Units.inchesToMeters(kWheelDiameter); // meters
        public static final int kEncoderResolution = 4096;

        public static final double kCountsPerInch = (kWheelDiameter * Math.PI) / kEncoderResolution;

        public static final int kLeftMasterID = 10;
        public static final int kLeftFollowerID = 11;

        public static final int kRightMasterID = 12;
        public static final int kRightFollowerID = 13;

        public static final double kMaxSpeed = 3.956304;  //12.98 ft/s to m/s (AndyMark 10.71:1 Toughbox Mini)
        public static final double kMaxAngularSpeed = 2 * Math.PI;  //1 rad/s (test)
    }
}
