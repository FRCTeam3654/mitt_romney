// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double turnPIDGyroP = 0.009;
    public static double turnPIDGyroI = 0;
    public static double turnPIDGyroD = 0.0075;
    public static double turnToleranceDeg = 3; 
    public static double turnRateToleranceDegPerS = 24;
//for Ramsite
    public static final class DriveConstants {
        public static final double ksVolts = 0.929;
        public static final double kvVoltSecondsPerMeter = 6.33;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
    
        public static final double kPDriveVel = 0.26; //0.16; //0.085;
    
        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
            
        //modify following 6 constants
        public static final int kEncoderCPR = 1440;
        public static final double encoderTicksPerRev = 1440; // added by Team 3654 michele was here
        public static final double kWheelDiameterMeters = 0.07;
        public static final double kWheelCircumferenceMeter = 0.2198; // pi x 6.0 x 2.54 /100 added by Team 3654
    
        //public static final double kGearing = 10.71  ;   // added by Team 3654
        public static final double gearRatio = 1; // added by Team 3654
        public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * gearRatio / (double) kEncoderCPR;
        
        // Shuffleboard constants
        public static String SBTabDriverDisplay = "Driver Display";

        public static final boolean tuningMode = false;
    }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;
        public static final double kMaxCentripetalAcceleration = 1.0 * kMaxAccelerationMetersPerSecondSquared;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }

}
