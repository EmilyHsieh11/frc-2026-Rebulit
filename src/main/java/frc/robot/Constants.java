package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.subsystems.vision.ApriltagTarget;
import static edu.wpi.first.units.Units.Inches;

public final class Constants {

    public static class FieldConstants {
        public static final double inchesToMeters = 0.0254;

        public static final double kAllianceZoneLength = 651.22 * inchesToMeters; // 16.54m
        public static final double kAllianceZoneWidth = 317.69 * inchesToMeters;
        public static final Translation2d Red_Hub_Absolute_Position = new Translation2d(182.11*inchesToMeters, 158.84*inchesToMeters);
        public static final Translation2d Blue_Hub_Absolute_Position = new Translation2d((651.22-182.11)*inchesToMeters, 158.84*inchesToMeters);
    }

    public static class ShooterConstants {
        public static final double shooterMotorSpeed = -4.5;
        public static final double hoodBiggestAngle = 16;
        public static final double closestAngle = 0.5;
        public static final double shooterVoltage = 10;
        public static final double indexerVoltage = -8;

        public static final int khoodMotorId = 43;
        public static final int kshooterMotorId = 44;
        public static final int kindexerMotorId = 58;
        public static final double gearRatio = 20.0 / 360.0 / 3.0;
        public static final double littlePinkGearDiameter = 25; // mm
        public static final double littlePinkGearTeeth = 20; // 齒數
        public static final double sectorRadius = 253.9; // mm 
        public static final double kG = 0.0;

        public static final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        static{
            table.put(1.0, 10.0); // vision_distance, hood_angle_degree
            table.put(2.0, 20.0);
            table.put(3.0, 35.0);
        }
    }

    public static final class IntakeConstants {
        public static final int RollerId = 6;
        public static final int PivotId = 61;

        public static final double pivotPosition = 0;
        public static final double rollervoltage = 0;
        public static final double intakekp = 30.0, intakeki = 0, intakekd = 0, intakekv = 2.5, intakekg = 0.3, intakeks = 0;
        
        public static final double kIntakeVolts = -3.0;
        public static final double kpivotLowered = 10.23;
        public static final double kpivotRaised = 3.58;
 
    }
    
    public static final class VisionConstants {

        public static final Transform3d ROBOT_TO_CAMERA_Steve = new Transform3d(
            new Translation3d(
                0.275,   // X - 往前（米）← 測量後改這裡
                -0.275,   // Y - 往左（米）← 測量後改這裡
                0.39    // Z - 離地高度（米）← 測量後改這裡
            ),
            new Rotation3d(
                0,                        // Roll
                Math.toRadians(60),      // Pitch（向下角度）← 測量後改這裡
                0                         // Yaw
            )
        );
        
        /* IDs 3,4 and 19,20 are on the side of the hub that we can't shoot from, so don't include them */
        public static final int[] RedHubApriltagIds = new int[]{
            2, /* 3, 4, */ 5, 8, 9, 10, 11
        };
        public static final int[] BlueHubApriltagIds = new int[]{
            18, /* 19, 20, */ 21, 24, 25, 26, 27
        };

        public static final ApriltagTarget RedHub = new ApriltagTarget(Map.of(
            2, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            5, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            8, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
            9, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
            10, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            11, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
        ));

        public static final ApriltagTarget BlueHub = new ApriltagTarget(Map.of(
            18, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            21, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            24, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
            25, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
            26, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
            27, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
        ));

    }
}