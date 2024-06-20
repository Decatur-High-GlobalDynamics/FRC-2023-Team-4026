// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Ports.SwervePorts;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        // auto distances, need to adjust for swerve and get rid of unused ones
        public static final double EXIT_COMMUNITY_DISTANCE = 5;
        public static final double EXIT_COMMUNITY_CHARGE_STATION_DISTANCE = 5;
        public static final double RETURN_TO_CHARGE_STATION_DISTANCE = -1.5;

        public static final double ELEVATOR_MAX_POWER_CHANGE = 0.275;

        public static final double ELEVATOR_DEADBAND_VALUE = .6;
        public static final double maxElevatorMotorSpeed = 0.3; // Normally .2

        // elevator positions
        public static final double MINIMUM_ELEVATOR_POSITION = 2.2;
        public static final double carryElevatorPos = 4.87;
        public static final double bottomElevatorTargetPosition = 21.30;
        public static final double middleElevatorTargetPosition = 31.91;
        public static final double topElevatorTargetPosition = 45.48;
        public static final double substationPickupElevatorTargetPosition = 30;

        public static final double clawCloseThreshold = 11.6;

        public static final double ELEVATOR_POWER_EXPONENT = 3;

        public static final double JOYSTICK_DEADBAND = 0.1;

        public static final class CANSparkMaxPeriodicRates {
                // In ms
                public static final int LOW_INTERVAL = 30; // Originally 20
                public static final int MID_INTERVAL = 100; // Originally 50
                public static final int HIGH_INTERVAL = 1000; // Originally 500
        }

        public static final class Swerve {
                public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

                /* Drivetrain Constants */
                public static final double trackWidth = Units.inchesToMeters(21.708); // will test
                                                                                      // when
                                                                                      // testing PID
                public static final double wheelBase = Units.inchesToMeters(27.1); // will test when
                                                                                   // testing
                                                                                   // PID
                public static final double wheelDiameter = Units.inchesToMeters(4.0);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                /*
                 * Swerve Kinematics No need to ever change this unless you are not doing a
                 * traditional rectangular/square 4 module swerve
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

                /* Swerve Voltage Compensation */
                public static final double voltageComp = 12.0; // need to confirm this

                /* Module Gear Ratios */
                public static final double driveGearRatio = 6.75;
                public static final double angleGearRatio = 150.0 / 7.0;

                /* Motor Inverts */
                public static final boolean angleMotorInvert = true; // check when testing
                public static final boolean driveMotorInvert = false;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 25;
                public static final int anglePeakCurrentLimit = 40;
                public static final double anglePeakCurrentDuration = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveContinuousCurrentLimit = 35;
                public static final int drivePeakCurrentLimit = 60;
                public static final double drivePeakCurrentDuration = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /*
                 * These values are used by the drive falcon to ramp in open loop and closed
                 * loop
                 * driving. We found a small open loop ramp (0.25) helps with tread wear,
                 * tipping,
                 * etc
                 */
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                /* Angle Motor PID Values */
                public static final double angleKP = 0.025; // must find/double check all of these
                                                            // in sysid
                public static final double angleKI = 0.0;
                public static final double angleKD = 0.0;
                public static final double angleKF = 0.0;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.05; // TODO: This must be tuned to specific
                                                           // robot
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                /*
                 * Drive Motor Characterization Values Divide SYSID values by 12 to convert from
                 * volts to percent output for CTRE
                 */
                public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to
                                                                  // specific
                                                                  // robot
                public static final double driveKV = (1.51 / 12);
                public static final double driveKA = (0.27 / 12);

                /* Angle Motor Conversion Factors */
                public static final double angleConversionFactor = 360.0 / angleGearRatio;

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static final double BASE_DRIVE_SPEED = 0.5, BASE_TURN_SPEED = 1.5;
                public static final double SLOW_SPEED = 0.5;
                public static final double AUTO_SPEED = 0.85;
                public static final double NORMAL_SPEED = 1;
                public static final double FAST_SPEED = 1.5;
                public static final double maxSpeed = 0.7; // 4.5; //TODO: This must be tuned to
                                                           // specific
                                                           // robot
                /** Radians per Second */
                public static final double maxAngularVelocity = 2.25; // 10.0; //TODO: This must be
                                                                      // tuned to
                                                                      // specific robot

                /* Neutral Modes */
                public static final IdleMode angleNeutralMode = IdleMode.kCoast;
                public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

                public static final boolean DEFAULT_ANGLE_INVERT = false;

                public static final double[] ANGLE_OFFSETS = new double[] {
                                114.3, // FL
                                80.2, // FR
                                61.4, // RL
                                313.4 // RR
                };

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = SwervePorts.MOD0_DRIVEMOTOR;
                        public static final int angleMotorID = SwervePorts.MOD0_ANGLEMOTOR;
                        public static final int canCoderID = SwervePorts.MOD0_CANCODER;
                        public static Rotation2d angleOffset = Rotation2d.fromDegrees(
                                        ANGLE_OFFSETS[0] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                                        driveMotorID, angleMotorID, canCoderID, angleOffset);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = SwervePorts.MOD1_DRIVEMOTOR;
                        public static final int angleMotorID = SwervePorts.MOD1_ANGLEMOTOR;
                        public static final int canCoderID = SwervePorts.MOD1_CANCODER;
                        public static Rotation2d angleOffset = Rotation2d.fromDegrees(
                                        ANGLE_OFFSETS[1] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                                        driveMotorID, angleMotorID, canCoderID, angleOffset);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = SwervePorts.MOD2_DRIVEMOTOR;
                        public static final int angleMotorID = SwervePorts.MOD2_ANGLEMOTOR;
                        public static final int canCoderID = SwervePorts.MOD2_CANCODER;
                        public static Rotation2d angleOffset = Rotation2d.fromDegrees(
                                        ANGLE_OFFSETS[2] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                                        driveMotorID, angleMotorID, canCoderID, angleOffset);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 { // TODO: This must be tuned to specific robot
                        public static final int driveMotorID = SwervePorts.MOD3_DRIVEMOTOR;
                        public static final int angleMotorID = SwervePorts.MOD3_ANGLEMOTOR;
                        public static final int canCoderID = SwervePorts.MOD3_CANCODER;
                        public static Rotation2d angleOffset = Rotation2d.fromDegrees(
                                        ANGLE_OFFSETS[3] - (DEFAULT_ANGLE_INVERT ? 180 : 0));;
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                                        driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
        }

        public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                                  // tuned to specific robot
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;

                /* Constraint for the motion profilied robot angle controller */
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond,
                                kMaxAngularSpeedRadiansPerSecondSquared);
        }

}
