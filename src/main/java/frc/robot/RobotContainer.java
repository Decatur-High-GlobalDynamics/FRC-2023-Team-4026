// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SetElevatorTargetCommand;
import frc.robot.commands.SetElevatorTargetOverrideCommand;
import frc.robot.autos.AutoBalanceAuto;
import frc.robot.autos.DriveDistanceAuto;
import frc.robot.autos.GoodDriveDistanceAuto;
import frc.robot.commands.ClawGrabberCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetClawThresholdOverrideCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClawIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        public static RobotContainer instance;

        public SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem();
        public ClawIntakeSubsystem clawIntake = new ClawIntakeSubsystem();
        public ElevatorSubsystem elevator = new ElevatorSubsystem(clawIntake);;
        public VisionSubsystem vision = new VisionSubsystem();

        public Joystick primaryController;
        public Joystick secondaryController;

        public static ShuffleboardTab shuffleboard = Shuffleboard.getTab("SmartDashboard");

        // The robot's subsystems and commands are defined here...
        private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

        private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

        SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // shuffleboard = Shuffleboard.getTab("SmartDashboard");

                SmartDashboard.putBoolean("Invert Swerve", false);

                swerveDrive.setAngleOffsets(SmartDashboard.getBoolean("Invert Swerve",
                                Constants.Swerve.DEFAULT_ANGLE_INVERT));

                shuffleboard.addDouble("Gyro", () -> swerveDrive.gyro.getYaw());

                autoChooser = new SendableChooser<>();

                autoChooser.setDefaultOption("Charge Station", chargeStationAuto);
                autoChooser.addOption("Exit Community", exitCommunityAuto);

                shuffleboard.addDouble("Yaw Gyro", () -> swerveDrive.gyro.getYaw());
                shuffleboard.addDouble("Pitch Gyro", () -> swerveDrive.gyro.getRoll());

                shuffleboard.addDouble("Gyro Offset", () -> swerveDrive.gyroOffset);

                shuffleboard.add(autoChooser);

                // shuffleboard.add("Get Angle Offsets", new InstantCommand(() -> {
                //         System.out.println("Getting angle offsets...");

                //         for (int i = 0; i < swerveDrive.mSwerveMods.length; i++) {
                //                 double degrees = swerveDrive.mSwerveMods[i].getCanCoder()
                //                                 .getDegrees();
                //                 System.out.println("Setting mod " + i + " to " + degrees + "...");
                //                 Constants.Swerve.ANGLE_OFFSETS[i] = degrees;
                //         }

                //         swerveDrive.setAngleOffsets(false);
                // }));

                secondaryController = new Joystick(2); // We need this for testing in elevator

                // Configure the button bindings
                configurePrimaryBindings();
                configureSecondaryBindings();

                instance = this;

                // shuffleboard.addBoolean("NavX Calibrate", () ->
                // swerveDrive.yawGyro.isCalibrating());
                // shuffleboard.addBoolean("NavX Connected", () ->
                // swerveDrive.yawGyro.isConnected());
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
         * passing it
         * to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configurePrimaryBindings() {
                primaryController = new Joystick(0);

                JoystickButton a = new JoystickButton(primaryController,
                                LogitechControllerButtons.a);
                JoystickButton b = new JoystickButton(primaryController,
                                LogitechControllerButtons.b);
                JoystickButton x = new JoystickButton(primaryController,
                                LogitechControllerButtons.x);
                JoystickButton y = new JoystickButton(primaryController,
                                LogitechControllerButtons.y);
                JoystickButton bumperLeft = new JoystickButton(primaryController,
                                LogitechControllerButtons.bumperLeft);
                JoystickButton bumperRight = new JoystickButton(primaryController,
                                LogitechControllerButtons.bumperRight);
                JoystickButton triggerLeft = new JoystickButton(primaryController,
                                LogitechControllerButtons.triggerLeft);
                JoystickButton triggerRight = new JoystickButton(primaryController,
                                LogitechControllerButtons.triggerRight);
                // JoystickButton up = new
                // JoystickButton(primaryController,LogitechControllerButtons.up);
                // JoystickButton down = new
                // JoystickButton(primaryController,LogitechControllerButtons.down);
                // JoystickButton left = new
                // JoystickButton(primaryController,LogitechControllerButtons.left);
                // JoystickButton right = new
                // JoystickButton(primaryController,LogitechControllerButtons.right);

                swerveDrive
                                // vertical axis of left joystick -> translation
                                .setDefaultCommand(new TeleopSwerveCommand(swerveDrive,
                                                () -> -primaryController.getY(),
                                                () -> -primaryController.getX(), // horizontal
                                                                                 // axis
                                                                                 // of
                                                                                 // left
                                                                                 // joystick->
                                                                                 // strafe
                                                // we want horizontal of right stick, if not twist,
                                                // then change
                                                () -> primaryController.getTwist(),
                                                () -> triggerLeft.getAsBoolean(),
                                                () -> triggerRight.getAsBoolean()
                                /*
                                 * ()->y.getAsBoolean() owen doesn't want a button to turn
                                 * field-relative off
                                 */
                                ));

                x.onTrue(new InstantCommand(() -> {
                        swerveDrive.zeroGyro();
                        swerveDrive.gyroOffset = 0;
                }));
                y.onTrue(new InstantCommand(() -> {
                        for (SwerveModule mod : swerveDrive.mSwerveMods)
                                mod.resetToAbsolute();
                }));

                a.onTrue(new InstantCommand(() -> {
                        boolean invert = SmartDashboard.getBoolean("Invert Serve", false);
                        invert = !invert;
                        swerveDrive.setAngleOffsets(invert);
                        SmartDashboard.putBoolean("Invert Swerve", invert);
                }));
                b.onTrue(new InstantCommand(() -> {
                        swerveDrive.setGyro(180);
                }));
        }

        private void configureSecondaryBindings() {
                secondaryController = new Joystick(1);

                elevator.setDefaultCommand(new MoveElevatorCommand(
                                () -> -secondaryController.getY(), elevator));

                JoystickButton a = new JoystickButton(secondaryController,
                                LogitechControllerButtons.a);
                JoystickButton b = new JoystickButton(secondaryController,
                                LogitechControllerButtons.b);
                JoystickButton x = new JoystickButton(secondaryController,
                                LogitechControllerButtons.x);
                JoystickButton y = new JoystickButton(secondaryController,
                                LogitechControllerButtons.y);
                JoystickButton bumperLeft = new JoystickButton(secondaryController,
                                LogitechControllerButtons.bumperLeft);
                JoystickButton bumperRight = new JoystickButton(secondaryController,
                                LogitechControllerButtons.bumperRight);
                JoystickButton triggerLeft = new JoystickButton(secondaryController,
                                LogitechControllerButtons.triggerLeft);
                JoystickButton triggerRight = new JoystickButton(secondaryController,
                                LogitechControllerButtons.triggerRight);
                POVButton up = new POVButton(secondaryController, LogitechControllerButtons.up);
                POVButton down = new POVButton(secondaryController, LogitechControllerButtons.down);
                POVButton left = new POVButton(secondaryController, LogitechControllerButtons.left);
                ;
                POVButton right = new POVButton(secondaryController,
                                LogitechControllerButtons.right);
                JoystickButton start = new JoystickButton(secondaryController,
                                LogitechControllerButtons.start);

                a.whileTrue(new ClawGrabberCommand(Value.kForward, clawIntake, true));
                b.whileTrue(new ClawGrabberCommand(Value.kReverse, clawIntake, false));
                // new IntakeMotorCommand( () -> bumperRight.getAsBoolean(),clawIntake);

                up.onTrue(new SetElevatorTargetCommand(Constants.topElevatorTargetPosition,
                                elevator));
                left.onTrue(new SetElevatorTargetCommand(Constants.carryElevatorPos, elevator));
                right.onTrue(new SetElevatorTargetCommand(Constants.middleElevatorTargetPosition,
                                elevator));
                down.onTrue(new SetElevatorTargetCommand(Constants.bottomElevatorTargetPosition,
                                elevator));
                bumperLeft.onTrue(new SetElevatorTargetCommand(
                                Constants.substationPickupElevatorTargetPosition, elevator));

                bumperRight.onTrue(new SetClawThresholdOverrideCommand(true, elevator));
                bumperRight.onFalse(new SetClawThresholdOverrideCommand(false, elevator));

                triggerLeft.onTrue(new SetElevatorTargetOverrideCommand(true, elevator));
                triggerLeft.onFalse(new SetElevatorTargetOverrideCommand(false, elevator));

                start.onTrue(new SetElevatorTargetCommand(Constants.MINIMUM_ELEVATOR_POSITION,
                                elevator));

                shuffleboard.addBoolean("Test", () -> Robot.isTest);
                if (Robot.isTest) {
                        x.onTrue(new SetElevatorTargetCommand(
                                        Constants.middleElevatorTargetPosition, true, elevator)
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.carryElevatorPos,
                                                        true, elevator))
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.middleElevatorTargetPosition,
                                                        true, elevator))
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.carryElevatorPos,
                                                        true, elevator))
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.middleElevatorTargetPosition,
                                                        true, elevator))
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.carryElevatorPos,
                                                        true, elevator))
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.middleElevatorTargetPosition,
                                                        true, elevator))
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.carryElevatorPos,
                                                        true, elevator))
                                        .andThen(new SetElevatorTargetCommand(
                                                        Constants.middleElevatorTargetPosition,
                                                        true, elevator)));
                }
        }

        // enum PossibleAutos
        // {
        // NORMAL_AUTO,
        // CHARGE_STATION_AUTO
        // }

        private final Command exitCommunityAuto = new InstantCommand(() -> {
                swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        }).andThen(new ClawGrabberCommand(Value.kReverse, clawIntake, false))
                        .andThen(new SetElevatorTargetCommand(Constants.topElevatorTargetPosition,
                                        true, elevator))
                        .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
                        .andThen(new SetElevatorTargetCommand(Constants.carryElevatorPos, true,
                                        elevator))
                        .andThen(new GoodDriveDistanceAuto(Constants.EXIT_COMMUNITY_DISTANCE, -0.4,
                                        swerveDrive));

        private final Command chargeStationAuto = new InstantCommand(() -> {
                swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        }).andThen(new ClawGrabberCommand(Value.kReverse, clawIntake, false))
                        .andThen(new SetElevatorTargetCommand(Constants.topElevatorTargetPosition,
                                        true, elevator))
                        .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
                        .andThen(new SetElevatorTargetCommand(Constants.carryElevatorPos, true,
                                        elevator))
                        .andThen(new GoodDriveDistanceAuto(
                                        Constants.EXIT_COMMUNITY_CHARGE_STATION_DISTANCE, -0.4,
                                        swerveDrive))
                        .andThen(new GoodDriveDistanceAuto(
                                        Constants.RETURN_TO_CHARGE_STATION_DISTANCE, 0.4,
                                        swerveDrive));
        // .andThen(new AutoBalanceAuto(swerveDrive));

        // // Old autos below

        // private final Command driveBackAuto = new
        // DriveDistanceAuto(Constants.BALANCE_DISTANCE,
        // swerveDrive);

        // private final Command openThenDriveAuto = new
        // ClawGrabberCommand(Value.kForward, clawIntake,
        // true)
        // .andThen(new DriveDistanceAuto(Constants.BALANCE_DISTANCE, swerveDrive));

        // private final Command openClaw = new ClawGrabberCommand(Value.kReverse,
        // clawIntake, true);

        // private final Command overThenBalanceAuto = new DriveDistanceAuto(
        // Constants.OVER_CHARGESTATION_DISTANCE, swerveDrive)
        // .andThen(new DriveDistanceAuto(Constants.RETURN_TO_CHARGESTATION_DISTANCE,
        // swerveDrive));

        // private final Command highBalance = new SetElevatorTargetCommand(
        // Constants.topElevatorTargetPosition, true, elevator)
        // .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
        // .andThen(
        // new SetElevatorTargetCommand(Constants.MINIMUM_ELEVATOR_POSITION, true,
        // elevator))
        // .andThen(new DriveDistanceAuto(Constants.BALANCE_DISTANCE, swerveDrive));

        // private final Command midBalance = new SetElevatorTargetCommand(
        // Constants.middleElevatorTargetPosition, true, elevator)
        // .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
        // .andThen(
        // new SetElevatorTargetCommand(Constants.MINIMUM_ELEVATOR_POSITION, true,
        // elevator))
        // .andThen(new DriveDistanceAuto(Constants.BALANCE_DISTANCE, swerveDrive));

        // private final Command backOut = new DriveDistanceAuto(
        // 50000 * Constants.normalAutoDriveBackDistance, swerveDrive);

        // private final Command lowBalance = new SetElevatorTargetCommand(
        // Constants.bottomElevatorTargetPosition, true, elevator)
        // .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
        // .andThen(
        // new SetElevatorTargetCommand(Constants.MINIMUM_ELEVATOR_POSITION, true,
        // elevator))
        // .andThen(new DriveDistanceAuto(Constants.BALANCE_DISTANCE, swerveDrive));

        // private final Command highBack = new
        // SetElevatorTargetCommand(Constants.topElevatorTargetPosition,
        // true, elevator)
        // .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
        // .andThen(
        // new SetElevatorTargetCommand(Constants.MINIMUM_ELEVATOR_POSITION, true,
        // elevator))
        // .andThen(new DriveDistanceAuto(Constants.normalAutoDriveBackDistance,
        // swerveDrive));

        // private final Command midBack = new SetElevatorTargetCommand(
        // Constants.middleElevatorTargetPosition, true, elevator)
        // .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
        // .andThen(
        // new SetElevatorTargetCommand(Constants.MINIMUM_ELEVATOR_POSITION, true,
        // elevator))
        // .andThen(new DriveDistanceAuto(Constants.normalAutoDriveBackDistance,
        // swerveDrive));

        // private final Command lowBack = new SetElevatorTargetCommand(
        // Constants.bottomElevatorTargetPosition, true, elevator)
        // .andThen(new ClawGrabberCommand(Value.kForward, clawIntake, true))
        // .andThen(
        // new SetElevatorTargetCommand(Constants.MINIMUM_ELEVATOR_POSITION, true,
        // elevator))
        // .andThen(
        // new DriveDistanceAuto(50000 * Constants.normalAutoDriveBackDistance,
        // swerveDrive));

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous
                Command choice = autoChooser.getSelected();
                // choice.schedule();
                return choice;

        }
}