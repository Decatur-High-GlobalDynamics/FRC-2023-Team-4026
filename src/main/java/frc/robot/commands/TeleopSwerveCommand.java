package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerveCommand extends CommandBase {
        private SwerveDriveSubsystem s_Swerve;
        private DoubleSupplier translationSup;
        private DoubleSupplier strafeSup;
        private DoubleSupplier rotationSup;
        private BooleanSupplier robotCentricSup, slowSpeedSupplier, fastSpeedSupplier; // used for field relative

        public TeleopSwerveCommand(SwerveDriveSubsystem s_Swerve, DoubleSupplier translationSup,
                        DoubleSupplier strafeSup,
                        DoubleSupplier rotationSup,
                        BooleanSupplier slowSpeedSupplier,
                        BooleanSupplier fastSpeedSupplier /* BooleanSupplier robotCentricSup */) {
                this.s_Swerve = s_Swerve;
                addRequirements(s_Swerve);

                this.translationSup = translationSup;
                this.strafeSup = strafeSup;
                this.rotationSup = rotationSup;
                // this.robotCentricSup = robotCentricSup;

                this.slowSpeedSupplier = slowSpeedSupplier;
                this.fastSpeedSupplier = fastSpeedSupplier;
        }

        private double getSpeed() {
                if (slowSpeedSupplier.getAsBoolean())
                        return Constants.Swerve.SLOW_SPEED;
                if (fastSpeedSupplier.getAsBoolean())
                        return Constants.Swerve.FAST_SPEED;
                return Constants.Swerve.NORMAL_SPEED;
        }

        @Override
        public void execute() {
                /* Get Values, Deadband */
                double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(),
                                Constants.JOYSTICK_DEADBAND);
                double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(),
                                Constants.JOYSTICK_DEADBAND);
                double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(),
                                Constants.JOYSTICK_DEADBAND);

                double speed = getSpeed();
                SmartDashboard.putNumber("Swerve Speed", speed);

                /* Drive */
                s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(
                                speed * Constants.Swerve.BASE_DRIVE_SPEED),
                                rotationVal * speed * Constants.Swerve.BASE_TURN_SPEED,
                                /* !robotCentricSup.getAsBoolean(), */ true, // field relative is always on
                                true);
        }
}