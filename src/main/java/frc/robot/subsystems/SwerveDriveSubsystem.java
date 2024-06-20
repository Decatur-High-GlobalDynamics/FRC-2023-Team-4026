package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Ports;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase
{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    // public AHRS gyro; // AHRS is the NavX class
    public Pigeon2 gyro;
    public double maxModuleSpeed = Constants.Swerve.maxSpeed;

    public double gyroOffset = 0;

    public SwerveDriveSubsystem()
    {
        // gyro = new AHRS(Port.kMXP); // need to check this. port for gyro
        gyro = new Pigeon2(Ports.GYRO);
        // pls check ahrs gyro setup EDIT: gyro should automatically calibrate once
        // powered on
        gyro.configFactoryDefault();
        zeroGyro();

        setAngleOffsets(false);

        // swerve module construction
        mSwerveMods = new SwerveModule[]
        {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug with inverting
         * motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        // construct odometry (full robot position/incorporated module states)
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(),
                getModulePositions());
    }

    public void setAngleOffsets(boolean invert)
    {
        System.out.println("Setting angle offsets...");

        double[] offsets = Constants.Swerve.ANGLE_OFFSETS;

        Constants.Swerve.Mod0.angleOffset = Rotation2d.fromDegrees(offsets[0] - (invert ? 180 : 0));
        Constants.Swerve.Mod1.angleOffset = Rotation2d.fromDegrees(offsets[1] - (invert ? 180 : 0));
        Constants.Swerve.Mod2.angleOffset = Rotation2d.fromDegrees(offsets[2] - (invert ? 180 : 0));
        Constants.Swerve.Mod3.angleOffset = Rotation2d.fromDegrees(offsets[3] - (invert ? 180 : 0));
    }

    // main driving method. translation is change in every direction
    public void drive(Translation2d translation, double rotation, boolean fieldRelative,
            boolean isOpenLoop)
    {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),
                                translation.getY(), rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        // Log module state
        for (int i = 0; i < swerveModuleStates.length; i++)
        {
            SwerveModuleState mod = swerveModuleStates[i];
            SmartDashboard.putNumber("Mod " + i + " Target Angle", mod.angle.getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Target - CANCoder",
                    mod.angle.getDegrees() - mSwerveMods[i].getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Target Speed", mod.speedMetersPerSecond);
        }

        // lowers module speeds to max attainable speed (avoids going above topspeed)
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxModuleSpeed);

        // sets modules to desired state (angle, speed)
        for (SwerveModule mod : mSwerveMods)
        {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxModuleSpeed);

        for (SwerveModule mod : mSwerveMods)
        {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    // gets position of robot on the field (odometry)
    public Pose2d getPose()
    {
        return swerveOdometry.getPoseMeters();
    }

    // resets odometry (position on field)
    public void resetOdometry(Pose2d pose)
    {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    // returns array of a modules' states (angle, speed) for each one
    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods)
        {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    // returns module positions(for each individual module)
    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods)
        {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void setGyro(double degrees)
    {
        System.out.println("Setting gyro to " + degrees + "...");
        gyroOffset = degrees - gyro.getYaw();
        zeroGyro();
    }

    public void zeroGyro()
    {
        System.out.println("Zeroing gyro");

        gyro.setYaw(0); // Used to setYaw(0);
    }

    public Rotation2d getYaw()
    {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getYaw() + gyroOffset)
                : Rotation2d.fromDegrees(gyro.getYaw() + gyroOffset);
    }

    public void resetModulesToAbsolute()
    {
        for (SwerveModule mod : mSwerveMods)
        {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic()
    {
        swerveOdometry.update(getYaw(), getModulePositions());

        // smartdashboard logging per module
        for (SwerveModule mod : mSwerveMods)
        {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
                    mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                    mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                    mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());
    }

    public void resetEncoders()
    {
        for (SwerveModule mod : mSwerveMods)
        {
            mod.resetToAbsolute();
        }
    }
}