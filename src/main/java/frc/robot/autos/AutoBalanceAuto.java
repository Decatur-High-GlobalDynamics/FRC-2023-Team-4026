package frc.robot.autos;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoBalanceAuto extends CommandBase
{
    SwerveDriveSubsystem s_Swerve;

    private double balancingSpeed;
    private int balancingDirection;

    private double tiltDeadband;

    private double wheelLockSpeed;

    private boolean wheelsLocked;

    private Pigeon2 gyro;

    public AutoBalanceAuto(SwerveDriveSubsystem s_Swerve)
    {
        this.s_Swerve = s_Swerve;
        balancingSpeed = 0.25;
        balancingDirection = 1;
        tiltDeadband = 12;
        wheelLockSpeed = 0;

        wheelsLocked = false;

        gyro = s_Swerve.gyro;

        addRequirements(s_Swerve);
    }

    @Override
    public void execute()
    {
        System.out.println("Autobalancing");
        if (gyro.getRoll() < tiltDeadband && gyro.getRoll() > -tiltDeadband)
        {
            balancingDirection = 0;

            if (!wheelsLocked)
            {
                wheelLockSpeed = 0.01;
                wheelsLocked = true;
            }
        }
        else if (gyro.getRoll() > 0 && balancingDirection == 1)
        {
            balancingSpeed /= 2;
            balancingDirection = 1;
            wheelLockSpeed = 0;
            wheelsLocked = false;
        }
        else if (gyro.getRoll() < 0 && balancingDirection == -1)
        {
            balancingSpeed /= 2;
            balancingDirection = -1;
            wheelLockSpeed = 0;
            wheelsLocked = false;
        }

        if (balancingSpeed < 0.15)
        {
            balancingSpeed = 0.15;
        }

        s_Swerve.drive(new Translation2d(balancingSpeed * balancingDirection, wheelLockSpeed),
                0 * Constants.Swerve.maxAngularVelocity,
                /* !robotCentricSup.getAsBoolean(), */ true, // field relative is always on
                true);

        wheelLockSpeed = 0;
    }

}
