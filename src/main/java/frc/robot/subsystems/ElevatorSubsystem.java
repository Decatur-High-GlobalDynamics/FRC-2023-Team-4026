package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ITeamTalon;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.TeamTalonFX;
import frc.robot.commands.MoveElevatorCommand;

public class ElevatorSubsystem extends SubsystemBase {

    public ITeamTalon elevatorMotorMain, elevatorMotorSub;
    public double targetPosition = Constants.MINIMUM_ELEVATOR_POSITION;
    public final double DEADBAND_VALUE = Constants.ELEVATOR_DEADBAND_VALUE;

    public AnalogPotentiometer potentiometer;

    public ClawIntakeSubsystem intake;
    public boolean clawThresholdOverridden;

    public boolean targetOverridden;

    public double speed;

    public double newPower;

    public DigitalInput elevatorLimitSwitch;

    private static final double kP = 1.32; // 1.2392;
    private static final double kI = 0;
    private static final double kD = .0185; // 0.019259;

    PIDController pid = new PIDController(kP, kI, kD);

    public ElevatorSubsystem(ClawIntakeSubsystem intake) {
        elevatorMotorMain = new TeamTalonFX("Subsystem.Elevator.ElevatorMotorMain",
                Ports.ELEVATOR_MOTOR_MAIN);
        elevatorMotorSub = new TeamTalonFX("Subsystem.Elevator.ElevatorMotorSub",
                Ports.ELEVATOR_MOTOR_SUB);

        elevatorMotorMain.resetEncoder();
        elevatorMotorMain.enableVoltageCompensation(true);
        elevatorMotorMain.setInverted(false);
        elevatorMotorMain.setNeutralMode(NeutralMode.Brake);

        elevatorMotorSub.resetEncoder();
        elevatorMotorSub.enableVoltageCompensation(true);
        elevatorMotorSub.setInverted(true);
        elevatorMotorSub.setNeutralMode(NeutralMode.Brake);
        // elevatorMotorSub.follow(elevatorMotorMain);

        elevatorLimitSwitch = new DigitalInput(Ports.ELEVATOR_LIMIT_SWITCH);

        potentiometer = new AnalogPotentiometer(Ports.ELEVATOR_POTENTIOMETER, 100);
        RobotContainer.shuffleboard.addDouble("Elevator", () -> potentiometer.get());
        RobotContainer.shuffleboard.addDouble("Elevator Target", () -> targetPosition);
        RobotContainer.shuffleboard.addDouble("Main Elevator Encoder",
                () -> elevatorMotorMain.getCurrentEncoderValue());
        RobotContainer.shuffleboard.addDouble("Sub Elevator Encoder",
                () -> elevatorMotorSub.getCurrentEncoderValue());
        RobotContainer.shuffleboard.addDouble("Main Elevator Power", () -> elevatorMotorMain.get());
        RobotContainer.shuffleboard.addDouble("Sub Elevator Power", () -> elevatorMotorSub.get());
        RobotContainer.shuffleboard.addBoolean("Elevator Override", () -> targetOverridden);
        RobotContainer.shuffleboard.addBoolean("Claw Threshold Override",
                () -> clawThresholdOverridden);
        RobotContainer.shuffleboard.addBoolean("Elevator in Target", () -> isInTarget());
        RobotContainer.shuffleboard.addBoolean("Elevator Limit Switch",
                () -> elevatorLimitSwitch.get());

        this.intake = intake;
    }

    public void setTargetPosition(double newTargetPosition, String reason) {
        targetPosition = newTargetPosition;
    }

    private double getCappedPower(double desired) {
        return Math.min(Constants.maxElevatorMotorSpeed,
                Math.max(desired, -Constants.maxElevatorMotorSpeed));
    }

    public void setSpeed(double newSpeed) {
        double sign = Math.signum(newSpeed);
        // System.out.println("Passed Speed: " + newSpeed);
        this.speed = newSpeed;

        speed = Math.pow(speed, Constants.ELEVATOR_POWER_EXPONENT);

        speed *= Constants.maxElevatorMotorSpeed;

        speed = getCappedPower(speed);

        double currentPower = elevatorMotorMain.get();

        if ((speed < currentPower)) {
            newPower = Math.max(speed, currentPower - Constants.ELEVATOR_MAX_POWER_CHANGE);
        } // /jack morton lawrence torrance jack morton lawrence torrance jack morton
          // lawrence
          // torrance
        else if ((speed > currentPower)) {
            newPower = Math.min(speed, currentPower + Constants.ELEVATOR_MAX_POWER_CHANGE);
        } else {
            newPower = speed;
        }

        if ((potentiometer.get() > Constants.topElevatorTargetPosition && speed > 0)
                || (potentiometer.get() < Constants.MINIMUM_ELEVATOR_POSITION && speed < 0)) {
            newPower = 0;
        }

        if (elevatorLimitSwitch.get()) {
            if (newPower < 0) {
                newPower = 0;
            }
        }

        speed = Math.abs(speed) * sign;

        // System.out.println("Elevator Speed: " + speed + ", Sign: " + sign);
        elevatorMotorMain.set(newPower, "Joystick said so");
        elevatorMotorSub.set(newPower, "Joystick said so");
    }

    public void periodic() {
        // System.out.println("Elevator Power: " + elevatorMotorMain.get());
        // System.out.println("Current Potentiometer Value: " + potentiometer.get());

        if (!targetOverridden) {
            if (!isInTarget()) {
                // setSpeed(Math.signum(targetPosition - potentiometer.get()));

                setSpeed(pid.calculate(potentiometer.get(), targetPosition));
            } else {
                setSpeed(0);
            }
        }

        if (potentiometer.get() < Constants.clawCloseThreshold && !clawThresholdOverridden
                && intake.clawGrabberLeft.get() != Value.kReverse && Robot.isEnabled)
            intake.setSolenoid(Value.kReverse);

    }

    public boolean isInTarget() {
        double delta = targetPosition - potentiometer.get();
        return Math.abs(delta) < DEADBAND_VALUE;
    }

    public void resetTarget() {
        targetPosition = potentiometer.get();
    }

}
