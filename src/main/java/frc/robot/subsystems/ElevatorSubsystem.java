package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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
    public double targetPosition = Constants.restElevatorTargetPosition;
    public final double DEADBAND_VALUE = Constants.ELEVATOR_DEADBAND_VALUE;
    
    public AnalogPotentiometer potentiometer;

    public ClawIntakeSubsystem intake;
    public boolean clawThresholdOverridden;

    public boolean targetOverridden;

    public double speed;

    public double newPower;

    public DigitalInput elevatorLimitSwitch;

    public ElevatorSubsystem(ClawIntakeSubsystem intake)
    {
        elevatorMotorMain = new TeamTalonFX("Subsystem.Elevator.ElevatorMotorMain", Ports.ELEVATOR_MOTOR_MAIN);
        elevatorMotorSub = new TeamTalonFX("Subsystem.Elevator.ElevatorMotorSub", Ports.ELEVATOR_MOTOR_SUB);

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
        RobotContainer.shuffleboard.addDouble("Main Elevator Encoder", () -> elevatorMotorMain.getCurrentEncoderValue());
        RobotContainer.shuffleboard.addDouble("Sub Elevator Encoder", () -> elevatorMotorSub.getCurrentEncoderValue());
        RobotContainer.shuffleboard.addDouble("Main Elevator Power", () -> elevatorMotorMain.get());
        RobotContainer.shuffleboard.addDouble("Sub Elevator Power", () -> elevatorMotorSub.get());
        RobotContainer.shuffleboard.addBoolean("Elevator Override", () -> targetOverridden);
        RobotContainer.shuffleboard.addBoolean("Claw Threshold Override", () -> clawThresholdOverridden);
        RobotContainer.shuffleboard.addBoolean("Elevator in Target", () -> isInTarget());
        RobotContainer.shuffleboard.addBoolean("Elevator Limit Switch", () -> elevatorLimitSwitch.get());

        this.intake = intake;
    }

    public void setTargetPosition(double newTargetPosition, String reason) {
        targetPosition = newTargetPosition;
    }

    private double getRampedPower(double desired) {
        double currentPower = elevatorMotorMain.get();
    
        if ((desired < currentPower))
        {
          desired = Math.max(desired, currentPower - Constants.ELEVATOR_MAX_POWER_CHANGE);
        } 
        else if ((desired > currentPower))
        {
          desired = Math.min(desired, currentPower + Constants.ELEVATOR_MAX_POWER_CHANGE);
        }
    
        return desired;
      }

    private double getCappedPower(double desired)
    {
        return Math.min(1, Math.max(desired, -1));
    }

    public void setSpeed(double newSpeed) {
        double sign = Math.signum(newSpeed);
        // System.out.println("Passed Speed: " + newSpeed);
        speed = newSpeed;

        speed *= Constants.maxElevatorMotorSpeed;

        speed = getRampedPower(speed);

        if((potentiometer.get() > Constants.topElevatorTargetPosition && speed > 0) ||
            (potentiometer.get() < Constants.MINIMUM_ELEVATOR_POSITION && speed < 0)) {
            speed = 0;
        }

        if (elevatorLimitSwitch.get()) {
            if (speed < 0) {
                speed = 0;
            }
        }

        speed = getCappedPower(speed);

        // speed = Math.abs(speed) * sign;

        // System.out.println("Elevator Speed: " + speed + ", Sign: " + sign);
        elevatorMotorMain.set(speed, "Joystick said so");
        elevatorMotorSub.set(speed, "Joystick said so");
    }

    public void periodic() {
        // System.out.println("Elevator Power: " + elevatorMotorMain.get());
        // System.out.println("Current Potentiometer Value: " + potentiometer.get());
        
        if(!targetOverridden) {
            if (!isInTarget()) {
                System.out.println("Periodic is setting elevator speed to " + (targetPosition - potentiometer.get()));
                setSpeed(Math.signum(targetPosition - potentiometer.get()));
            } 
            else {
                setSpeed(0);
            }
        }

        if(potentiometer.get() < Constants.clawCloseThreshold && !clawThresholdOverridden
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
