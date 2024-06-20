package frc.robot;

import edu.wpi.first.wpilibj.I2C.Port;

public class Ports {
   public final static int INTAKE_MOTOR = 6;
   public final static int ELEVATOR_MOTOR_MAIN = 18;
   public final static int ELEVATOR_MOTOR_SUB = 17;
   public final static int PNEUMATICS_HUB = 5;
   public final static int CLAW_LEFT_CLOSE = 13; // Do these later
   public final static int CLAW_LEFT_OPEN = 15;
   public final static int CLAW_RIGHT_CLOSE = 0;
   public final static int CLAW_RIGHT_OPEN = 0;
   public final static int ELEVATOR_POTENTIOMETER = 2;
   public final static int ELEVATOR_LIMIT_SWITCH = 9;
   public final static Port NAV_X = Port.kMXP; // Need to check this
   public final static int GYRO = 0; // CHECK THIS!!!

   public class SwervePorts { // TODO: Assign port values for each module

      // Front left
      public final static int MOD0_DRIVEMOTOR = 4;
      public final static int MOD0_ANGLEMOTOR = 8;
      public final static int MOD0_CANCODER = 4;

      // Front right
      public final static int MOD1_DRIVEMOTOR = 3;
      public final static int MOD1_ANGLEMOTOR = 9;
      public final static int MOD1_CANCODER = 2;

      // Rear left
      public final static int MOD2_DRIVEMOTOR = 2;
      public final static int MOD2_ANGLEMOTOR = 6;
      public final static int MOD2_CANCODER = 3;

      // Rear right
      public final static int MOD3_DRIVEMOTOR = 1;
      public final static int MOD3_ANGLEMOTOR = 7;
      public final static int MOD3_CANCODER = 1;
   }
}
