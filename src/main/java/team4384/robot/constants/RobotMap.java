package team4384.robot.constants;
import edu.wpi.first.math.util.Units;

public class RobotMap {

    public static final double drivebaseWidth = Units.inchesToMeters(23.0);
    public static final double drivebaseLength = Units.inchesToMeters(23.5);
  
    public static final double stickDeadband = 0.1; // Joystick Deadband

    public static class Intake {
        public static final int turnMotorLeader = 49;
        public static final int turnMotorFollower = 51;
        public static final int spinner = 57;
    }

    // Use these constants to enable or disable sub system during development or testing
    public static class SubSystemEnabler {
        public static final boolean enableIntake = true;
        public static final boolean enableIntakeStow = false;
        public static final boolean enableClimber = false;
        public static final boolean enableAprilTagVision = true;
        public static final boolean enableNoteDetectVision = false;
    }

    public static class Limelight {
        public final static String name = "limelight-benzene";
    }
    
    public static class Pivot {
        
        public static class DeviceID {
            public static final int turnMotorLeader = 46;
            public static final int turnMotorFollower = 47;
            public static final int leftIndexer = 55;
            public static final int rightIndexer = 56;

            public static final int leftShooter = 57;
            public static final int rightShooter = 58;
        }

        public static class Speeds {
            public static final double backSpeed = -0.1;
            public static final double indexSpeed = 0.1;
            public static final double shootSpeed = 0.5;
            public static final double pivotSpeed = 0.1;
        }

        public static class Presets {
            public static final double feedPos = 0;
            public static final double ampPos = 0;
            public static final double humanPlayerPos = 0;
        }
    }

    public static class Elevator {
        public static class Device_ID {
            public static int leftEngageID = 21;
            public static int rightEngageID = 22;
        }

        public static class Presets {
            public static final double home = 0.0;
            public static final double engaged = 1.0;
        }
    }
}
