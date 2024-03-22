package team4384.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.*;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import team4384.robot.subsystems.*;

import team4384.robot.sim.SimSuperStructure;


import team4384.robot.commands.intake.CmdIntake;
import team4384.robot.commands.intake.CmdPosManualIntake;
import team4384.robot.commands.pivot.CmdPosSetPointAngle;
import team4384.robot.commands.shooter.CmdVelSetPoint;
import team4384.robot.commands.TeleopSwerve;

import java.nio.file.Path;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final Joystick driver = new Joystick(0);

    public final Joystick manip = new Joystick(1);

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    final JoystickButton SlowMode  = new JoystickButton(driver, 1);
    final JoystickButton zeroOdo = new JoystickButton(driver, 3);
    final JoystickButton resetGyro = new JoystickButton(driver, 4);
    final JoystickButton intakeFeed = new JoystickButton(driver,5);
    final JoystickButton shooterOut = new JoystickButton(driver,5);

    public final JoystickButton intakeRaise = new JoystickButton(manip, 6);
    public final JoystickButton intakeLower = new JoystickButton(manip, 7);
    //public final JoystickButton intakeFeed = new JoystickButton(manip, 8);
    public final JoystickButton intakeBackFeed = new JoystickButton(manip, 9);
    public final JoystickButton pivotRaise = new JoystickButton(manip, 1);
    public final JoystickButton pivotLower = new JoystickButton(manip, 2);
//    public final JoystickButton shooterOut = new JoystickButton(manip, 3);
    public final JoystickButton shooterback = new JoystickButton(manip, 4);
    public final JoystickButton index = new JoystickButton(manip, 5);
    public final JoystickButton engage = new JoystickButton(manip, 10);

    private boolean engaged = false;



//ToDo - Read from DriveStation the alliance and pass the value to the Vision SubSystem

 //ToDo - Configure limelight in the Constants Class
    private static String kLimeLight_1 = "limelight-at";
    private static String kLimeLight_2 = "limelight-note";

    private final VisionLLAprilTag m_VisionAprilTag = new VisionLLAprilTag(DriverStation.Alliance.Blue, kLimeLight_1);
    private final FieldTelemetry m_FieldTelemetry = new FieldTelemetry();
    private SimSuperStructure m_visualizer;
  

    private final int translationAxis = Joystick.kDefaultYChannel;
    private final int strafeAxis = Joystick.kDefaultXChannel;
    private final int rotationAxis = Joystick.kDefaultZChannel;


    SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /* Sub System Instances  */
    private final Intake mIntake = new Intake();
    private final Shooter mShooter = new Shooter();
    public final Swerve s_Swerve = new Swerve();
    private final Pivot mPivot = new Pivot();
    private final Elevator mElevator = new Elevator();
//    private final Limelight limelight = new Limelight(s_Swerve, mPivot);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
       // Sendable Chooser Initialization

       // Configure the button bindings
       configureButtonBindings();

       s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
                    () -> driver.getRawAxis(3),
                    SlowMode,
                    resetGyro
            )
        );

       AutoBuilder.configureHolonomic(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            s_Swerve::getModuleStates,
            s_Swerve::autoDrive,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(1.0, 0, 0),
                    new PIDConstants(1.0, 0, 0),
                    5.0,
                    0.38,
                    new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
            },
            s_Swerve);

       //getStartingPose();

        Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("centerAuto");

        NamedCommands.registerCommand("Shoot", new Command() {
                    private final Timer time = new Timer();

                    @Override
                    public void initialize() {
                        time.start();
                    }

                    @Override
                    public void execute() {
                        while (time.get() < 2) {
                            mShooter.shoot();
                        }
                    }
                    @Override
                    public void end(boolean interrupted) {
                        if (time.get() >2){
                            mShooter.stopShoot();
                            boolean isFinished = true;
                        }

                    }
                });
        NamedCommands.registerCommand("Index", new Command() {

            private final Timer time = new Timer();
            @Override
            public void initialize() {
                time.start();
            }
            @Override
            public void execute() {
                while (time.get() <2) {
                    mShooter.index();
                }
            }
            @Override
            public void end(boolean interrupted){
                //mShooter.stopIndex();
            }
        });
        NamedCommands.registerCommand("intakeLower", new Command() {

            private final Timer time = new Timer();
            @Override
            public void initialize() {
                time.start();
            }
            @Override
            public void execute() {
                    while (time.get() <2) {
                        mIntake.lower();
                    }
            }
            @Override
            public void end(boolean interrupted){
                while(time.get() >2) {
                    mIntake.halt();
                }
            }
        });
        NamedCommands.registerCommand("intakeFeed", new Command() {
            private final Timer time = new Timer();
            @Override
            public void initialize() {
                time.start();
            }
            @Override

            public void execute() {
                while (time.get() < 2) {
                    mIntake.feed();
                }
            }
            public void end(boolean inerrupted){
                if (time.get() >2){
                    boolean isFinished = true;
                }
            }

        });

        NamedCommands.registerCommand("Pivot", new Command() {
            private final Timer time = new Timer();
            @Override
            public void initialize() {
                time.start();
            }
            @Override
            public void execute() {
                while (time.get() < 2) {
                    mPivot.lower();
                }
            }
            @Override
            public void end(boolean interrupted) {
                mPivot.stop();
                    }
        });
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("center");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("back");

        Command auto1 = AutoBuilder.followPath(path1);
        Command auto2 = AutoBuilder.followPath(path2);

        Command auto3 = AutoBuilder.buildAuto("backAuto");
        Command auto4 = AutoBuilder.buildAuto("centerAuto");

        autoChooser.setDefaultOption("center", auto1);
        autoChooser.addOption("back", auto2);
        autoChooser.addOption("backAuto", auto3);
        autoChooser.addOption("CenterAuto", auto4);
        SmartDashboard.putData("Auto Choices", autoChooser);

        if (RobotBase.isSimulation()) {
            m_visualizer = new SimSuperStructure();
            m_visualizer.registerIntake(mIntake);
            }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroGyro));
        intakeRaise.whileTrue(new Command() {
            @Override
            public void execute() {
                mIntake.raise();
            }
        });

        intakeLower.whileTrue(new Command() {
            @Override
            public void execute() {
                mIntake.lower();
            }
        });

        intakeFeed.whileTrue(new Command() {
            @Override
            public void execute() {
                mIntake.feed();

            }
        });

        intakeBackFeed.whileTrue(new Command() {
            @Override
            public void execute() {
                mIntake.backfeed();
            }
        });
//
        shooterback.whileTrue(new Command() {
            @Override
            public void execute() {
                mShooter.back();
            }
            @Override
            public void end(boolean interrupted) {
                mShooter.stopShoot();
            }
        });

        shooterOut.whileTrue(new Command() {
            @Override
            public void execute() {
                mShooter.shoot();
            }
            @Override
            public void end(boolean interrupted) {
                mShooter.stopShoot();
            }
        });

        index.whileTrue(new Command() {
            @Override
            public void execute() {
                mShooter.index();
            }

            @Override
            public void end(boolean interrupted) {
                mShooter.stopShoot();
            }
        });

        pivotRaise.whileTrue(new Command() {
            @Override
            public void execute() {
                mPivot.raise();
            }
            @Override
            public void end(boolean interrupted) {
                mPivot.stop();
            }
        });

        pivotLower.whileTrue(new Command() {
            @Override
            public void execute() {
                mPivot.lower();
            }
            @Override
            public void end(boolean interrupted) {
                mPivot.stop();
            }
        });

        engage.onTrue(new Command() {
            @Override
            public void execute() {
                if(engaged) {
                    mElevator.engage();
                    engaged = false;
                } else {
                    mElevator.disengage();
                    engaged = true;
                }
            }
        });
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

  
  // ToDo - Disable this function during competition. Required only for development and testing purpose.

  public void periodicRefresh(boolean simState) {    
    Pose2d visionRobotPose = new Pose2d();
    if (simState) {
       visionRobotPose = new Pose2d(11.753862616734166,2.540550112908883, new Rotation2d(0));
       if (m_visualizer != null) m_visualizer.periodic();
    }
    else{
      visionRobotPose = m_VisionAprilTag.getRobotPose2D();
      //limelight.amp_center(m_VisionAprilTag.getDesiredRobotPose());
    }
    m_FieldTelemetry.periodicPoseOfField("Robot", visionRobotPose);
  }
  
}
