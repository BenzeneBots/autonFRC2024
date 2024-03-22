package team4384.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import team4384.robot.subsystems.Intake;

public class CmdIntake extends Command {

    Intake mIntake;
    double mIntakeWheelSpeed; 

    public CmdIntake(Intake Intake, double intakeWheelSpeed) {
        this.mIntake = Intake;
        this.mIntakeWheelSpeed = intakeWheelSpeed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mIntake);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mIntake.intakeWheelForward(mIntakeWheelSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIntake.intakeWheelForward(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //ToDo - Can we detect when to end intake command automatically? Determine conditions when intake can stop
        //ToDo - Without a finish logic, this command will run forever . 
        return false;
    }    
    
}
