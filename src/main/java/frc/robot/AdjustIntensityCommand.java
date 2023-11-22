package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LauncherSubsystem;

public class AdjustIntensityCommand extends CommandBase {
    private final LauncherSubsystem launcherSubsystem;
private double motorPower;

//** Craetes a new AdjustIntensityCommand. */
public AdjustIntensityCommand(LauncherSubsystem launcherSubsystem, double motorPower) {
    // // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSubsystem = launcherSubsystem;
    this.motorPower = motorPower;
}

//Called when the command is initially scheduled.
@Override
public void initialize() {}

//Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    launcherSubsystem.setMotorPower(motorPower);
}

//Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return false;
}
}
