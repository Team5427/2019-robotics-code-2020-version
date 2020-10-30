/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5427.robot.commands.auto;

import org.usfirst.frc.team5427.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StraightMovement extends PIDCommand 
{

  private double timeSinceInitialized;
  /**
   * Creates a new StraightMovement.
   */
  public StraightMovement() {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0, 0),
        // This should return the measurement
        () -> Robot.getAHRS().getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> 
        {
          // Use the output here
          Robot.driveTrain.driveLeft(-output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }


  @Override
  public void initialize() 
  {
    Robot.getAHRS().reset();
    timeSinceInitialized = Timer.getFPGATimestamp();
  }

  @Override
  public void execute()
  {
    timeSinceInitialized = Timer.getFPGATimestamp();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return timeSinceInitialized > 10;
  }

  @Override
  public void end(boolean interrupted)
  {
    Robot.driveTrain.driveRight.set(0);
    Robot.driveTrain.driveLeft.set(0);
  }
}
