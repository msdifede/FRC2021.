/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CarWash;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.PreShooter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;
/**
 * An example command that uses an example subsystem.
 */
public class ShootSpheres extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Launcher launcher;
  private final PreShooter preShooter;
  private final CarWash carWash;
  private final LimeLight limelight;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootSpheres(Launcher launcher, PreShooter preShooter, CarWash carWash, LimeLight limelight) {
    this.launcher = launcher;
    this.preShooter = preShooter;
    this.carWash = carWash;
    this.limelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
    addRequirements(preShooter);
    addRequirements(carWash);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcher.spin();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  if( launcher.atSetpoint() ) {
      
      // Move the motors to the speed according to the distance from the robot to the turret
      preShooter.moveExitMotor(limelight.getTargetDistance() * Constants.distanceModifier);
      preShooter.moveEntryMotor(limelight.getTargetDistance() * Constants.distanceModifier);
      carWash.move();
    } else {
      preShooter.moveEntryMotor(0);
      carWash.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    preShooter.stop();
    carWash.stop();
    launcher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
