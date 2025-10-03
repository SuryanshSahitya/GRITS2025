package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.algee;
import frc.robot.subsystems.elevator.elevatorsub;

public class barge extends Command {
  private final elevatorsub elevator;
  private final double targetPosition;
  private final algee algee;
  private final double tolerance = 0.25; // Tolerance to switch from Motion Magic to PID
  private double l0 = 0;
  private double l1 = -17.3;
  private double l2 = -27.1033203125;
  private double l3 = -27.1033203125;
  private double l4 = -22.03759765625;
  private boolean first;
  private boolean up;

  private double flipsetpoint = -23.38310546875;

  private enum State {
    MOVING,
    HOLDING
  }

  private State currentState;

  /**
   * Creates a new ElevatorMoveAndHoldCommand.
   *
   * @param elevator The elevator subsystem.
   * @param targetPosition The target position (in sensor units) to move to.
   */
  public barge(elevatorsub elevator, double targetPosition, boolean hi, algee algee) {
    this.up = hi;
    this.elevator = elevator;
    this.targetPosition = targetPosition;
    this.algee = algee;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

    // Start in the MOVING state and reset encoders if needed.
    currentState = State.MOVING;
    if (elevator.getLeftPosition() < 0.1) {
      elevator.resetenc();
    }

    first = true;
  }

  @Override
  public void execute() {

    // Check if the flip motor has reached its setpoint.
    // Note: Use flipsetpoint (not targetPosition) for the check. q
    if (!elevator.flipcheck(flipsetpoint)) {

      // Command the flip motor until it is at its setpoint.
      elevator.setMotionMagicflip(flipsetpoint);
      // Do not start moving the elevator until the flip motor is ready.
      return;
    }

    if (!elevator.autoncheck2(targetPosition)) {
      elevator.setMotionMagic(targetPosition);
      elevator.setMotionMagicflip(flipsetpoint);
    }

    if (elevator.autoncheck2(targetPosition)) {
      elevator.setMotionMagic(targetPosition);
      elevator.setMotionMagicflip(flipsetpoint);

      algee.setShooter(0.9);
    }

    // Once the flip motor is holding its setpoint, command the elevator.

    // When close enough to the target, switch to PID holding mode.
    // if (Math.abs(currentPos - targetPosition) < tolerance) {
    //   currentState = State.HOLDING;
    //   elevator.initializePid(targetPosition);
    // } else if (currentState == State.HOLDING) {
    //   // Use PID to hold the position.
    //   elevator.Motionmagictoggle(targetPosition);
  }

  @Override
  public boolean isFinished() {
    // This command runs until it is interrupted (for example, by another command).
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the elevator when the command ends.

  }
}
