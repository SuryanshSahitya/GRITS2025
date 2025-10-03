package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.elevatorsub;

public class whatthehelly extends Command {
  private final elevatorsub elevator;

  private int targetposition;
  private final double tolerance = 0.25; // Tolerance to switch from Motion Magic to PID
  private double l0 = 0;

  private double l1 = -7.13134765625;
  private double l2 = -26.4;
  private double l3 = -26.4;
  private double l4 = -23.38310546875;

  private boolean first;
  private boolean up;

  private double flipsetpoint;

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
  public whatthehelly(elevatorsub elevator, int targetposition, boolean hi) {
    this.up = hi;
    this.elevator = elevator;
    this.targetposition = targetposition;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {

    if (elevator.whichlist() == 1) {

      Constants.setRobotState(Constants.RobotState.IDLE);
    } else if (elevator.whichlist() == 2) {
      Constants.setRobotState(Constants.RobotState.ALGEA);
    }

    if (targetposition == 1) {
      Constants.setElevatorState(Constants.Elevatorposition.L1);
    } else if (targetposition == 2) {
      Constants.setElevatorState(Constants.Elevatorposition.L2);
    } else if (targetposition == 3) {
      Constants.setElevatorState(Constants.Elevatorposition.L3);
    } else if (targetposition == 4) {
      Constants.setElevatorState(Constants.Elevatorposition.L4);

      // Normal
    } else {
      Constants.setElevatorState(Constants.Elevatorposition.L0);
    }
    // Start in the MOVING state and reset encoders if needed.
    currentState = State.MOVING;
    if (elevator.getLeftPosition() < 0.1) {
      elevator.resetenc();
    }

    first = true;
  }

  @Override
  public void execute() {

    if (up) {
      // Set flipsetpoint based on the desired elevator state.
      if (Constants.getElevatorState() == Constants.Elevatorposition.L1) {
        flipsetpoint = l1;

      } else if (Constants.getElevatorState() == Constants.Elevatorposition.L2) {
        flipsetpoint = l2;
      } else if (Constants.getElevatorState() == Constants.Elevatorposition.L3) {
        flipsetpoint = l3;
      } else if (Constants.getElevatorState() == Constants.Elevatorposition.L4) {
        flipsetpoint = l4;

        // BargeShoot

      }

      // Check if the flip motor has reached its setpoint.
      // Note: Use flipsetpoint (not targetPosition) for the check.
      if (!elevator.flipcheck(-14.6)) {

        // Command the flip motor until it is at its setpoint.
        elevator.setMotionMagicflip(flipsetpoint);
        // Do not start moving the elevator until the flip motor is ready.
        return;
      }

      // Once the flip motor is holding its setpoint, command the elevator.

      elevator.setMotionMagic1(targetposition);
      elevator.setMotionMagicflip(flipsetpoint);
      // When close enough to the target, switch to PID holding mode.
      // if (Math.abs(currentPos - targetPosition) < tolerance) {
      //   currentState = State.HOLDING;
      //   elevator.initializePid(targetPosition);
      // } else if (currentState == State.HOLDING) {
      //   // Use PID to hold the position.
      //   elevator.Motionmagictoggle(targetPosition);
    } else if (up == false) {
      elevator.Motionmagic(0);
      if (elevator.check(0)) {
        elevator.setMotionMagicflip(-0.4);
        // idk
      }
    }
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
