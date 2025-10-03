package frc.robot.subsystems.flywheel;

// import frc.robot.subsystems.lookuptable.setpoint;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.autovision;

public class shooter extends SubsystemBase {
  private TalonFX intake = new TalonFX(16);

  private int distance;
  private Timer time3 = new Timer();

  public shooter() {

    intake.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Intake Current", intake.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake Velocity", intake.getVelocity().getValueAsDouble());

    SmartDashboard.putString("Coral sate", Constants.getCoralstate().name());
    SmartDashboard.putString("vision sate", Constants.getvisionstate().name());
    SmartDashboard.putBoolean("velcoitycheck", velocitycheck());
  }

  public boolean check() {
    return distance > 75;
  }

  public double getDistance() {
    return distance;
  }

  public void speed(double speed) {
    intake.set(speed);
  }

  public double velocity() {
    return intake.getVelocity().getValueAsDouble();
  }

  public boolean hasCurrentSpike() {

    double current = intake.getStatorCurrent().getValueAsDouble();

    double CURRENT_SPIKE_THRESHOLD = 40.0;

    return current > CURRENT_SPIKE_THRESHOLD;
  }

  public boolean velocitywe() {

    double current = intake.getStatorCurrent().getValueAsDouble();

    return current < 10;
  }

  public boolean velocitycheck() {
    return Math.abs(intake.getVelocity().getValueAsDouble()) > 40;
  }

  public Command cmd(double speed) {

    return new Command() {
      @Override
      public void initialize() {

        time3.reset();
        time3.start();
      }

      @Override
      public void execute() {
        if (speed == -0.2 && Math.abs(intake.getVelocity().getValueAsDouble()) > 10) {
          Constants.setCoralstate(Constants.coralstate.None);
        }

        if (speed == 0.07
            && Math.abs(intake.getVelocity().getValueAsDouble()) < 4
            && time3.get() > 1) {
          Constants.setCoralstate(Constants.coralstate.Holding);
        }

        if (speed == -0.3) {
          Constants.setCoralstate(Constants.coralstate.None);
        }
        // check(position);
        intake.set(speed);
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public void visioncheck() {
    if (Constants.getvisionstate() == autovision.None) {
      Constants.setvisionsate(autovision.Holding);
    } else {
      Constants.setvisionsate(autovision.None);
    }
  }

  public Command scourcecmd(double speed) {
    return new Command() {
      @Override
      public void initialize() {

        // Initialization code, such as resetting encoders or PID controllers
        // int kErrThreshold = 10; // how many sensor units until its close-enough
        // int kLoopsToSettle = 2; // how many loops sensor must be close-enough
        // int _withinThresholdLoops = 0;
      }

      @Override
      public void execute() {
        // check(position);
        intake.set(speed);
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command scourcecmd1(double speed) {
    return new Command() {
      @Override
      public void initialize() {

        // Initialization code, such as resetting encoders or PID controllers
        // int kErrThreshold = 10; // how many sensor units until its close-enough
        // int kLoopsToSettle = 2; // how many loops sensor must be close-enough
        // int _withinThresholdLoops = 0;
      }

      @Override
      public void execute() {
        // check(position);
        intake.set(speed);
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public boolean hasVelocity(double inputVelo) {

    double Velo = intake.getVelocity().getValueAsDouble();

    double velocitythreshold = inputVelo;

    return Math.abs(Velo) > velocitythreshold;
  }

  public boolean hasVelocityautoalighn() {
    Constants.setvisionsate(autovision.None);

    double Velo = intake.getVelocity().getValueAsDouble();

    double velocitythreshold = 18;

    return Math.abs(Velo) > velocitythreshold;
  }

  public boolean hasVelocityautoalighnl3() {

    double Velo = intake.getVelocity().getValueAsDouble();

    double velocitythreshold = 11;

    return Math.abs(Velo) > velocitythreshold;
  }

  public Command autoncmdOut(double speed, double Velocity) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        intake.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        intake.set(0);
      }

      @Override
      public boolean isFinished() {
        return hasVelocity(Velocity);
      }
    };
  }

  public Command autoncmdIn(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        time3.reset();
        time3.start();
      }

      @Override
      public void execute() {
        intake.set(speed);
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return time3.get() > 0.75;
      }
    };
  }
}
