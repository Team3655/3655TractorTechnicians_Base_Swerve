package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandNXT extends CommandGenericHID {

  private static final int FIRE_BUTTON_STAGE1 = 1;
  private static final int FIRE_BUTTON_STAGE2 = 2;
  private static final int A1 = 3;
  private static final int FIRE_PADDLE_UP = 21;
  private static final int FIRE_PADDLE_DOWN = 21;

  private final GenericHID hid;

  public CommandNXT(int port) {
    super(port);
    hid = new GenericHID(port);
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the first stage is depressed but not when the second is
   */
  public Trigger fireStage1() {
    return new Trigger(
        () -> hid.getRawButton(FIRE_BUTTON_STAGE1) && !hid.getRawButton(FIRE_BUTTON_STAGE2));
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the second stage is depressed
   */
  public Trigger fireStage2() {
    return new Trigger(() -> hid.getRawButton(FIRE_BUTTON_STAGE2));
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the second stage is depressed
   */
  public Trigger firePaddleUp() {
    return new Trigger(() -> hid.getRawButton(FIRE_PADDLE_UP));
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the second stage is depressed
   */
  public Trigger firePaddleDown() {
    return new Trigger(() -> hid.getRawButton(FIRE_PADDLE_DOWN));
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the second stage is depressed
   */
  public Trigger A1() {
    return new Trigger(() -> hid.getRawButton(A1));
  }
}
