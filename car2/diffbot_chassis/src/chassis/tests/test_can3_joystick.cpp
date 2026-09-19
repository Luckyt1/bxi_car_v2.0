#ifdef NDEBUG
#undef NDEBUG
#endif

#include "chassis/can3_joystick.hpp"

#include <cassert>

namespace
{

void observe_ready_inputs(chassis::Can3Joystick & joystick)
{
  auto action = joystick.observe(7, 0.0);
  assert(!action.selection_changed && action.delta_degrees == 0);
  action = joystick.observe_button(3, false);
  assert(!action.selection_changed && action.delta_degrees == 0);
  action = joystick.observe_button(1, false);
  assert(!action.selection_changed && action.delta_degrees == 0);
}

void arm_neutral(chassis::Can3Joystick & joystick)
{
  observe_ready_inputs(joystick);
  assert(joystick.neutral_ready());
  assert(joystick.arm());
  assert(joystick.armed());
}

void startup_pressed_axis_does_not_arm_until_it_returns_neutral()
{
  chassis::Can3Joystick joystick;
  auto action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe_button(3, false);
  assert(action.delta_degrees == 0);
  action = joystick.observe_button(1, false);
  assert(action.delta_degrees == 0);
  assert(!joystick.neutral_ready());
  assert(!joystick.arm());

  action = joystick.observe(7, 0.0);
  assert(action.delta_degrees == 0);
  assert(joystick.neutral_ready());
  assert(joystick.arm());
}

void initial_events_record_state_but_never_emit_actions_and_disarm()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);
  auto action = joystick.observe(7, -1.0, true);
  assert(!joystick.armed());
  assert(action.delta_degrees == 0);
  assert(!joystick.arm());

  action = joystick.observe(7, 0.0, true);
  assert(action.delta_degrees == 0);
  assert(joystick.neutral_ready());
  assert(joystick.arm());
}

void initial_button_events_record_state_but_never_select_and_disarm()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);
  auto action = joystick.observe_button(1, true, true);
  assert(!joystick.armed());
  assert(!action.selection_changed);
  assert(joystick.selected_motor() == 1);
  assert(!joystick.arm());

  action = joystick.observe_button(1, false, true);
  assert(!action.selection_changed);
  assert(joystick.neutral_ready());
  assert(joystick.arm());
}

void axis7_negative_steps_up_once_then_requires_neutral()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);

  auto action = joystick.observe(7, -1.0);
  assert(action.motor_id == 1 && action.delta_degrees == 1);
  action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -0.3);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -0.2);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 1);
}

void axis7_positive_steps_down_once_then_requires_neutral()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);

  auto action = joystick.observe(7, 1.0);
  assert(action.motor_id == 1 && action.delta_degrees == -1);
  action = joystick.observe(7, 1.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, 0.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, 0.6);
  assert(action.delta_degrees == -1);
}

void direct_flip_between_positive_and_negative_does_not_repeat()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);

  auto action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 1);
  action = joystick.observe(7, 1.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 0);
}

void button1_selects_next_motor_once_per_press()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);

  auto action = joystick.observe_button(1, true);
  assert(action.selection_changed && action.motor_id == 2);
  assert(joystick.selected_motor() == 2);
  action = joystick.observe_button(1, true);
  assert(!action.selection_changed && action.motor_id == 2);
  action = joystick.observe_button(1, false);
  assert(!action.selection_changed);
  action = joystick.observe_button(1, true);
  assert(action.selection_changed && action.motor_id == 3);
  action = joystick.observe_button(1, false);
  assert(!action.selection_changed);
  action = joystick.observe_button(1, true);
  assert(action.selection_changed && action.motor_id == 1);
}

void button3_selects_previous_motor_once_per_press()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);

  auto action = joystick.observe_button(3, true);
  assert(action.selection_changed && action.motor_id == 3);
  assert(joystick.selected_motor() == 3);
  action = joystick.observe_button(3, true);
  assert(!action.selection_changed && action.motor_id == 3);
  action = joystick.observe_button(3, false);
  assert(!action.selection_changed);
  action = joystick.observe_button(3, true);
  assert(action.selection_changed && action.motor_id == 2);
  action = joystick.observe_button(3, false);
  assert(!action.selection_changed);
  action = joystick.observe_button(3, true);
  assert(action.selection_changed && action.motor_id == 1);
}

void selecting_motor_while_axis7_is_held_requires_neutral_before_step()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);

  auto action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 1 && action.motor_id == 1);
  action = joystick.observe_button(1, true);
  assert(action.selection_changed && action.motor_id == 2);
  action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, 0.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -1.0);
  assert(action.delta_degrees == 1 && action.motor_id == 2);
}

void startup_held_button_does_not_arm_until_released()
{
  chassis::Can3Joystick joystick;
  auto action = joystick.observe(7, 0.0);
  assert(action.delta_degrees == 0);
  action = joystick.observe_button(3, true);
  assert(!action.selection_changed);
  action = joystick.observe_button(1, false);
  assert(!action.selection_changed);
  assert(!joystick.neutral_ready());
  assert(!joystick.arm());

  action = joystick.observe_button(3, false);
  assert(!action.selection_changed);
  assert(joystick.neutral_ready());
  assert(joystick.arm());
}

void axis6_is_ignored_and_does_not_affect_selection_or_arm()
{
  chassis::Can3Joystick joystick;
  auto action = joystick.observe(6, 1.0);
  assert(!action.selection_changed && action.delta_degrees == 0);
  assert(!joystick.neutral_ready());
  assert(!joystick.arm());
  assert(joystick.selected_motor() == 1);
  action = joystick.observe(7, 0.0);
  assert(!action.selection_changed && action.delta_degrees == 0);
  action = joystick.observe_button(3, false);
  assert(!action.selection_changed);
  assert(!joystick.neutral_ready());
  action = joystick.observe_button(1, false);
  assert(!action.selection_changed);
  assert(joystick.neutral_ready());
  assert(joystick.arm());
  action = joystick.observe(6, -1.0);
  assert(!action.selection_changed && action.motor_id == 1 && action.delta_degrees == 0);
  assert(joystick.selected_motor() == 1);
}

void unknown_axis_and_button_do_not_mark_ready_or_emit_actions()
{
  chassis::Can3Joystick joystick;
  auto action = joystick.observe(3, 1.0);
  assert(!action.selection_changed && action.delta_degrees == 0);
  action = joystick.observe_button(2, true);
  assert(!action.selection_changed && action.delta_degrees == 0);
  assert(!joystick.neutral_ready());
  assert(!joystick.arm());
  assert(joystick.selected_motor() == 1);
}

void disarm_preserves_selection_but_requires_neutral_before_rearming()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);
  auto action = joystick.observe_button(1, true);
  assert(action.selection_changed && joystick.selected_motor() == 2);
  joystick.disarm();
  assert(!joystick.armed());
  assert(joystick.selected_motor() == 2);
  assert(!joystick.arm());

  action = joystick.observe_button(1, false);
  assert(!action.selection_changed);
  assert(joystick.arm());
  action = joystick.observe(7, -1.0);
  assert(action.motor_id == 2 && action.delta_degrees == 1);
}

void reset_clears_seen_axes_armed_state_and_selection()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);
  auto action = joystick.observe_button(3, true);
  assert(action.selection_changed && joystick.selected_motor() == 3);
  joystick.reset();
  assert(!joystick.armed());
  assert(!joystick.neutral_ready());
  assert(joystick.selected_motor() == 1);
  assert(!joystick.arm());
}

void threshold_hysteresis_blocks_jitter_until_true_neutral()
{
  chassis::Can3Joystick joystick;
  arm_neutral(joystick);

  auto action = joystick.observe(7, -0.49);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -0.50);
  assert(action.delta_degrees == 1);
  action = joystick.observe(7, -0.26);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -0.50);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -0.25);
  assert(action.delta_degrees == 0);
  action = joystick.observe(7, -0.50);
  assert(action.delta_degrees == 1);
}

}  // namespace

int main()
{
  startup_pressed_axis_does_not_arm_until_it_returns_neutral();
  initial_events_record_state_but_never_emit_actions_and_disarm();
  initial_button_events_record_state_but_never_select_and_disarm();
  axis7_negative_steps_up_once_then_requires_neutral();
  axis7_positive_steps_down_once_then_requires_neutral();
  direct_flip_between_positive_and_negative_does_not_repeat();
  button1_selects_next_motor_once_per_press();
  button3_selects_previous_motor_once_per_press();
  selecting_motor_while_axis7_is_held_requires_neutral_before_step();
  startup_held_button_does_not_arm_until_released();
  axis6_is_ignored_and_does_not_affect_selection_or_arm();
  unknown_axis_and_button_do_not_mark_ready_or_emit_actions();
  disarm_preserves_selection_but_requires_neutral_before_rearming();
  reset_clears_seen_axes_armed_state_and_selection();
  threshold_hysteresis_blocks_jitter_until_true_neutral();
}
