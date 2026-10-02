#pragma once

#include <cmath>
#include <chrono>

namespace chassis
{

struct ArmJoystickAction
{
  bool selection_changed{false};
  // 0=x, 1=z, 2=planar pitch; direction is a single +/- step, not a joint angle.
  unsigned coordinate{0};
  int direction{0};
  bool repeated{false};
};

class ArmJoystick
{
public:
  using Clock = std::chrono::steady_clock;

  ArmJoystickAction observe(
    unsigned axis, double normalized, bool initial = false,
    Clock::time_point now = Clock::now())
  {
    ArmJoystickAction action;
    action.coordinate = selected_coordinate_;
    if (axis != kStepAxis) {return action;}

    normalized = std::isfinite(normalized) ? normalized : 0.0;
    seen_step_ = true;
    step_value_ = normalized;

    if (initial) {
      disarm();
      return action;
    }
    if (!armed_) {return action;}

    if (is_neutral(step_value_)) {
      step_ready_ = true;
      suspend_repeat();
      return action;
    }
    if (repeat_direction_ != 0 && repeat_direction_ * step_value_ > 0) {
      suspend_repeat();  // A direct direction flip still requires neutral.
    }
    if (step_ready_ && is_active(step_value_)) {
      step_ready_ = false;
      action.coordinate = selected_coordinate_;
      action.direction = step_value_ < 0.0 ? 1 : -1;
      repeat_direction_ = action.direction;
      next_repeat_ = now + std::chrono::milliseconds(100);
    }
    return action;
  }

  bool repeat_active() const
  {
    return armed_ && repeat_direction_ != 0 && is_active(step_value_) &&
           repeat_direction_ * step_value_ < 0;
  }

  ArmJoystickAction repeat(Clock::time_point now = Clock::now())
  {
    ArmJoystickAction action;
    action.coordinate = selected_coordinate_;
    if (repeat_active() && now >= next_repeat_) {
      action.direction = repeat_direction_;
      action.repeated = true;
      // No catch-up loop: a delayed control tick emits at most one new step.
      next_repeat_ = now + std::chrono::milliseconds(100);
    }
    return action;
  }

  void suspend_repeat() {repeat_direction_ = 0;}

  ArmJoystickAction observe_button(unsigned button, bool pressed, bool initial = false)
  {
    ArmJoystickAction action;
    action.coordinate = selected_coordinate_;
    auto * state = button_state(button);
    if (state == nullptr) {return action;}

    state->seen = true;
    const bool was_pressed = state->pressed;
    state->pressed = pressed;

    if (initial) {
      disarm();
      return action;
    }
    if (!armed_ || !pressed || was_pressed) {return action;}

    selected_coordinate_ = button == kPreviousButton ? previous_coordinate(selected_coordinate_) :
      next_coordinate(selected_coordinate_);
    suspend_repeat();
    if (!is_neutral(step_value_)) {step_ready_ = false;}
    action.selection_changed = true;
    action.coordinate = selected_coordinate_;
    return action;
  }

  bool arm()
  {
    if (!neutral_ready()) {
      disarm();
      return false;
    }
    armed_ = true;
    step_ready_ = true;
    return true;
  }

  bool armed() const {return armed_;}

  bool neutral_ready() const
  {
    return seen_step_ && is_neutral(step_value_) && previous_button_.seen &&
           next_button_.seen && !previous_button_.pressed && !next_button_.pressed;
  }

  unsigned selected_coordinate() const {return selected_coordinate_;}

  void disarm()
  {
    armed_ = false;
    step_ready_ = false;
    suspend_repeat();
  }

  void reset()
  {
    selected_coordinate_ = 0;
    seen_step_ = false;
    step_value_ = 0.0;
    previous_button_ = {};
    next_button_ = {};
    disarm();
  }

private:
  static constexpr unsigned kStepAxis = 7;
  static constexpr unsigned kPreviousButton = 3;
  static constexpr unsigned kNextButton = 1;
  static constexpr double kActiveThreshold = 0.5;
  static constexpr double kNeutralThreshold = 0.25;

  struct ButtonState
  {
    bool seen{false};
    bool pressed{false};
  };

  static bool is_active(double value) {return std::abs(value) >= kActiveThreshold;}
  static bool is_neutral(double value) {return std::abs(value) <= kNeutralThreshold;}
  static unsigned next_coordinate(unsigned coordinate) {return (coordinate + 1) % 3;}
  static unsigned previous_coordinate(unsigned coordinate) {return (coordinate + 2) % 3;}

  ButtonState * button_state(unsigned button)
  {
    if (button == kPreviousButton) {return &previous_button_;}
    if (button == kNextButton) {return &next_button_;}
    return nullptr;
  }

  bool seen_step_{false};
  double step_value_{0.0};
  ButtonState previous_button_{};
  ButtonState next_button_{};
  bool armed_{false};
  bool step_ready_{false};
  int repeat_direction_{0};
  Clock::time_point next_repeat_{};
  unsigned selected_coordinate_{0};
};

}  // namespace chassis
