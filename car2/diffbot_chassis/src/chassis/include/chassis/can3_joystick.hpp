#pragma once

#include <cmath>

namespace chassis
{

struct Can3JoystickAction
{
  bool selection_changed{false};
  unsigned motor_id{1};
  int delta_degrees{0};
};

class Can3Joystick
{
public:
  Can3JoystickAction observe(unsigned axis, double normalized, bool initial = false)
  {
    Can3JoystickAction action;
    action.motor_id = selected_motor_;
    if (axis != kStepAxis) {return action;}

    normalized = std::isfinite(normalized) ? normalized : 0.0;
    seen_step_ = true;
    step_value_ = normalized;

    if (initial) {
      armed_ = false;
      step_ready_ = false;
      return action;
    }
    if (!armed_) {return action;}

    if (is_neutral(step_value_)) {
      step_ready_ = true;
      return action;
    }
    if (step_ready_ && is_active(step_value_)) {
      step_ready_ = false;
      action.motor_id = selected_motor_;
      action.delta_degrees = step_value_ < 0.0 ? 1 : -1;
    }
    return action;
  }

  Can3JoystickAction observe_button(unsigned button, bool pressed, bool initial = false)
  {
    Can3JoystickAction action;
    action.motor_id = selected_motor_;
    auto * state = button_state(button);
    if (state == nullptr) {return action;}

    state->seen = true;
    const bool was_pressed = state->pressed;
    state->pressed = pressed;

    if (initial) {
      armed_ = false;
      step_ready_ = false;
      return action;
    }
    if (!armed_ || !pressed || was_pressed) {return action;}

    selected_motor_ = button == kPreviousButton ? previous_motor(selected_motor_) :
      next_motor(selected_motor_);
    if (!is_neutral(step_value_)) {step_ready_ = false;}
    action.selection_changed = true;
    action.motor_id = selected_motor_;
    return action;
  }

  bool arm()
  {
    if (!neutral_ready()) {
      armed_ = false;
      step_ready_ = false;
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

  unsigned selected_motor() const {return selected_motor_;}

  void disarm()
  {
    armed_ = false;
    step_ready_ = false;
  }

  void reset()
  {
    selected_motor_ = 1;
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
  static unsigned next_motor(unsigned motor) {return motor == 3 ? 1 : motor + 1;}
  static unsigned previous_motor(unsigned motor) {return motor == 1 ? 3 : motor - 1;}

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
  unsigned selected_motor_{1};
};

}  // namespace chassis
