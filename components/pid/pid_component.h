#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#ifdef USE_OUTPUT
#include "esphome/components/output/float_output.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif

#include "pid_controller.h"

namespace esphome {
namespace pid {

class PIDComponent : public Component {
 public:
  PIDComponent() = default;
  void setup() override;
  void dump_config() override;

  void set_value_sensor(sensor::Sensor *sensor) { value_sensor_ = sensor; }
  void set_target_sensor(sensor::Sensor *sensor) { target_sensor_ = sensor; }
#ifdef USE_NUMBER
  void set_target_number(number::Number *number) { target_number_ = number; }
#endif
  void set_kp(float kp) { controller_.kp_ = kp; }
  void set_ki(float ki) { controller_.ki_ = ki; }
  void set_kd(float kd) { controller_.kd_ = kd; }
  void set_min_integral(float min_integral) { controller_.min_integral_ = min_integral; }
  void set_max_integral(float max_integral) { controller_.max_integral_ = max_integral; }
  void set_output_samples(int in) { controller_.output_samples_ = in; }
  void set_derivative_samples(int in) { controller_.derivative_samples_ = in; }

  void set_threshold_low(float in) { controller_.threshold_low_ = in; }
  void set_threshold_high(float in) { controller_.threshold_high_ = in; }
  void set_kp_multiplier(float in) { controller_.kp_multiplier_ = in; }
  void set_ki_multiplier(float in) { controller_.ki_multiplier_ = in; }
  void set_kd_multiplier(float in) { controller_.kd_multiplier_ = in; }
  void set_starting_integral_term(float in) { controller_.set_starting_integral_term(in); }

  void set_deadband_output_samples(int in) { controller_.deadband_output_samples_ = in; }

  #ifdef USE_OUTPUT
  void set_output(output::FloatOutput *output) { output_ = output; }
  #endif

  float get_output_value() const { return output_value_; }
  float get_error_value() const { return controller_.error_; }
  float get_kp() { return controller_.kp_; }
  float get_ki() { return controller_.ki_; }
  float get_kd() { return controller_.kd_; }
  float get_min_integral() { return controller_.min_integral_; }
  float get_max_integral() { return controller_.max_integral_; }
  float get_proportional_term() const { return controller_.proportional_term_; }
  float get_integral_term() const { return controller_.integral_term_; }
  float get_derivative_term() const { return controller_.derivative_term_; }
  int get_output_samples() { return controller_.output_samples_; }
  int get_derivative_samples() { return controller_.derivative_samples_; }

  float get_threshold_low() { return controller_.threshold_low_; }
  float get_threshold_high() { return controller_.threshold_high_; }
  float get_kp_multiplier() { return controller_.kp_multiplier_; }
  float get_ki_multiplier() { return controller_.ki_multiplier_; }
  float get_kd_multiplier() { return controller_.kd_multiplier_; }
  int get_deadband_output_samples() { return controller_.deadband_output_samples_; }
  bool in_deadband() { return controller_.in_deadband(); }

  // int get_derivative_samples() const { return controller_.derivative_samples; }
  // float get_deadband() const { return controller_.deadband; }
  // float get_proportional_deadband_multiplier() const { return controller_.proportional_deadband_multiplier; }

  void add_on_pid_computed_callback(std::function<void()> &&callback) {
    pid_computed_callback_.add(std::move(callback));
  }

  void reset_integral_term();
  void set_output_range(float output_min, float output_max) { output_min_= output_min; output_max_ = output_max; }

 protected:

  void update_pid_();
  void write_output_(float value);

  /// The sensor used for getting the current value
  sensor::Sensor *value_sensor_;
  sensor::Sensor *target_sensor_{nullptr};
  #ifdef USE_NUMBER
  number::Number *target_number_{nullptr};
  #endif

  #ifdef USE_OUTPUT
  output::FloatOutput *output_{nullptr};
  float output_min_, output_max_;
  #endif


  PIDController controller_;
  /// Output value as reported by the PID controller, for PIDComponentSensor
  float output_value_;
  CallbackManager<void()> pid_computed_callback_;

  float current_value_{NAN};
  float target_value_{NAN};
};


template<typename... Ts> class PIDResetIntegralTermAction : public Action<Ts...> {
 public:
  PIDResetIntegralTermAction(PIDComponent *parent) : parent_(parent) {}

  void play(Ts... x) { this->parent_->reset_integral_term(); }

 protected:
  PIDComponent *parent_;
};

template<typename... Ts> class PIDSetControlParametersAction : public Action<Ts...> {
 public:
  PIDSetControlParametersAction(PIDComponent *parent) : parent_(parent) {}

  void play(Ts... x) {
    auto kp = this->kp_.value(x...);
    auto ki = this->ki_.value(x...);
    auto kd = this->kd_.value(x...);

    this->parent_->set_kp(kp);
    this->parent_->set_ki(ki);
    this->parent_->set_kd(kd);
  }

 protected:
  TEMPLATABLE_VALUE(float, kp)
  TEMPLATABLE_VALUE(float, ki)
  TEMPLATABLE_VALUE(float, kd)

  PIDComponent *parent_;
};

}  // namespace pid
}  // namespace esphome
