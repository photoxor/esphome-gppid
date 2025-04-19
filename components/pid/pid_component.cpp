#include "pid_component.h"

#include "esphome/core/log.h"

namespace esphome {
namespace pid {

static const char *const TAG = "pid";

void PIDComponent::setup() {
    this->input_sensor_->add_on_state_callback([this](float state) {
        ESP_LOGD(TAG, "sensor callback - got value %f", state);
        this->update_pid_(state);
    });
    if (this->target_sensor_ != nullptr) {
        this->target_sensor_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "target sensor callback - submitting value %f", state);
            this->target_value_ = state;
        });
    }
#ifdef USE_NUMBER
    if (this->target_number_ != nullptr) {
        this->target_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "number callback - submitting value %f", state);
            this->target_value_ = state;
        });
    }
#endif
}

void PIDComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "PID Controller", this);
    ESP_LOGCONFIG(TAG, "  Control Parameters:");
    ESP_LOGCONFIG(TAG, "    kp: %.5f, ki: %.5f, kd: %.5f, output samples: %d",
                  controller_.kp_, controller_.ki_, controller_.kd_,
                  controller_.output_samples_);

    if (controller_.threshold_low_ == 0 && controller_.threshold_high_ == 0) {
        ESP_LOGCONFIG(TAG, "  Deadband disabled.");
    } else {
        ESP_LOGCONFIG(TAG, "  Deadband Parameters:");
        ESP_LOGCONFIG(
            TAG,
            "    threshold: %0.5f to %0.5f, multipliers(kp: %.5f, ki: "
            "%.5f, kd: %.5f), output samples: %d",
            controller_.threshold_low_, controller_.threshold_high_,
            controller_.kp_multiplier_, controller_.ki_multiplier_,
            controller_.kd_multiplier_, controller_.deadband_output_samples_);
    }
}

void PIDComponent::write_output_(float value) {
#ifdef USE_OUTPUT
    ESP_LOGD(TAG, "write output value %f, clamped to %f..%f", value, output_min_, output_max_);
    auto tmp = clamp(value, this->output_min_, this->output_max_);
    this->output_value_ = tmp;
    this->output_->set_level(tmp);
#endif

    this->pid_computed_callback_.call();
}

void PIDComponent::update_pid_(float current_value) {
    ESP_LOGD(TAG, "update_pid");
    float value = this->controller_.update(this->target_value_, current_value);
    this->write_output_(value);
}

void PIDComponent::reset_integral_term() {
    this->controller_.reset_accumulated_integral();
}

}  // namespace pid
}  // namespace esphome
