#include "pid_component.h"

#include "esphome/core/log.h"

namespace esphome {
namespace pid {

static const char *const TAG = "pid";

void PIDComponent::setup() {
    this->input_sensor_->add_on_state_callback([this](float state) {
        ESP_LOGD(TAG, "input sensor callback - got value %f", state);
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
            ESP_LOGD(TAG, "target number callback - submitting value %f", state);
            if (std::isfinite(state)) this->target_value_ = state;
        });
        // retrieve current inital value
        this->target_value_ = this->target_number_->state;
    }
    if (this->kp_number_ != nullptr) {
        this->kp_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "kp callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_kp(state);
        });
        // retrieve current inital value
        this->set_kp(this->kp_number_->state);
    }
    if (this->ki_number_ != nullptr) {
        this->ki_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "ki callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_ki(state);
        });
        // retrieve current inital value
        this->set_ki(this->ki_number_->state);
    }
    if (this->kd_number_ != nullptr) {
        this->kd_number_->add_on_state_callback([this](float state) {
            ESP_LOGD(TAG, "kd callback - submitting value %f", state);
            if (std::isfinite(state)) this->set_kd(state);
        });
        // retrieve current inital value
        this->set_kd(this->kd_number_->state);
    }

#endif
}

void PIDComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "PID Controller (HR)");
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
  if (this->autotuner_ != nullptr) {
    this->autotuner_->dump_config();
  }
}

void PIDComponent::update_pid_(float current_value) {
    if (std::isfinite(current_value) && std::isfinite(this->target_value_)) {
        float value = this->controller_.update(this->target_value_, current_value);
        ESP_LOGD(TAG, "update_pid: %f -> %f: %f", current_value, this->target_value_, value);
        ESP_LOGD(TAG, "write output value %f, clamped to %f..%f", value, output_min_, output_max_);

        // Check autotuner
        if (this->autotuner_ != nullptr && !this->autotuner_->is_finished()) {
          auto res = this->autotuner_->update(this->target_value, current_value);
          if (res.result_params.has_value()) {
            this->controller_.kp_ = res.result_params->kp;
            this->controller_.ki_ = res.result_params->ki;
            this->controller_.kd_ = res.result_params->kd;
            // keep autotuner instance so that subsequent dump_configs will print the long result message.
          } else {
            value = res.output;
          }
        }
        
        auto tmp = clamp(value, this->output_min_, this->output_max_);
        this->output_value_ = tmp;
        this->pid_computed_callback_.call();
#ifdef USE_OUTPUT
        ESP_LOGD(TAG, "write output: tmp: %f", tmp);
        if (this->output_ != nullptr) {
            this->output_->set_level(tmp);
        }
#endif
    }
    else {
        ESP_LOGD(TAG, "nan");
    }
}

void PIDComponent::reset_integral_term() {
    this->controller_.reset_accumulated_integral();
}

void PIDComponent::start_autotune(std::unique_ptr<PIDAutotuner> &&autotune) {
  this->autotuner_ = std::move(autotune);
  float min_value = 0.0f;
  float max_value = 0.0f;
  this->autotuner_->config(min_value, max_value);
  this->autotuner_->set_autotuner_id(this->get_name());

  ESP_LOGI(TAG,
           "%s: Autotune has started. This can take a long time depending on the "
           "responsiveness of your system. Your system "
           "output will be altered to deliberately oscillate above and below the setpoint multiple times. "
           "Until your sensor provides a reading, the autotuner may display \'nan\'",
           this->get_name().c_str());

  this->set_interval("autotune-progress", 10000, [this]() {
    if (this->autotuner_ != nullptr && !this->autotuner_->is_finished())
      this->autotuner_->dump_config();
  });
}

}  // namespace pid
}  // namespace esphome
