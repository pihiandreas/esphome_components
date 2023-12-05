#include "empc.h"
#include "esphome/core/log.h"

namespace esphome {
namespace empc {

static const char *const TAG = "ep_em_pulsecounter";

// valid for HLW8012 and CSE7759
static const uint32_t EMPC_CLOCK_FREQUENCY = 3579000;

void EmPcComponent::setup() {
  float reference_voltage = 0;
  ESP_LOGCONFIG(TAG, "Setting up Pulse Counter Energy Meter...");
  this->sel_pin_->setup();
  this->sel_pin_->digital_write(this->current_mode_);
  this->cf_store_.pulse_counter_setup(this->cf_pin_);
  this->cf1_store_.pulse_counter_setup(this->cf1_pin_);

  // Initialize multipliers
  if (this->sensor_model_ == EMPC_SENSOR_MODEL_BL0937) {
    reference_voltage = 1.218f;
    this->power_multiplier_ =
        reference_voltage * reference_voltage * this->voltage_divider_ / this->current_resistor_ / 1721506.0f;
    this->current_multiplier_ = reference_voltage / this->current_resistor_ / 94638.0f;
    this->voltage_multiplier_ = reference_voltage * this->voltage_divider_ / 15397.0f;
  } else {
    // HLW8012 and CSE7759 have same reference specs
    reference_voltage = 2.43f;
    this->power_multiplier_ = reference_voltage * reference_voltage * this->voltage_divider_ / this->current_resistor_ *
                              64.0f / 24.0f / EMPC_CLOCK_FREQUENCY;
    this->current_multiplier_ = reference_voltage / this->current_resistor_ * 512.0f / 24.0f / EMPC_CLOCK_FREQUENCY;
    this->voltage_multiplier_ = reference_voltage * this->voltage_divider_ * 256.0f / EMPC_CLOCK_FREQUENCY;
  }
}
void EmPcComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "EMPC:");
  LOG_PIN("  SEL Pin: ", this->sel_pin_)
  LOG_PIN("  CF Pin: ", this->cf_pin_)
  LOG_PIN("  CF1 Pin: ", this->cf1_pin_)
  ESP_LOGCONFIG(TAG, "  Change measurement mode every %" PRIu32, this->change_mode_every_);
  ESP_LOGCONFIG(TAG, "  Current resistor: %.1f mΩ", this->current_resistor_ * 1000.0f);
  ESP_LOGCONFIG(TAG, "  Voltage Divider: %.1f", this->voltage_divider_);
  LOG_UPDATE_INTERVAL(this)
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_)
  LOG_SENSOR("  ", "Current", this->current_sensor_)
  LOG_SENSOR("  ", "Power", this->power_sensor_)
  LOG_SENSOR("  ", "Energy", this->energy_sensor_)
}
float EmPcComponent::get_setup_priority() const { return setup_priority::DATA; }
void EmPcComponent::update() {
  // HLW8012 has 50% duty cycle
  pulse_counter::pulse_counter_t raw_cf = this->cf_store_.read_raw_value();
  pulse_counter::pulse_counter_t raw_cf1 = this->cf1_store_.read_raw_value();
  float cf_hz = raw_cf / (this->get_update_interval() / 1000.0f);
  if (raw_cf <= 1) {
    // don't count single pulse as power
    cf_hz = 0.0f;
  }
  float cf1_hz = raw_cf1 / (this->get_update_interval() / 1000.0f);
  if (raw_cf1 <= 1) {
    // don't count single pulse as anything
    cf1_hz = 0.0f;
  }

  if (this->nth_value_++ < 2) {
    ESP_LOGD(TAG, "nth_value_(%d) too high, skipping..", nth_value_);
    return;
  }

  float power = cf_hz * this->power_multiplier_;

  if (this->change_mode_at_ != 0 || this->change_mode_every_ == 0) {
    // Only read cf1 after one cycle. Apparently it's quite unstable after being changed.
    // Go ahead and read if change_mode_every_ == 0 (no changing of mode)
    if (this->current_mode_) {
      float current = cf1_hz * this->current_multiplier_;
      // ESP_LOGD(TAG, "Got power=%.1fW, current=%.1fA", power, current);
      ESP_LOGD(TAG, "Got power=%.1fW, current=%03.1fA(read), voltage=%03.1fV(calc)", power, current, (current > 0.0f ? power / current : 0.0f));
      if (this->current_sensor_ != nullptr) {
        this->current_sensor_->publish_state(current);
      }
    } else {
      float voltage = cf1_hz * this->voltage_multiplier_;
      // ESP_LOGD(TAG, "Got power=%.1fW, voltage=%.1fV", power, voltage);
      ESP_LOGD(TAG, "Got power=%.1fW, current=%03.1fA(calc), voltage=%03.1fV(read)", power, (voltage > 0.0f ? power / voltage : 0.0f), voltage);
      if (this->voltage_sensor_ != nullptr) {
        this->voltage_sensor_->publish_state(voltage);
      }
    }
  }

  if (this->power_sensor_ != nullptr) {
    this->power_sensor_->publish_state(power);
  }

  if (this->energy_sensor_ != nullptr) {
    cf_total_pulses_ += raw_cf;
    float energy = cf_total_pulses_ * this->power_multiplier_ / 3600;
    this->energy_sensor_->publish_state(energy);
  }

  if (this->change_mode_every_ != 0 && this->change_mode_at_++ == this->change_mode_every_) {
    this->current_mode_ = !this->current_mode_;
    ESP_LOGV(TAG, "Changing mode to %s mode", this->current_mode_ ? "CURRENT" : "VOLTAGE");
    this->change_mode_at_ = 0;
    this->sel_pin_->digital_write(this->current_mode_);
  }
}

}  // namespace empc
}  // namespace esphome
