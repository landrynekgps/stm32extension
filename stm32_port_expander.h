// Must disable logging if using logging in main.cpp or in other custom components for the
//  __c causes a section type conflict with __c thingy
// you can enable logging and use it if you enable this in logger:
/*
logger:
  level: DEBUG
  esp8266_store_log_strings_in_flash: False
  */

//#define APE_LOGGING

// take advantage of LOG_ defines to decide which code to include
#ifdef LOG_BINARY_OUTPUT
#define STM32_BINARY_OUTPUT
#endif
#ifdef LOG_BINARY_SENSOR
#define STM32_BINARY_SENSOR
#endif
#ifdef LOG_SENSOR
#define STM32_SENSOR
#endif

static const char *TAGstm32 = "stm32";

#define STM32_CMD_DIGITAL_READ 0
#define STM32_CMD_WRITE_ANALOG 2
#define STM32_CMD_WRITE_DIGITAL_HIGH 3
#define STM32_CMD_WRITE_DIGITAL_LOW 4
#define STM32_CMD_SETUP_PIN_OUTPUT 5
#define STM32_CMD_SETUP_PIN_INPUT_PULLUP 6
#define STM32_CMD_SETUP_PIN_INPUT 7
// 8 analog registers.. A0 to A7
// A4 and A5 not supported due to I2C
#define CMD_ANALOG_READ_A0 0b1000 // 0x8
// ....
#define CMD_ANALOG_READ_A7 0b1111 // 0xF

#define CMD_SETUP_ANALOG_INTERNAL 0x10
#define CMD_SETUP_ANALOG_DEFAULT 0x11

#define get_stm32(constructor) static_cast<STM32Extension *>(constructor.get_component(0))

#define stm32_binary_output(stm32, pin) get_stm32(stm32)->get_binary_output(pin)
#define stm32_binary_sensor(stm32, pin) get_stm32(stm32)->get_binary_sensor(pin)
#define stm32_analog_input(stm32, pin) get_stm32(stm32)->get_analog_input(pin)

#ifdef STM32_BINARY_OUTPUT
class Stm32BinaryOutput : public output::BinaryOutput
{
public:
  Stm32BinaryOutput(STM32Extension *parent, uint8_t pin)
  {
    this->parent_ = parent;
    this->pin_ = pin;
  }
  void write_state(bool state) override;
  uint8_t get_pin() { return this->pin_; }

protected:
  STM32Extension *parent_;
  uint8_t pin_;
  // Pins are setup as output after the state is written, STM32 has no open drain outputs, after setting an output it will either sink or source thus activating outputs writen to false during a flick.
  bool setup_{true};
  bool state_{false};

  friend class STM32Extension;
};
#endif

#ifdef STM32_BINARY_SENSOR
class Stm32BinarySensor : public binary_sensor::BinarySensor
{
public:
  Stm32BinarySensor(STM32Extension *parent, uint8_t pin)
  {
    this->pin_ = pin;
  }
  uint8_t get_pin() { return this->pin_; }

protected:
  uint8_t pin_;
};
#endif

#ifdef STM32_SENSOR
class Stm32AnalogInput : public sensor::Sensor
{
public:
  Stm32AnalogInput(STM32Extension *parent, uint8_t pin)
  {
    this->pin_ = pin;
  }
  uint8_t get_pin() { return this->pin_; }

protected:
  uint8_t pin_;
};
#endif

class STM32Extension : public Component, public I2CDevice
{
public:
  STM32Extension(I2CBus *bus, uint8_t address, bool vref_default = false)
  {
    set_i2c_address(address);
    set_i2c_bus(bus);
    this->vref_default_ = vref_default;
  }
  void setup() override
  {
#ifdef APE_LOGGING
    ESP_LOGCONFIG(TAGstm32, "Setting up STM32Extension at %#02x ...", address_);
#endif

    /* We cannot setup as usual as STM32 boots later than esp8266
            Poll i2c bus for our STM32 for a n seconds instead of failing fast,
            also this is important as pin setup (INPUT_PULLUP, OUTPUT it's done once)
        */
    this->configure_timeout_ = millis() + 5000;
  }
  void loop() override
  {
    if (millis() < this->configure_timeout_)
    {
      bool try_configure = millis() % 100 > 50;
      if (try_configure == this->configure_)
        return;
      this->configure_ = try_configure;

      if (ERROR_OK == this->read_register(STM32_CMD_DIGITAL_READ, const_cast<uint8_t *>(this->read_buffer_), 3))
      {
#ifdef APE_LOGGING
        ESP_LOGCONFIG(TAGstm32, "STM32Extension found at %#02x", address_);
#endif
        delay(10);
        if (this->vref_default_)
        {
          this->write_register(CMD_SETUP_ANALOG_DEFAULT, nullptr, 0); // 0: unused
        }

        // Config success
        this->configure_timeout_ = 0;
        this->status_clear_error();
#ifdef STM32_BINARY_SENSOR
        for (Stm32BinarySensor *pin : this->input_pins_)
        {
          App.feed_wdt();
          uint8_t pinNo = pin->get_pin();
#ifdef APE_LOGGING
          ESP_LOGCONFIG(TAGstm32, "Setup input pin %d", pinNo);
#endif
          this->write_register(STM32_CMD_SETUP_PIN_INPUT_PULLUP, &pinNo, 1);
          delay(20);
        }
#endif
#ifdef STM32_BINARY_OUTPUT
        for (Stm32BinaryOutput *output : this->output_pins_)
        {
          if (!output->setup_)
          { // this output has a valid value already
            this->write_state(output->pin_, output->state_, true);
            App.feed_wdt();
            delay(20);
          }
        }
#endif
#ifdef STM32_SENSOR
        for (Stm32AnalogInput *sensor : this->analog_pins_)
        {
          App.feed_wdt();
          uint8_t pinNo = sensor->get_pin();
#ifdef APE_LOGGING
          ESP_LOGCONFIG(TAGstm32, "Setup analog input pin %d", pinNo);
#endif
          this->write_register(STM32_CMD_SETUP_PIN_INPUT, &pinNo, 1);
          delay(20);
        }
#endif
        return;
      }
      // Still not answering
      return;
    }
    if (this->configure_timeout_ != 0 && millis() > this->configure_timeout_)
    {
#ifdef APE_LOGGING
      ESP_LOGE(TAGstm32, "STM32Extension NOT found at %#02x", address_);
#endif
      this->mark_failed();
      return;
    }

#ifdef STM32_BINARY_SENSOR
    if (ERROR_OK != this->read_register(STM32_CMD_DIGITAL_READ, const_cast<uint8_t *>(this->read_buffer_), 3))
    {
#ifdef APE_LOGGING
      ESP_LOGE(TAGstm32, "Error reading. Reconfiguring pending.");
#endif
      this->status_set_error();
      this->configure_timeout_ = millis() + 5000;
      return;
    }
    for (Stm32BinarySensor *pin : this->input_pins_)
    {
      uint8_t pinNo = pin->get_pin();

      uint8_t bit = pinNo % 8;
      uint8_t value = pinNo < 8 ? this->read_buffer_[0] : pinNo < 16 ? this->read_buffer_[1] : this->read_buffer_[2];
      bool ret = value & (1 << bit);
      if (this->initial_state_)
        pin->publish_initial_state(ret);
      else
        pin->publish_state(ret);
    }
#endif
#ifdef STM32_SENSOR
    for (Stm32AnalogInput *pin : this->analog_pins_)
    {
      uint8_t pinNo = pin->get_pin();
      pin->publish_state(analogRead(pinNo));
    }
#endif
    this->initial_state_ = false;
  }

#ifdef STM32_SENSOR
  uint16_t analogRead(uint8_t pin)
  {
    bool ok = (ERROR_OK == this->read_register((uint8_t)(CMD_ANALOG_READ_A0 + pin), const_cast<uint8_t *>(this->read_buffer_), 2));
#ifdef APE_LOGGING
    ESP_LOGVV(TAGstm32, "analog read pin: %d ok: %d byte0: %d byte1: %d", pin, ok, this->read_buffer_[0], this->read_buffer_[1]);
#endif
    uint16_t value = this->read_buffer_[0] | ((uint16_t)this->read_buffer_[1] << 8);
    return value;
  }
#endif

#ifdef STM32_BINARY_OUTPUT
  output::BinaryOutput *get_binary_output(uint8_t pin)
  {
    Stm32BinaryOutput *output = new Stm32BinaryOutput(this, pin);
    output_pins_.push_back(output);
    return output;
  }
#endif
#ifdef STM32_BINARY_SENSOR
  binary_sensor::BinarySensor *get_binary_sensor(uint8_t pin)
  {
    Stm32BinarySensor *binarySensor = new Stm32BinarySensor(this, pin);
    input_pins_.push_back(binarySensor);
    return binarySensor;
  }
#endif
#ifdef STM32_SENSOR
  sensor::Sensor *get_analog_input(uint8_t pin)
  {
    Stm32AnalogInput *analogInput = new Stm32AnalogInput(this, pin);
    analog_pins_.push_back(analogInput);
    return analogInput;
  }
#endif

protected:
  bool vref_default_;
  uint32_t configure_timeout_;
  bool configure_{true};
  bool initial_state_{true};
  uint8_t read_buffer_[3];
  std::vector<Stm32BinaryOutput *> output_pins_;
  std::vector<Stm32BinarySensor *> input_pins_;
  std::vector<Stm32AnalogInput *> analog_pins_;
};
