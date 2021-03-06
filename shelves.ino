#include <Arduino.h>
#include <Wire.h>
#include "src/MPR121/MPR121.h"  // https://github.com/BareConductive/mpr121.git

//#define SLIDER_DEBUG_SERIAL

//
// TUNING CONSTANTS
//
constexpr uint8_t PWM_STEPS = 64;  // NOTE: also change Shelf::set()
constexpr size_t NUM_SMOOTHING_SAMPLES = 20;

// Slider tuning constants
constexpr float A_MAX = -48.0;  // The "fully touched" value
constexpr float B_MAX = -48.0;
constexpr uint8_t MPR121_LED_PINS[] = {4, 5, 6, 7, 8, 11};  // ELE9 and 10 have bugs
constexpr uint8_t MPR121_NUM_LEDS = (sizeof(MPR121_LED_PINS) / sizeof(MPR121_LED_PINS[0]));
constexpr unsigned long TAP_MIN_MS = 30;
constexpr unsigned long TAP_MAX_MS = 300;
constexpr float DISPLAY_IDLE_BRIGHTNESS = 0.1;
constexpr float DISPLAY_FADE_INCREMENT = 0.01;


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Configuration and control for a single LED output channel
 */
class Shelf {
public:
  Shelf(volatile uint8_t& port, uint8_t bit, uint8_t arduino_pin)
    : port_(port)
    , on_mask_(1 << bit)  // pre-compute the masks, to shave some cycles in the ISR
    , off_mask_(~(1 << bit))
    , arduino_pin_(arduino_pin)
    , value_(0)
  {}

  uint8_t get_arduino_pin() const { return arduino_pin_; }

  void pwm(uint8_t count) {
    if (count == 0 && value_ != 0)
    {
      port_ |= on_mask_;  // ON
    }
    else if (count >= value_)
    {
      port_ &= off_mask_;  // OFF
    }
  }

  void set(float value) {
    const uint8_t MAX_VALUE = PWM_STEPS - 1;
    value_ = max(0, min(fmap(value, 0.0, 1.0, 0, MAX_VALUE), MAX_VALUE));
  }

private:
  volatile uint8_t& port_;
  const uint8_t on_mask_;
  const uint8_t off_mask_;
  const uint8_t arduino_pin_;
  volatile uint8_t value_ = 0;
};

/**
 * Handles reading from the two touch slider strips and updating the display on the MPR121
 */
class TouchSlider {
public:
  void init() {
    if (!MPR121.begin(0x5A)) {
      Serial.println("Failed to init MPR121 library");
      return;
    }

    MPR121.setFFI(FFI_10);
    MPR121.setSFI(SFI_10);
    MPR121.setGlobalCDT(CDT_500NS);
    MPR121.setGlobalCDC(63);
    MPR121.setNumDigPins(8);

    for (int pin : MPR121_LED_PINS) {
      MPR121.pinMode(pin, OUTPUT);
    }

    calibrate();
  }

  void calibrate() {
    if(!MPR121.isInited()) {
      Serial.println("MPR121 Not inited: Refusing to calibrate");
      return;
    }
    MPR121.updateFilteredData();
    nominal_a_value_ = MPR121.getFilteredData(0);
    nominal_b_value_ = MPR121.getFilteredData(1);
  }

  bool read(float& value) {
    if(!MPR121.isInited()) {
      Serial.println("MPR121 Not inited: Refusing to read");
      return false;
    }

    // Raw slider read
    bool touched = read_raw_slider(current_value_);

    // Tap detection. Look for touch begin/end
    if (touched && !last_touched_) {
      // Tap begin
      touch_start_ms_ = millis();
    } else if (!touched && last_touched_) {
      // Tap end
      const unsigned long now_ms = millis();

      // millis() rolls over after ~50 days. It's very unlikely that this
      // will happen during our touch, but just in case, let's catch it.
      // This code assumes that a tap is <50 days long.
      unsigned long touch_duration_ms = 0;
      if (now_ms < touch_start_ms_) {
        constexpr unsigned long MILLIS_MAX = -1;
        touch_duration_ms = (MILLIS_MAX - touch_start_ms_) + now_ms;
      } else {
        // The simple case
        touch_duration_ms = now_ms - touch_start_ms_;
      }

      if ((touch_duration_ms >= TAP_MIN_MS) && (touch_duration_ms <= TAP_MAX_MS)) {
        // We've received a tap. Send the light value to the side that the user tapped
        current_value_ = current_value_ < 0.5 ? 0.0 : 1.0;
      }
    }

    last_touched_ = touched;
    value = current_value_;
    return touched;
  }

  // Simple display update routine that performs delta updates to only
  // the needed LED's. I found the I2C requests to be slow.
  //   value      [0.0, 1.0]
  //   brightness [0.0, 1.0]
  void update_display(float value, float brightness) {
    if(!MPR121.isInited()) {
      Serial.println("MPR121 Not inited: Refusing to update display");
      return;
    }

    value = max(0, min(value, 1.0));

    // Compute new LED values
    uint8_t new_values[MPR121_NUM_LEDS] = {0};
    const uint8_t in_led = value * MPR121_NUM_LEDS;  // integer flooring
    new_values[in_led] = brightness * 255;  // The LED that the value is in is always 100%
    const float partial = (value * MPR121_NUM_LEDS) - in_led;  // [0, 1) of where the value lands within the LED

    if (partial > 0.5 && (in_led != (MPR121_NUM_LEDS - 1)))
    {
      new_values[in_led + 1] = ((partial - 0.5) / 0.5) * brightness * 255;
    }
    else if (partial < 0.5 && (in_led != 0))
    {
      new_values[in_led - 1] = ((-1.0 * (partial / 0.5)) + 1.0) * brightness * 255;
    }

    // Commit the new states
    for (uint8_t i = 0; i < MPR121_NUM_LEDS; i++)
    {
      const uint8_t new_value = max(new_values[i], 0);
      if (led_values_[i] != new_value)
      {
        MPR121.analogWrite(MPR121_LED_PINS[i], new_value);
        led_values_[i] = new_value;
      }
    }
  }

private:
  /**
   * Reads the capactive ramp slider and returns a boolean for
   * touched/not-touched. When touched, "value" is populated with
   * a [0,1] for the slider. "value" is unchanged when not-touched.
   */
  bool read_raw_slider(float& value)
  {
    MPR121.updateFilteredData();

    // Read sensor data and normalize to [0,1] (0 = not touched, 1 = touched)
    float a = (MPR121.getFilteredData(0) - nominal_a_value_) / A_MAX;
    float b = (MPR121.getFilteredData(1) - nominal_b_value_) / B_MAX;
    a = constrain(a, 0.0, 1.0);
    b = constrain(b, 0.0, 1.0);

    // The sensors are far more sensitive at their extremes,
    // and not as much at their tips. Attempt to linearize this.
    a = pow(a, 0.7);  // sqrt() (pow(0.5)) was too harsh, and amplified the low-end too much
    b = pow(b, 0.7);

    bool touched = ((a + b) >= 1.0);
    if (touched)
    {
      value = ((b - a) + 1) / 2.0;
      if (value < 0.15) {
        value = 0.0;
      } else if (value > 0.85) {
        value = 1.0;
      }
    }

    #ifdef SLIDER_DEBUG_SERIAL
    static uint8_t i = 0;
    if (i++ > 25)
    {
      Serial.print("Touched = ");
      Serial.print(touched);
      Serial.print("  value = ");
      Serial.print(value);
      Serial.print("  sum = ");
      Serial.print(a + b);
      Serial.print("  A = ");
      Serial.print(a);
      Serial.print("  B = ");
      Serial.println(b);
      i = 0;
    }
    #endif

    return touched;
  }

private:
  // Ambient un-touched capacitance values. Anything beyond which is a touch.
  int nominal_a_value_ = 0;
  int nominal_b_value_ = 0;

  // The current un-smoothed target value of the slider
  float current_value_ = 0.0;

  // LED information on the MPR121 display. Used for delta updates.
  uint8_t led_values_[MPR121_NUM_LEDS] = {};

  bool last_touched_ = false;  // used for touch start/end edge detection
  unsigned long touch_start_ms_ = 0;
};


/**
 * Simple rolling window average filter.
 * This not only smoothes out some of the jitter in the value,
 * but also gives it some nice inertia/LERPing when transitioning
 * between distant values.
 */
class Smoother {
public:
  float operator()(float value) {
    history_[history_cursor_++] = value;
    if (history_cursor_ >= NUM_SMOOTHING_SAMPLES)
    {
      history_cursor_ = 0;
    }

    float v = 0.0;
    for (size_t i = 0; i < NUM_SMOOTHING_SAMPLES; i++)
    {
      v += history_[i];
    }
    return v / NUM_SMOOTHING_SAMPLES;
  }

private:
  // The history buffer used to smooth and LERP the value around on the slider
  size_t history_cursor_ = 0;
  float history_[NUM_SMOOTHING_SAMPLES] = {0.0};
};


//
// GLOBALS
//
bool master = false;
TouchSlider slider;
Smoother smoother;
float display_brightness = DISPLAY_IDLE_BRIGHTNESS;

// Used by the ISR to quickly set output pins
Shelf shelves[] = {
  {PORTB, 4, 12},  // Shelf 0, Pin D12
  {PORTB, 3, 11},  // Shelf 1, Pin D11
  {PORTB, 2, 10},  // Shelf 2, Pin D10
  {PORTB, 1, 9},   // Shelf 3, Pin D9
  {PORTB, 0, 8},   // Shelf 4, Pin D8
  {PORTD, 7, 7},   // Shelf 5, Pin D7
  {PORTD, 6, 6},   // Shelf 6, Pin D6
};
constexpr uint8_t NUM_SHELVES = (sizeof(shelves) / sizeof(shelves[0]));
volatile uint8_t isr_count = 0xff;

// ~8KHz ISR
ISR(TIMER2_COMPA_vect)
{
  ++isr_count;
  if (isr_count >= PWM_STEPS) {
    isr_count = 0;
  }

  // Update all outputs
  for (uint8_t i = 0; i < NUM_SHELVES; i++) {
    shelves[i].pwm(isr_count);
  }
}

void uart_send(float value) {
  const uint8_t int_value = round(fmap(value, 0.0, 1.0, 0, 255));
  Serial.write('[');
  Serial.write(int_value);
  Serial.write(']');
}

void uart_receive(float& value) {
  while (Serial.available() > 0) {
    int c = Serial.read();
    // If we find the start character, perform two more reads.
    // One for the value, and one for the end character.
    if (c == '[') {
      const uint8_t new_value = Serial.read();
      if (Serial.read() == ']') {
        // End character found, commit the value
        value = fmap(new_value, 0, 255, 0.0, 1.0);
      } // else discard everything and carry on
    }
  }
}

void setup()
{
  Serial.begin(115200);

  // Read personality pin
  pinMode(2, INPUT_PULLUP);
  master = (digitalRead(2) == LOW);
  pinMode(13, OUTPUT);

  if (master) {
    // Display master LED
    digitalWrite(13, HIGH);

    slider.init();
    slider.update_display(0, DISPLAY_IDLE_BRIGHTNESS);

    for (uint8_t i = 0; i < NUM_SHELVES; i++) {
      pinMode(shelves[i].get_arduino_pin(), OUTPUT);
    }
  }

  // 16mhz (Arduino Clock) / 8 (Prescalar) / 255 (Output Compare Register) / 64 (Software Counter)
  //   =
  // ~120Hz PWM with 64 steps

  TIFR2 = (1 << TOV2);    /* clear interrupt flag */
  TCCR2B = (1 << CS21);   /* start timer (ck/8 prescalar) */
  TCCR2A = (1 << WGM21);  /* CTC mode */
  OCR2A = 255;            /* Output Compare Register */
  TIMSK2 = (1 << OCIE2A); /* enable timer2 output compare match interrupt */
}

void loop()
{
  float light_value = 0.0;

  if (master) {
    // We are the master
    float slider_value = 0.0;
    const bool touched = slider.read(slider_value);
    light_value = smoother(slider_value);
    const bool value_achieved = (light_value == slider_value);

    if (touched) {
      display_brightness = 1.0;
    } else if (value_achieved && display_brightness > DISPLAY_IDLE_BRIGHTNESS) {
      // Fade out the display slowly once the touch is released and the slider is done sliding
      display_brightness -= DISPLAY_FADE_INCREMENT;
      display_brightness = max(0, display_brightness);
    }

    slider.update_display(light_value, display_brightness);
    #ifndef SLIDER_DEBUG_SERIAL
    uart_send(light_value);
    #endif
  } else {
    uart_receive(light_value);
  }

  // The higher the shelf value, the more shelves turn on
  constexpr float VALUE_PER_SHELF = 1.0 / NUM_SHELVES;
  float value = light_value;
  for (int i = 0; i < NUM_SHELVES; i++) {
    shelves[i].set(min(fmap(value, 0, VALUE_PER_SHELF, 0.0, 1.0), 1.0));
    value = max(0, value - VALUE_PER_SHELF);
  }

  delay(10);  // milliseconds
}
