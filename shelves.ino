#include <Arduino.h>
#include <MPR121.h>  // https://github.com/BareConductive/mpr121.git
#include <MPR121_Datastream.h>
#include <Wire.h>

//#define DEBUG_SERIAL

//
// TUNING CONSTANTS
//
constexpr bool ENABLE_MPR121 = false;
constexpr bool SLAVE_I2C_ADDRESS = 1;
constexpr uint8_t PWM_STEPS = 64;  // NOTE: also change Shelf::set()

// Slider tuning constants
constexpr float A_MAX = -40.0;  // The "fully touched" value
constexpr float B_MAX = -38.0;  // The B sensor is slightly weaker for some reason, probably the longer wire.
constexpr size_t NUM_SMOOTHING_SAMPLES = 20;
constexpr uint8_t LED_PINS[] = {4, 5, 6, 7, 8, 11};  // ELE9 and 10 have bugs
constexpr uint8_t NUM_LEDS = (sizeof(LED_PINS) / sizeof(LED_PINS[0]));


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

  void set(uint8_t value) {
    value_ = value >> 2;  // [0, 255] --> [0, 63]
  }

private:
  volatile uint8_t& port_;
  const uint8_t on_mask_;
  const uint8_t off_mask_;
  const uint8_t arduino_pin_;
  volatile uint8_t value_ = 0;
};

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

    for (int pin : LED_PINS) {
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
    nominal_a_value = MPR121.getFilteredData(0);
    nominal_b_value = MPR121.getFilteredData(1);
  }

  bool read(float& value) {
    if(!MPR121.isInited()) {
      Serial.println("MPR121 Not inited: Refusing to read");
      return false;
    }
    bool touched = read_raw_slider(current_value);
    value = rolling_filter_value(current_value);
    return touched;
  }

  // Simple display update routine that performs delta updates to only
  // the needed LED's. I found the I2C requests to be slow.
  void update_display(float value) {
    if(!MPR121.isInited()) {
      Serial.println("MPR121 Not inited: Refusing to update display");
      return;
    }

    value = max(0, min(value, 1.0));

    // Compute new LED values
    uint8_t new_values[NUM_LEDS] = {0};
    const uint8_t in_led = value * NUM_LEDS;  // integer flooring
    new_values[in_led] = 255;  // The LED that the value is in is always 100%
    const float partial = (value * NUM_LEDS) - in_led;  // [0, 1) of where the value lands within the LED

    if (partial > 0.5 && (in_led != (NUM_LEDS - 1)))
    {
      new_values[in_led + 1] = ((partial - 0.5) / 0.5) * 255;
    }
    else if (partial < 0.5 && (in_led != 0))
    {
      new_values[in_led - 1] = ((-1.0 * (partial / 0.5)) + 1.0) * 255;
    }

    // Commit the new states
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
      const uint8_t new_value = max(new_values[i], 0);
      if (led_values[i] != new_value)
      {
        MPR121.analogWrite(LED_PINS[i], new_value);
        led_values[i] = new_value;
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
    float a = (MPR121.getFilteredData(0) - nominal_a_value) / A_MAX;
    float b = (MPR121.getFilteredData(1) - nominal_b_value) / B_MAX;
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
      if (value < 0.1) {
        value = 0.0;
      } else if (value > 0.9) {
        value = 1.0;
      }
    }

    #ifdef DEBUG_SERIAL
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

  /**
   * Adds the value to a simple rolling window average filter.
   * This not only smoothes out some of the jitter in the value,
   * but also gives it some nice inertia/LERPing when transitioning
   * between distant values.
   */
  float rolling_filter_value(float value)
  {
    history[history_cursor++] = value;
    if (history_cursor >= NUM_SMOOTHING_SAMPLES)
    {
      history_cursor = 0;
    }

    float v = 0.0;
    for (size_t i = 0; i < NUM_SMOOTHING_SAMPLES; i++)
    {
      v += history[i];
    }
    return v / NUM_SMOOTHING_SAMPLES;
  }

private:
  // Ambient un-touched capacitance values. Anything beyond which is a touch.
  int nominal_a_value = 0;
  int nominal_b_value = 0;

  // The current un-smoothed target value of the slider
  float current_value = 0.0;

  // The history buffer used to smooth and LERP the value around on the slider
  size_t history_cursor = 0;
  float history[NUM_SMOOTHING_SAMPLES] = {0.0};

  // LED information on the MPR121 display. Used for delta updates.
  uint8_t led_values[NUM_LEDS];
};

//
// GLOBALS
//
bool master = false;
TouchSlider slider;

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

void setup()
{
  Serial.begin(9600);

  // Read personality pin
  pinMode(2, INPUT_PULLUP);
  master = (digitalRead(2) == LOW);
  pinMode(13, OUTPUT);

  if (master) {
    // Display master LED
    digitalWrite(13, HIGH);

    if (ENABLE_MPR121) {
      slider.init();  // Calls I2C Wire.begin(); for us
    } else {
      Wire.begin();
    }

    for (uint8_t i = 0; i < NUM_SHELVES; i++) {
      pinMode(shelves[i].get_arduino_pin(), OUTPUT);
    }

    for (float v = 0.0; v <= 1.0; v += 0.05)
    {
      slider.update_display(v);
      delay(8);
    }

    for (float v = 1.0; v >= 0.0; v -= 0.05)
    {
      slider.update_display(v);
      delay(8);
    }
  } else {
    Wire.begin(SLAVE_I2C_ADDRESS);
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
  uint8_t shelf_value = 0;  // 0-255

  if (master) {
    Wire.beginTransmission(SLAVE_I2C_ADDRESS);
    Wire.write(255);
    Wire.endTransmission();
    digitalWrite(13, HIGH);
    delay(1000);
    Wire.beginTransmission(SLAVE_I2C_ADDRESS);
    Wire.write(0);
    Wire.endTransmission();
    digitalWrite(13, LOW);
    delay(1000);
    return;

    // We are the master

    // TODO: transact 8-bit ints from the slider to make things consistent
    static float value = 0;
    slider.read(value);
    slider.update_display(value);

    // Convert to single byte [0-255] value
    shelf_value = max(0, min(fmap(value, 0.0, 1.0, 0, 255), 255));

    Wire.beginTransmission(0 /* broadcast address */);
    Wire.write(shelf_value);
    Wire.endTransmission();
  } else {
    // We are the slave
    while(Wire.available())
    {
      shelf_value = Wire.read();
      Serial.println(shelf_value);
    }
    if (shelf_value == 255) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
  }

  for (int i = 0; i < NUM_SHELVES; i++) {
    shelves[i].set(shelf_value);
  }

  delay(10);  // milliseconds
}
