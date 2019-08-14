#include <WS2812B.h>


#define NUM_LEDS 24
#define TRIGGERVALUE 2000
#define TRIANGLESIDE 503 // found by measuring time delay between 2 microphones in a straigt line
//  or by measuring the distance using the speed of sound (343m/s)
#define MAXTIME 2*TRIANGLESIDE
#define DELAYTIME 20000 // wait before it can respond again
#define LEDDECAY 100



WS2812B strip = WS2812B(NUM_LEDS);

int micA = PA0;
int micB = PA1;
int micC = PA2;

bool micA_trigger = false;
bool micB_trigger = false;
bool micC_trigger = false;

unsigned long micA_timestamp, micB_timestamp, micC_timestamp;

uint8_t ledValues[NUM_LEDS - 1][3];

struct Origin
{
  double angle; // in degree
  double distance;
};

Origin ori = {0, 0};
int ledDecayValue = 0;

Origin calculate(double a, double b, double c, double G) {
  double r_1 = (-2 * a * a * a + a * a * (b + c) - sqrt(3) * sqrt(-((a - b) * (a - b) - G * G) * ((a - c) * (a - c) - G * G) * ((b - c) * (b - c) - G * G)) - (b + c) * (2 * b * b - 3 * b * c + 2 * c * c - G * G) + a * (b * b + c * c + G * G)) / (4 * (a * a + b * b - b * c + c * c - a * (b + c)) - 3 * G * G);
  double r_2 = (-2 * a * a * a + a * a * (b + c) + sqrt(3) * sqrt(-((a - b) * (a - b) - G * G) * ((a - c) * (a - c) - G * G) * ((b - c) * (b - c) - G * G)) - (b + c) * (2 * b * b - 3 * b * c + 2 * c * c - G * G) + a * (b * b + c * c + G * G)) / (4 * (a * a + b * b - b * c + c * c - a * (b + c)) - 3 * G * G);

  if (r_1 == 0 and r_2 == 0) {
    Serial.println("ZERO");
    Origin ori = { -1, -1};
    return ori;
  }

  double i_1 = 2 * a * a * a * (b - c) + 2 * b * b * b * c + 4 * c * c * G * G - 3 * G * G * G * G;
  double i_2 = 3 * a * a * (2 * c * (-b + c) + G * G) + b * b * (-6 * c * c + 5 * G * G);
  double i_3 = sqrt(3) * sqrt(-((a - b) * (a - b) - G * G) * ((a - c) * (a - c) - G * G) * ((b - c) * (b - c) - G * G));
  double i_4 = b * b * b - 3 * b * b * c + 2 * c * c * c + (2 * b + c) * G * G;
  double i_5 = 8 * (a * a + b * b - b * c + c * c - a * (b + c)) * G - 6 * G * G * G;
  double i_7 = 2 * sqrt(3) * (-4 * (a * a + b * b - b * c + c * c - a * (b + c)) * G + 3 * G * G * G);

  double x, y;
  if (r_1 >= r_2) {
    x = (i_1 + i_2 + 2 * b * (2 * c * c * c - 3 * c * G * G + i_3) - 2 * a * (i_4 + i_3)) / i_5;
    y = (6 * b * b * b * (a - c) + 2 * a * i_3 + 2 * b * i_3 - 4 * c * i_3 - 3 * (a - G) * (a + G) * (2 * (a - c) * c + G * G) - 3 * b * b * (2 * (a - c) * (2 * a + c) + G * G) + 6 * b * (a * a * a + a * a * c - 2 * a * c * c + c * G * G)) / i_7;
  } else {
    x = (i_1 + i_2 + 2 * a * i_3 - 2 * a * (i_4) - 2 * b * (-2 * c * c * c + 3 * c * G * G + i_3)) / i_5;
    y = (6 * b * b * b * (a - c) - 2 * a * i_3 - 2 * b * i_3 + 4 * c * i_3 - 3 * (a - G) * (a + G) * (2 * (a - c) * c + G * G) - 3 * b * b * (2 * (a - c) * (2 * a + c) + G * G) + 6 * b * (a * a * a + a * a * c - 2 * a * c * c + c * G * G)) / i_7;
  }

  double angle = atan2 (y - G / (2 * sqrt(3)), x - 0.5 * G) * 180 / PI;
  if (angle < 0) {
    angle += 360;
  }
  double distance = sqrt((x - 0.5 * G) * (x - 0.5 * G) + (y - G / (2 * sqrt(3))) * (y - G / (2 * sqrt(3))));

  Origin ori = {angle, distance};

  return ori;
}

void setup() {
  Serial.begin(9600);
  pinMode(micA, INPUT_ANALOG);
  pinMode(micB, INPUT_ANALOG);
  pinMode(micC, INPUT_ANALOG);

  for (int i = 0; i < NUM_LEDS; i++) {
    for (int j = 0; j < 3; j++) {
      ledValues[i][j] = 0;
    }
  }

  strip.begin();
  strip.show();
  strip.setBrightness(255);
}

void loop() {
  if (!micA_trigger and analogRead(micA) > TRIGGERVALUE and micros() > micA_timestamp + DELAYTIME) {
    micA_timestamp = micros();
    micA_trigger = true;
  }
  if (!micB_trigger and analogRead(micB) > TRIGGERVALUE and micros() > micB_timestamp + DELAYTIME) {
    micB_timestamp = micros();
    micB_trigger = true;
  }
  if (!micC_trigger and analogRead(micC) > TRIGGERVALUE and micros() > micC_timestamp + DELAYTIME) {
    micC_timestamp = micros();
    micC_trigger = true;
  }

  if ((micros() - micA_timestamp > MAXTIME and micA_trigger) or
      (micros() - micB_timestamp > MAXTIME and micB_trigger) or
      (micros() - micC_timestamp > MAXTIME and micC_trigger)) {
    micA_trigger = false;
    micB_trigger = false;
    micC_trigger = false;
  } else if (micA_trigger and micB_trigger and micC_trigger) {
    if (micA_timestamp < micB_timestamp and micA_timestamp < micC_timestamp) {
      // micA triggered first
      ori = calculate(0, micB_timestamp - micA_timestamp, micC_timestamp - micA_timestamp, TRIANGLESIDE);
    } else if (micB_timestamp < micC_timestamp) {
      // micB triggered first
      ori = calculate(micA_timestamp - micB_timestamp, 0, micC_timestamp - micB_timestamp, TRIANGLESIDE);
    } else {
      // micC triggered first
      ori = calculate(micA_timestamp - micC_timestamp, micB_timestamp - micC_timestamp, 0, TRIANGLESIDE);
    }
    micA_trigger = false;
    micB_trigger = false;
    micC_trigger = false;

    if (ori.angle != -1) {
      uint8_t led = map(ori.angle, 0, 360, NUM_LEDS, 0);
      ledValues[(led - 1) % (NUM_LEDS)][2] = 150;
      ledValues[led % (NUM_LEDS)][2] = 255;
      ledValues[(led + 1) % (NUM_LEDS)][2] = 150;
    }
  }



  ledDecayValue++;
  if (ledDecayValue > LEDDECAY) {

    for (int i = 0; i < NUM_LEDS; i++) {
      for (int j = 0; j < 3; j++) {
        if (ledValues[i][j] > 0) {
          ledValues[i][j]--;
        }
      }
      strip.setPixelColor(i, strip.Color(ledValues[i][0], ledValues[i][1], ledValues[i][2]));
    }
    strip.show();
    ledDecayValue = 0;
  }
}
