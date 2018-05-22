#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>


#define DHT11_PIN 7
#define DHTTYPE DHT22
const int XBee_wake = 9;
const int Mini_wake = 2;
#define battery A0
double voltage;
DHT dht(DHT11_PIN, DHTTYPE);
                //
void setup(void)
{
    DDRD &= B00000011;       // set Arduino pins 2 to 7 as inputs, leaves 0 & 1 (RX & TX) as is
    DDRB = B00000000;        // set pins 8 to 13 as inputs
    PORTD |= B11111100;      // enable pullups on pins 2 to 7
    PORTB |= B11111111;      // enable pullups on pins 8 to 13
    dht.begin();
    pinMode(13,OUTPUT);      // set pin 13 as an output so we can use LED to monitor
    digitalWrite(13,HIGH);   // turn pin 13 LED on
    Serial.begin(9600);
    
}
                //
void loop(void)
{
    // Stay awake for 1 second, then sleep.
    // LED turns off when sleeping, then back on upon wake.
   pinMode(XBee_wake, OUTPUT); 
  digitalWrite(XBee_wake, LOW);
  voltage = analogRead(battery);
  voltage = voltage*10 / 1024;
    //Serial.print("@floortemp:");
    //Serial.println(DHT.temperature);
    float humidity = dht.readHumidity();
    float temp = dht.readTemperature();
    Serial.print("#humidity:");
    Serial.print(humidity, 2);
    Serial.print(",");
    Serial.println(temp, 2);
    //Serial.println(voltage,2);
   // pinMode(XBee_wake, INPUT); // put pin in a high impedence state
    delay(650);
    digitalWrite(XBee_wake, HIGH);
    pinMode(XBee_wake, INPUT);
    
    sleepNow();
}
                //
void sleepNow(void)
{
    // Set pin 2 as interrupt and attach handler:
    attachInterrupt(0, pinInterrupt, LOW);
    //delay(100);
    //
    // Choose our preferred sleep mode:
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    //
    // Set sleep enable (SE) bit:
    sleep_enable();
    //
    // Put the device to sleep:
    digitalWrite(13,LOW);   // turn LED off to indicate sleep
    sleep_mode();
    //
    // Upon waking up, sketch continues from this point.
    sleep_disable();
    digitalWrite(13,HIGH);   // turn LED on to indicate awake
}
                //
void pinInterrupt(void)
{
    detachInterrupt(0);
}
