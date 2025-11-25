#include <Arduino.h>
#include <SoftwareSerial.h>

// ---- Motor pins (your mapping) ----
const int IN1 = 8;  // Left A
const int IN2 = 9;  // Left B
const int EN1 = 10; // Left enable (PWM-capable)

const int IN3 = 5;  // Right A
const int IN4 = 6;  // Right B
const int EN2 = 11; // Right enable (PWM-capable)

// ---- Bluetooth (HC-05) on D2 (RX), D3 (TX) ----
// HC-05 TX -> D2, HC-05 RX <- D3 (through voltage divider)
SoftwareSerial BT(2, 3);

// ---- Motor helpers ----
void leftStop()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, 0);
}

void rightStop()
{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN2, 0);
}

void leftDrive(int pwm)
{
    // pwm in [0, 255]
    if (pwm < 0)
        pwm = 0;
    if (pwm > 255)
        pwm = 255;
    analogWrite(EN1, pwm);
}

void rightDrive(int pwm)
{
    // pwm in [0, 255]
    if (pwm < 0)
        pwm = 0;
    if (pwm > 255)
        pwm = 255;
    analogWrite(EN2, pwm);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    // Clamp to [-255, 255]
    if (leftSpeed > 255)
        leftSpeed = 255;
    if (leftSpeed < -255)
        leftSpeed = -255;
    if (rightSpeed > 255)
        rightSpeed = 255;
    if (rightSpeed < -255)
        rightSpeed = -255;

    // ---- Left motor ----
    if (leftSpeed == 0)
    {
        leftStop();
    }
    else if (leftSpeed > 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        leftDrive(leftSpeed);
    }
    else
    { // leftSpeed < 0
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        leftDrive(-leftSpeed);
    }

    // ---- Right motor ----
    if (rightSpeed == 0)
    {
        rightStop();
    }
    else if (rightSpeed > 0)
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        rightDrive(rightSpeed);
    }
    else
    { // rightSpeed < 0
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        rightDrive(-rightSpeed);
    }
}

void stopAll()
{
    leftStop();
    rightStop();
}

// ---- Command parsing ----
// Expected line format:  V <left> <right>
// Example: "V 120 120\n" or "V -150 150\n"
void handleLine(const String &line)
{
    if (line.length() == 0)
        return;

    char cmd;
    int left, right;

    if (sscanf(line.c_str(), "%c %d %d", &cmd, &left, &right) == 3 && cmd == 'V')
    {
        setMotorSpeeds(left, right);
        Serial.print("CMD: V ");
        Serial.print(left);
        Serial.print(" ");
        Serial.println(right);
    }
    else
    {
        // Unknown / malformed line, ignore
        Serial.print("IGNORED: ");
        Serial.println(line);
    }
}

// Process data from a given Stream (BT or USB)
void processStream(Stream &stream, String &buffer)
{
    while (stream.available() > 0)
    {
        char c = stream.read();

        if (c == '\r')
        {
            // ignore CR
            continue;
        }
        else if (c == '\n')
        {
            // end of line
            handleLine(buffer);
            buffer = "";
        }
        else
        {
            // accumulate, avoid overflow
            if (buffer.length() < 31)
            {
                buffer += c;
            }
        }
    }
}

// Buffers for each input stream
String btBuffer;
String usbBuffer;

void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN2, OUTPUT);

    stopAll();

    Serial.begin(115200); // USB debug
    BT.begin(9600);       // HC-05 link

    Serial.println("ROS2-ready diff-drive: expects lines like 'V <left> <right>' over Bluetooth or USB.");
}

void loop()
{
    // Commands from PC via Bluetooth (/dev/rfcomm0)
    processStream(BT, btBuffer);

    // Optional: same protocol over USB Serial for debugging
    processStream(Serial, usbBuffer);
}
