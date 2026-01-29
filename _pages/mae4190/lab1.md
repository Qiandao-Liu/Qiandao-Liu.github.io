---
layout: archive
title: "Lab 1: Artemis and Bluetooth"
permalink: /mae4190/lab1/
author_profile: true
---

{% include base_path %}

## Prelab

### Setup

**Arduino IDE Configuration:**
- Installed Arduino IDE with SparkFun Apollo3 board manager
- Successfully connected and programmed the Artemis Nano board
- Tested basic functionality with Example sketches (Blink, Serial, AnalogRead, MicrophoneOutput)

**Python Environment Setup:**
- Created virtual environment `FastRobots_ble`
- Installed required packages: `numpy`, `pyyaml`, `colorama`, `nest_asyncio`, `bleak`, `jupyterlab`
- Started Jupyter server for running notebooks

**MAC Address Configuration:**
The Artemis board MAC address was obtained from the Serial Monitor and configured in `connections.yaml`:
```
Artemis MAC: C0:81:31:25:23:64
```

### Codebase Understanding

**Bluetooth Low Energy (BLE) Architecture:**

The BLE communication system uses a client-server model where:
- **Artemis board** acts as the peripheral (server), advertising BLE services
- **Computer (Jupyter)** acts as the central (client), connecting to the peripheral

**Key Components:**

1. **UUIDs (Universally Unique Identifiers):**
   - Service UUID: Identifies the BLE service
   - Characteristic UUIDs: Identify different data types (RX_STRING, TX_STRING, TX_FLOAT)
   - Generated unique UUID to avoid conflicts with other students' boards

2. **GATT Characteristics:**
   - `BLECStringCharacteristic` (RX_STRING): Receives commands from computer
   - `BLECStringCharacteristic` (TX_STRING): Sends string data to computer
   - `BLEFloatCharacteristic` (TX_FLOAT): Sends float data to computer

3. **Command Protocol:**
   - Commands formatted as: `<cmd_type>:<value1>|<value2>|...`
   - Command type is an integer mapped in `enum CommandTypes`
   - Values separated by delimiter `|`

4. **Arduino Classes:**
   - `RobotCommand`: Parses incoming command strings
   - `EString`: Enhanced string manipulation for building responses

5. **Python Controller:**
   - `ArtemisBLEController`: Manages BLE connection and data transfer
   - Notification handlers: Callback functions for asynchronous data reception

---

## Lab Tasks

### Configurations

**Generated New Service UUID:**
```python
from uuid import uuid4
uuid4()  # Generated: d1e59283-ea64-46d2-9619-feda9179e362
```

**Updated Files:**
- `ble_arduino.ino`: Updated `BLE_UUID_TEST_SERVICE`
- `connections.yaml`: Updated `ble_service` and `artemis_address`
- `cmd_types.py`: Added new command types

**Command Types Added:**
```cpp
enum CommandTypes {
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    GET_TIME_MILLIS,      // Task 3
    SEND_TIME_DATA,       // Task 6
    GET_TEMP_READINGS,    // Task 7
    START_RECORDING,      // Data collection control
    STOP_RECORDING
};
```

---

### Task 1: ECHO Command

**Objective:** Send a string from computer to Artemis, receive an augmented response.

**Implementation:**
```cpp
case ECHO: {
    char char_arr[MAX_MSG_SIZE];
    robot_cmd.get_next_value(char_arr);

    tx_estring_value.clear();
    tx_estring_value.append("Robot says -> ");
    tx_estring_value.append(char_arr);
    tx_estring_value.append(" :)");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    break;
}
```

**Python Test:**
```python
ble.send_command(CMD.ECHO, "HiHello")
s = ble.receive_string(ble.uuid['RX_STRING'])
print(f"Received: {s}")
```

**Result:**
```
Received: Robot says -> HiHello :)
```

**Screenshot:** [Add screenshot of Jupyter output]

---

### Task 2: SEND_THREE_FLOATS

**Objective:** Extract three float values from a command string on the Artemis.

**Implementation:**
```cpp
case SEND_THREE_FLOATS: {
    float float_a, float_b, float_c;
    robot_cmd.get_next_value(float_a);
    robot_cmd.get_next_value(float_b);
    robot_cmd.get_next_value(float_c);

    Serial.print("Three Floats: ");
    Serial.print(float_a);
    Serial.print(", ");
    Serial.print(float_b);
    Serial.print(", ");
    Serial.println(float_c);
    break;
}
```

**Python Test:**
```python
ble.send_command(CMD.SEND_THREE_FLOATS, "1.5|2.7|3.14")
```

**Result (Serial Monitor):**
```
Three Floats: 1.50, 2.70, 3.14
```

**Screenshot:** [Add screenshot of Serial Monitor]

---

### Task 3: GET_TIME_MILLIS Command

**Objective:** Create a command that returns the current time in milliseconds.

**Implementation:**
```cpp
case GET_TIME_MILLIS:
    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append((int)millis());
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    break;
```

**Python Test:**
```python
ble.send_command(CMD.GET_TIME_MILLIS, "")
time.sleep(0.5)
s = ble.receive_string(ble.uuid['RX_STRING'])
print(f"Received: {s}")
```

**Result:**
```
Received: T:105609
```

---

### Task 4: Notification Handler Setup

**Objective:** Set up a Python notification handler to receive data asynchronously.

**Implementation:**
```python
timestamps = []
arrival_times = []

def notification_handler(uuid, byte_array):
    global timestamps, arrival_times
    s = ble.bytearray_to_string(byte_array)
    arrival_time = time.time()

    if s.startswith("T:"):
        timestamp = int(s.split(":")[1].split("|")[0])
        timestamps.append(timestamp)
        arrival_times.append(arrival_time)
        print(f"Received timestamp: {timestamp} ms")

ble.start_notify(ble.uuid['RX_STRING'], notification_handler)
```

**Key Concept:** The notification handler is a callback function that executes automatically whenever the Artemis writes to the TX_STRING characteristic, enabling asynchronous data reception without polling.

---

### Task 5: Real-time Data Transfer Rate

**Objective:** Measure the effective data transfer rate by continuously requesting timestamps.

**Method:**
1. Send GET_TIME_MILLIS commands in a loop for 5 seconds
2. Notification handler records arrival times
3. Calculate message rate and average interval

**Python Code:**
```python
timestamps.clear()
arrival_times.clear()

start_time = time.time()
count = 0

while time.time() - start_time < 5:
    ble.send_command(CMD.GET_TIME_MILLIS, "")
    count += 1
    time.sleep(0.1)  # 100ms between requests

duration = arrival_times[-1] - arrival_times[0]
rate = len(timestamps) / duration
```

**Results:**
```
Sent 33 requests
Received 32 timestamps
Effective data transfer rate: 6.79 messages/second
Average interval: 147.20 ms between messages
```

**Analysis:**
- Despite requesting every 100ms, actual rate is ~147ms per message
- This includes BLE communication overhead, notification processing, and command execution time
- Effective data rate: **~7 messages/second** for real-time transmission

---

### Task 6: Array Storage and Batch Transfer

**Objective:** Store timestamps in an array on the Artemis and send all data at once.

**Implementation:**

**Global Arrays:**
```cpp
#define MAX_DATA_SIZE 1000
unsigned long timeStamps[MAX_DATA_SIZE];
int dataIndex = 0;
bool arrayFull = false;
bool collectingData = false;
int sampleInterval = 10;  // Sample every 10ms
```

**Data Recording in loop():**
```cpp
void record_data() {
    if (!arrayFull) {
        timeStamps[dataIndex] = millis();
        tempReadings[dataIndex] = getTempDegC();
        dataIndex++;

        if (dataIndex >= MAX_DATA_SIZE) {
            arrayFull = true;
        }
    }
}

// In loop():
if (collectingData && (millis() - lastSampleTime >= sampleInterval)) {
    record_data();
    lastSampleTime = millis();
}
```

**Batch Send Command:**
```cpp
case SEND_TIME_DATA: {
    int limit = arrayFull ? MAX_DATA_SIZE : dataIndex;
    for (int i = 0; i < limit; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append((int)timeStamps[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        delay(10); // Small delay for reliable transmission
    }

    dataIndex = 0;
    arrayFull = false;
    break;
}
```

**Python Test:**
```python
# Start recording
ble.send_command(CMD.START_RECORDING, "")
time.sleep(3)  # Record for 3 seconds
ble.send_command(CMD.STOP_RECORDING, "")

# Retrieve all data
ble.send_command(CMD.SEND_TIME_DATA, "")
time.sleep(5)  # Wait for all data
```

**Results:**
```
Recording started
Recording for 3 seconds...
Recording stopped. Samples: 344
Received 344 timestamps

Sampling rate: 100.0 samples/second
Average interval: 10.0 ms
```

**Analysis:**
- Successfully collected **344 samples** in 3 seconds
- **Sampling rate: ~115 samples/second** (10ms interval)
- Batch method allows much faster data collection than real-time transmission

---

### Task 7: Temperature Readings with Timestamps

**Objective:** Record temperature readings with corresponding timestamps, send both arrays together.

**Implementation:**
```cpp
case GET_TEMP_READINGS: {
    int limit = arrayFull ? MAX_DATA_SIZE : dataIndex;
    for (int i = 0; i < limit; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append((int)timeStamps[i]);
        tx_estring_value.append("|C:");
        tx_estring_value.append(tempReadings[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        delay(10);
    }

    dataIndex = 0;
    arrayFull = false;
    break;
}
```

**Python Handler:**
```python
def temp_notification_handler(uuid, byte_array):
    s = ble.bytearray_to_string(byte_array)

    # Parse "T:123456|C:25.5"
    if "T:" in s and "|C:" in s:
        parts = s.split("|")
        timestamp = int(parts[0].split(":")[1])
        temp = float(parts[1].split(":")[1])
        timestamps.append(timestamp)
        temp_readings.append(temp)

ble.start_notify(ble.uuid['RX_STRING'], temp_notification_handler)
ble.send_command(CMD.GET_TEMP_READINGS, "")
```

**Results:**
```
Received 344 temperature readings
Min temp: 33.00 °C
Max temp: 33.50 °C
Avg temp: 33.25 °C
```

**Observations:**
- Temperature readings are stable around 33°C (board temperature)
- Each reading correctly paired with its timestamp
- Format: `T:12345|C:33.25` allows easy parsing in Python

**Screenshot:** [Add screenshot showing temperature data plot]

---

## Discussion

### Comparison of Data Transfer Methods

**Method 1: Real-time Request-Response**
- Send command → Artemis responds → Receive data → Repeat
- Measured rate: **~7 messages/second** (147ms per message)

**Method 2: Batch Collection and Transfer**
- Artemis collects data locally → Send all at once when requested
- Collection rate: **~100 samples/second** (10ms interval)
- 344 samples collected in 3 seconds

### Advantages and Disadvantages

| Method | Advantages | Disadvantages | Best Use Case |
|--------|-----------|---------------|---------------|
| **Real-time** | • Immediate feedback<br>• Low memory usage<br>• Can react to data instantly | • Slow (7 msg/s)<br>• High BLE overhead<br>• Cannot sample fast sensors | • Slow-changing data<br>• Need instant feedback<br>• Interactive control |
| **Batch** | • Fast sampling (100+ samples/s)<br>• Low BLE overhead<br>• Efficient use of bandwidth | • Delayed feedback<br>• High memory usage<br>• Risk of data loss if array fills | • Fast sensor data (IMU, ToF)<br>• Post-processing analysis<br>• High-frequency sampling |

### Data Collection Speed - Batch Method

The batch method can record data as fast as the `loop()` cycle allows. With a 10ms sampling interval:
- **Sampling rate:** 100 samples/second
- **Can be faster:** Reducing interval to 5ms → 200 samples/second

The limiting factor is the `loop()` execution time, not BLE transmission.

### Memory Capacity Analysis

**Artemis RAM:** 384 kB = 384,000 bytes

**Current Implementation:**
```cpp
unsigned long timeStamps[1000];  // 4 bytes × 1000 = 4,000 bytes
float tempReadings[1000];         // 4 bytes × 1000 = 4,000 bytes
Total: 8,000 bytes
```

**Maximum Theoretical Capacity:**
Assuming 8 bytes per sample (timestamp + reading):
```
384,000 bytes / 8 bytes per sample = 48,000 samples
```

**Practical Considerations:**
- Leave ~100 kB for program and stack: **280 kB usable**
- Maximum samples: ~35,000
- At 100 samples/second: **350 seconds (5.8 minutes)** of continuous data

**Multi-sensor Arrays:**
If storing 3-axis IMU data (3 floats) + timestamp:
```
(4 bytes × 3) + 4 bytes = 16 bytes per sample
280,000 bytes / 16 bytes = 17,500 samples
```

### Challenges and Solutions

**Challenge 1:** Some messages were lost during real-time transmission.
- **Solution:** Added notification handler to catch all responses asynchronously, reducing reliance on polling.

**Challenge 2:** Array overflow when recording too long.
- **Solution:** Implemented `arrayFull` flag and START/STOP recording commands for controlled data collection.

**Challenge 3:** Parsing combined timestamp and temperature strings.
- **Solution:** Used delimiter-based format `T:123|C:25.5` and Python `split()` for reliable parsing.

### Key Learnings

1. **BLE has significant overhead:** Real-time communication is limited to ~7 messages/second due to connection intervals, notification delays, and command processing.

2. **Batch transfer is essential for fast sensors:** IMU data at 100+ Hz requires local storage and batch transmission.

3. **Memory management matters:** With limited RAM, must balance sample count vs. data resolution vs. number of sensors.

4. **Asynchronous callbacks are powerful:** Notification handlers allow non-blocking data reception, critical for real-time systems.

5. **Data format design:** String-based protocol (`T:123|C:25.5`) trades efficiency for simplicity and debuggability. Binary formats would be more efficient but harder to debug.

---

## Code Repository

Full code available on GitHub: [Add GitHub Gist link]

**Key Files:**
- [ble_arduino.ino](link) - Arduino sketch with all command handlers
- [demo.ipynb](link) - Jupyter notebook with all tasks
- [cmd_types.py](link) - Command type definitions

---

[Back to MAE 4190](/mae4190/)
