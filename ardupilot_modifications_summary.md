# ArduPilot Modifications Summary for Custom Boat Integration

Based on the `custom_boat_integration.md` specification, several modifications to the ArduPilot (Rover) firmware are required. This document outlines the technical approach, the specific files to be modified, and strategies for MAVLink telemetry integration.

## 1. High-Level Architectural Summary
The ArduPilot Rover firmware must be modified to support a suite of custom sensors, actuators, and telemetry loops. Specifically, the system requires:
1. **Custom I2C Polling Loop:** Implementing a 1 Hz polling loop for an ADS1115 analog-to-digital converter, an ABLIC S-35770 RPM sensor, and two TCA9534 I2C expanders.
2. **Custom ArduPilot Parameters:** Adding new configuration parameters to ArduPilot to scale and offset the analog readings from the ADS1115 (Trim Angle, Battery Voltage, Rudder Angle, Fuel Level).
3. **RC Input to I2C Actuation Mapping:** Intercepting RC Channel 8 to control the TCA9534 expander for Trim Up/Down actuation.
4. **Custom Telemetry Routing:** Sending custom metrics and lighting statuses to QGroundControl via MAVLink.

## 2. Specific Codebase Modifications

### 2.1 Custom Parameters Definition
We need to introduce custom parameters (`CUST_TRIM_MULT`, `CUST_TRIM_OFF`, `CUST_BAT_MULT`, etc.) for the ADS1115 scaling.
* **Files to Modify:**
  * `Rover/Parameters.h`: Declare the new parameters within the `Parameters` class structure.
  * `Rover/Parameters.cpp`: Define the default values and descriptions for the `CUST_*` parameters using the `@Param` documentation tags so they appear in QGroundControl.

### 2.2 Custom I2C Devices & 1 Hz Main Loop
A dedicated driver or custom module needs to be created to handle the I2C communications on the specified addresses.
* **Files to Modify/Create:**
  * **Create `Rover/CustomBoat.cpp` and `Rover/CustomBoat.h`:** To encapsulate the ADS1115, ABLIC S-35770, and TCA9534 logic.
  * `Rover/Rover.h` & `Rover/Rover.cpp`: Instantiate the custom module and hook it into the scheduler to run at 1 Hz (or run it as a background thread to prevent blocking the main loop with I2C bus traffic).
  * `Rover/radio.cpp` (or similar RC input handling logic): Hook into RC Channel 8 to trigger the TCA9534 I2C Trim commands.

### 2.3 RPM Sensor Driver Integration
The ABLIC S-35770 requires specialized handling (1s count, read, and reset). ArduPilot has a modular RPM sensor backend system.
* **Files to Modify/Create:**
  * **Create `libraries/AP_RPM/AP_RPM_ABLIC.cpp` & `.h`:** Implement a new RPM backend derived from `AP_RPM_Backend`.
  * `libraries/AP_RPM/AP_RPM.cpp` & `libraries/AP_RPM/AP_RPM_Params.cpp`: Register the new backend type so it can be selected via standard `RPM_TYPE` parameters.

### 2.4 Lighting Control & Status Verification
Lighting is routed to `AUX1-AUX5` via standard RC passthrough, but their operational status is verified via the TCA9534 expander (`000b`).
* **Implementation:** `Rover/CustomBoat.cpp` will read the TCA9534 `000b` at 1 Hz and push the status onto MAVLink.

## 3. MAVLink Telemetry Strategy

The requirement dictates sending custom data (Trim Angle, Rudder Angle, Fuel Level, Lighting Statuses, Engine RPM) to a QGroundControl fork.

### MAVLink Approach Recommendation
We have two main options for transmitting this data:

**Option A: Standard MAVLink Extensions (Recommended for ease of deployment)**
Instead of defining a custom MAVLink message, we can leverage existing, highly flexible MAVLink messages.
* **`NAMED_VALUE_FLOAT` / `NAMED_VALUE_INT`:** We can transmit custom states (e.g., "TRIM_ANG", "RUDD_ANG", "LGT_STAT"). QGroundControl natively parses these and makes them available to QML via the `Vehicle` object's parameter/telemetry dictionaries.
* **Native Battery & Fuel Messaging:** For Battery Voltage and Fuel Level, we should ideally hook into ArduPilot's native `BATTERY_STATUS` and `SYS_STATUS` MAVLink streams. We can write our ADS1115 readings into ArduPilot's core battery/fuel objects, allowing standard QGC widgets to work out-of-the-box, triggering native low-fuel warnings automatically.

**Option B: Custom MAVLink Message Definitions**
If the data structure is highly rigid and requires minimal bandwidth overhead, we can define a custom message (e.g., `BOAT_CUSTOM_STATUS`).
* **Files to Modify:** `modules/mavlink/message_definitions/v1.0/ardupilotmega.xml` (or `common.xml`).
* **Pros:** Strict typing, single packet per update.
* **Cons:** Requires recompiling the MAVLink headers for **both** ArduPilot and the QGroundControl fork, significantly complicating the build process and long-term maintainability.

**Conclusion:** Using `NAMED_VALUE_FLOAT` for custom indicators (Trim, Rudder, Lights) and mapping the ADS1115 fuel/battery readings into ArduPilot's native Battery/RPM libraries is the cleanest approach. It minimizes firmware modifications while fully supporting the custom QGC QML overlays.
