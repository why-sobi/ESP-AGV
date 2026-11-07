# E-Puck AGV Dual Autoencoder Anomaly Detection: Project Status & Blueprint

This document tracks our current implementation progress, the complete end-to-end blueprint, validation checks, and the critical engineering "gotchas" to avoid common pitfalls in the trenches.

---

## Current Progress & What’s Done

1. **Environment & Workspace Setup:**
* Established workspace layout containing `data`, `esp32-s3`, and `simulation` directories.
* Successfully configured Python virtual environment (`.venv`) and resolved Webots C-extension/MSVC CRT binary mismatches by aligning the environment and configuring per-controller `runtime.ini` files to point directly to `.venv\Scripts\python.exe`.


2. **Telemetry Logging Pipeline:**
* Implemented a Webots controller script that logs time, target motor velocities, wheel odometry (`actual_pos`), and ground IR sensors (`gs0`, `gs1`, `gs2`).
* Configured robust batch-writing to `data/SensorData-v1.csv` to ensure data safety during simulation runs.


3. **Data Inspection & Initial Analysis:**
* Verified data ingestion using exploratory scripts/notebooks (`15,000+` telemetry rows captured).
* Identified that the initial placeholder logic produces jerky, high-frequency square-wave oscillations, highlighting the need for a smooth PID controller before training anomaly detectors.



---

## Phase 1: Data Collection & Control Loop

### What to Do

1. **Upgrade to a PID Controller:** Replace the hardcoded threshold `if-else` script with a Proportional-Derivative (PD) line-following controller for smooth steering corrections based on error inputs ($\text{gs\_left} - \text{gs\_right}$).
2. **Record Baseline Data:** Run the simulation for 10–15 minutes under completely healthy, normal operating conditions using the PID controller.

### Checks to Handle

* Verify that telemetry logging drops no frames and time intervals remain uniform ($\Delta t = \text{Webots timestep}$).
* Check that ground IR values do not flatline or rail out prematurely during normal tracking.

### ESP-IDF Gotchas

* **Simulation Step vs. Controller Loop Desync:** Ensure your logging frequency strictly matches your `robot.step(timestep)` frequency. If you log asynchronously or miss steps, your $\Delta t$ for velocity derivatives will become corrupted with variable time-deltas.
* **Sensor Saturation:** Make sure your ground IR sensors don't hit their absolute min/max limits on straightaways. If they rail out early, your proportional error loses sensitivity.

---

## Phase 2: Feature Engineering & Preprocessing

### What to Do

1. **Derive Instantaneous Velocity:** Convert cumulative wheel positions (`actual_pos`) into actual velocities by calculating differences over time ($\Delta \text{pos} / \Delta t$).
2. **Construct Rolling Windows:** Group sequential telemetry steps into sliding temporal windows (e.g., lookback window of $W = 10$ or $20$ steps) so the models capture temporal behavior patterns rather than isolated frames.
3. **Feature Scaling:** Normalize all input features using `StandardScaler` (mean = 0, variance = 1) based *strictly* on the healthy training dataset.

### Checks to Handle

* Ensure no `NaN` or `Inf` values exist after computing velocity derivatives or sliding windows.
* Confirm feature shapes match the expected tensor dimensions for PyTorch (`[Batch_Size, Window_Size, Features]`).

### ESP-IDF Gotchas

* **Data Leakage in Scaling:** Never compute your `StandardScaler` mean and variance across the entire dataset (train + validation + test). Fit the scaler *only* on the training split, then transform the validation/test splits using those exact parameters.
* **Finite Difference Noise:** Raw odometry positions from physics engines can have minor quantization noise. When you take the derivative ($\Delta \text{pos} / \Delta t$), that noise gets amplified. Consider a mild rolling average or Savitzky-Golay filter if your derived velocities look overly jagged.

---

## Phase 3: Model Architecture & Training

### What to Do

1. **Implement Dual Autoencoders:** Build two separate PyTorch models:
* **Motor Autoencoder:** Inputs target velocities vs. actual derived velocities/odometry.
* **IR Autoencoder:** Inputs ground IR sensor streams (`gs0`, `gs1`, `gs2`).


2. **Train on Healthy Data Only:** Train both models using Mean Squared Error (MSE) loss exclusively on data collected from healthy PID runs.

### Checks to Handle

* Monitor training and validation loss curves to ensure they converge smoothly without severe overfitting.
* Use a validation split (e.g., 80% train, 20% validation) derived from a *separate* healthy simulation run to prevent data leakage.

### ESP-IDF Gotchas

* **Architecture Choice for Temporal Data:** A standard dense/MLP autoencoder treats every time step independently, missing temporal context. Since you are using sliding windows ($W$), use either a **1D Convolutional Autoencoder** or a **LSTM/GRU Autoencoder** so the model actually learns the *sequence* of motion, not just static frames.
* **Over-reconstruction:** If your autoencoder latent space is too large, it will learn to perfectly reconstruct *anything*—including anomalies. Keep the bottleneck compressed enough that it forces the model to generalize normal behavior.

---

## Phase 4: Anomaly Detection & Fault Isolation

### What to Do

1. **Establish Error Thresholds:** Calculate reconstruction error (MSE) across the validation dataset and set an anomaly threshold using a high statistical percentile (e.g., the 99th percentile of validation error).
2. **Inject Controlled Hardware Faults:** Test the system in Webots by introducing artificial anomalies:
* *Motor Fault:* Mechanically drag a wheel, simulate a motor stall, or introduce friction.
* *Sensor Fault:* Block an IR sensor with dust/debris or flatline a sensor signal.



### Checks to Handle

* **Fault Isolation (Decoupling):** Confirm that a motor failure spikes the error *only* in the Motor Autoencoder while leaving the IR Autoencoder calm, and vice versa.
* **False Positive Rate (FPR):** Test the model on a normal, healthy run it hasn't seen before to ensure it doesn't trigger false alarms.

### ESP-IDF Gotchas

* **Cascading Physical Failures:** In reality, physics couples systems together. If a motor stalls, the robot drifts off the line, causing the IR sensors to see different values. This can cause a *motor* fault to bleed into the *IR* autoencoder error stream. Look at the *leading edge* of the error spike (which subsystem failed first?) rather than just raw simultaneous triggers when diagnosing root causes.
* **Threshold Tuning Trap:** Setting the threshold via static percentiles works for simulation, but real-world hardware has ambient noise drift. Keep your threshold evaluation window rolling if you eventually move this logic to the ESP32-S3.