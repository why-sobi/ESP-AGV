# E-Puck AGV Dual Autoencoder Anomaly Detection: Project Status & Blueprint

This document tracks our current implementation progress, followed by the complete end-to-end blueprint, validation checks, and success criteria for building the dual autoencoder system.

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

## Phase 1: Data Collection & Control Loop (Next Steps)

### What to Do
1. **Upgrade to a PID Controller:** Replace the hardcoded threshold `if-else` script with a Proportional-Derivative (PD) line-following controller for smooth steering corrections based on error inputs ($\text{gs\_left} - \text{gs\_right}$).
2. **Record Baseline Data:** Run the simulation for 10–15 minutes under completely healthy, normal operating conditions using the PID controller.

### Checks to Handle
* Verify that telemetry logging drops no frames and time intervals remain uniform ($\Delta t = \text{Webots timestep}$).
* Check that ground IR values do not flatline or rail out prematurely during normal tracking.

### When to Know It's "Good Enough"
* **Visual Check:** Time-series plots show smooth, continuous gradient curves for motor adjustments rather than digital square-wave snaps.
* **Mathematical Check:** The absolute step-to-step change (first derivative) in motor commands has a standard deviation close to zero, proving minimal mechanical jerk.

---

## Phase 2: Feature Engineering & Preprocessing

### What to Do
1. **Derive Instantaneous Velocity:** Convert cumulative wheel positions (`actual_pos`) into actual velocities by calculating differences over time ($\Delta \text{pos} / \Delta t$).
2. **Construct Rolling Windows:** Group sequential telemetry steps into sliding temporal windows (e.g., lookback window of $W = 10$ or $20$ steps) so the models capture temporal behavior patterns rather than isolated frames.
3. **Feature Scaling:** Normalize all input features using `StandardScaler` (mean = 0, variance = 1) based strictly on the healthy training dataset.

### Checks to Handle
* Ensure no `NaN` or `Inf` values exist after computing velocity derivatives or sliding windows.
* Confirm feature shapes match the expected tensor dimensions for PyTorch (`[Batch_Size, Window_Size, Features]`).

### When to Know It's "Good Enough"
* Feature distributions are centered, scaled, and free of extreme outliers or sudden clipping artifacts.

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

### When to Know It's "Good Enough"
* Validation reconstruction loss stabilizes at a very low, asymptotic floor.
* The model can accurately reconstruct unseen healthy test runs with minimal error.

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

### When to Know It's "Good Enough"
* **False Positive Rate:** $< 1\%$ false alarms during normal, healthy navigation runs.
* **Detection Rate:** $100\%$ detection rate for injected motor stalls or sensor blocks, with reconstruction error spiking well above the established threshold within 1–2 seconds of fault onset.