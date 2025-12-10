# Hexapod Robot Locomotion System

## Overview (Abstract)

This project implements a robust locomotion system for a 6-legged hexapod robot using analytical inverse kinematics, grid-based workspace sampling, and time-scaled trajectory generation. The primary motivation was to enable stable walking while respecting the physical constraints of the robot's limited workspace and preventing servo overload that could crash the microcontroller. The system successfully achieves smooth, repeatable tripod gait locomotion through careful trajectory planning that considers workspace geometry at different leg heights. Key outcomes include: (1) a pre-computed workspace lookup table that reveals workspace shrinkage at elevated positions, (2) smooth motion profiles using cubic/trapozedal time scaling to prevent acceleration spikes, and (3) a position-aware pre-positioning system that eliminates sudden movements between gait cycles. The implementation demonstrates how proper motion planning can overcome hardware limitations to achieve reliable autonomous walking.

---

## Technical Approach

The system integrates three core robotics concepts to achieve stable locomotion:

### 1. Inverse Kinematics (Kinematics & Control)

The hexapod uses **analytical closed-form inverse kinematics** for a 3-link serial manipulator representing each leg. Given a desired foot position (x, y, z) in Cartesian space, the IK solver computes the three joint angles (hip θ₁, knee θ₂, ankle θ₃) required to reach that position.

**Mathematical Formulation:**
- **Hip angle (top view):** θ₁ = atan2(y, x)
- **Projected length:** r₁ = √(x² + y²) - a₁
- **Vertical component:** r₂ = z
- **Leg extension:** r₃ = √(r₁² + r₂²)
- **Knee angle (side view):** θ₂ = φ₁ + φ₂, where:
  - φ₁ = acos((a₃² - a₂² - r₃²) / (-2a₂r₃))
  - φ₂ = atan2(r₂, r₁)
- **Ankle angle:** θ₃ = -(π - φ₃), where φ₃ = acos((r₃² - a₂² - a₃²) / (-2a₂a₃))

Link lengths: a₁ = 37mm, a₂ = 63.54mm, a₃ = 200mm

The solver includes workspace validation by checking if the target distance exceeds the sum of link lengths, and uses constrained arc-cosine operations to prevent mathematical errors near singularities. Each of the 6 legs operates with a different base orientation (0°, 60°, 120°, 180°, 240°, 300° around the robot's Z-axis), requiring careful coordinate frame management.

### 2. Grid-Based Workspace Sampling (Motion Planning)

A critical discovery during development was that the robot's reachable workspace is **highly dependent on Z-height**. To address this, we implemented a pre-computed lookup table with 301 entries spanning Z = -300mm to 0mm (1mm resolution).

**Workspace Analysis Method:**
```python
# For each Z height, sample a grid of (X, Y) positions
for z in range(-300, 1):
    reachable_points = []
    for x in grid_x:
        for y in grid_y:
            if inverse_kinematics_valid(x, y, z):
                reachable_points.append((x, y))
    
    # Fit minimum enclosing circle
    center, radius = fit_circle(reachable_points)
    workspace_table[z] = (center_x, center_y, radius)
```

**Key Finding:** Workspace radius shrinks significantly when legs are lifted:
- Ground level (Z = -165mm): radius ≈ 83mm
- Lifted 20mm (Z = -145mm): radius ≈ 67mm
- **19% reduction** in reachable area

This insight drove the stride length calculation:
```cpp
float safe_radius = min(radius_ground, radius_lifted);
double stride_length = safe_radius * 2.0 * 1;
```

### 3. Trajectory Generation with Time Scaling (Motion Planning & Control)

The locomotion uses a **tripod gait pattern** where legs alternate in two groups of three:
- **Group A:** Legs 1, 3, 5 (0°, 120°, 240°)
- **Group B:** Legs 2, 4, 6 (60°, 180°, 300°)

**Gait Cycle Structure:**

Each group executes two phases:

1. **Swing Phase (leg in air):**
   - X, Y: Linear interpolation from backward extreme → forward extreme
   - Z: Parabolic arc for smooth lift/land
   ```cpp
   target_z = ground_z + 4.0 * step_height * t * (1 - t)
   ```
   Maximum height occurs at t = 0.5 (midpoint of swing)

2. **Stance Phase (leg on ground):**
   - X, Y: Linear interpolation from forward extreme → backward extreme
   - Z: Constant at ground level
   - This phase *pushes* the robot body forward

**Time Scaling for Smooth Motion:**

Raw linear interpolation (t ∈ [0, 1]) causes abrupt acceleration/deceleration. Three scaling functions were implemented to adjust the time per step dynamically:

1. **Quadratic (Ease-in-out):**
   ```cpp
   ms_multiplier(τ) = max(1.0, 50 * (τ - 0.5)²)
   ```
   - Slows down near endpoints (τ=0 and τ=1) for smooth starts/stops
   - Speeds up in the middle (τ=0.5) for efficient motion
   - Minimum 1.0× at center, maximum 13.5× at endpoints

2. **Trapezoidal (Constant velocity cruise):**
   ```cpp
   ms_multiplier(τ) = -7(5τ + 1)      if τ < 0.1  (ease in)
                    = 1.0             if 0.1 ≤ τ < 0.9  (constant speed)
                    = 7(5τ - 4)       if τ ≥ 0.9  (ease out)
   ```
   - Ramps up speed during first 10% of motion
   - Maintains constant 1.0× speed for middle 80%
   - Ramps down during final 10%

3. **Linear (Baseline - No scaling):**
   ```cpp
   ms_multiplier(τ) = 1.0  (constant)
   ```
   - No time adjustment; uniform step timing throughout motion
   - Used as baseline for comparison

The multiplier adjusts `ms_per_move` for each step, creating variable velocity profiles that reduce mechanical stress on servos.

**Position Management:**

To prevent drift across multiple walking cycles, the system implements:
- **Pre-positioning phase:** Smoothly interpolates from current leg positions to starting extremes over 1.5 seconds
- **Absolute positioning:** All targets calculated relative to workspace center, not cumulative movements
- **End-state preservation:** Legs remain at extremes after each cycle, ready for next iteration

---

## Implementation Details

### Hardware Platform

**Microcontroller:** Teensy 4.0
- ARM Cortex-M7 @ 600MHz
- 512KB RAM, 2MB Flash
- Critical for real-time servo control with ~10ms update intervals per position command

**Actuators:** 18× LX-224HV Serial Servos
- Max speed: 333.33°/sec (60° in 0.18s)
- Torque: 20 kg·cm @ 11.1V
- Communication: UART @ 115200 baud (2 buses, 9 servos each)
- 10-byte protocol: [Header, ID, Length, Command, Params..., Checksum]

**Leg Configuration:**
- 6 legs × 3 joints = 18 DOF total
- Base rotations: [0°, 60°, 120°, 180°, 240°, 300°]
- Joint limits: Hip ±90°, Knee 0°-180°, Ankle -90°-90°

### Software Architecture

**Module Structure:**
```
teensy_hexapod/
├── include/
│   ├── Kinematics.h        # IK/FK solvers
│   ├── Leg.h               # Single leg controller
│   ├── Hexapod.h           # Multi-leg coordinator
│   ├── WorkspaceLookup.h   # Pre-computed workspace data (PROGMEM)
│   └── LobotSerialServoControl.h  # UART servo driver
├── src/
│   ├── Kinematics.cpp      # 148 lines - analytical IK implementation
│   ├── Leg.cpp             # Servo control + range validation
│   ├── Hexapod.cpp         # 390 lines - gait generation
│   └── main.cpp            # Test harness
└── workspace_checker.ipynb # Python notebook for workspace analysis
```

**Key Design Decisions:**

1. **PROGMEM Storage for Workspace Table:**
   - 301 entries × 16 bytes = ~4.8KB
   - Stored in Flash to preserve RAM for real-time operations
   - Access via `pgm_read_float()` with ~5 cycle latency

2. **Dual UART Architecture:**
   - Legs 1-3 on Serial1, Legs 4-6 on Serial2
   - Prevents bus contention during simultaneous commands
   - Commands sent in parallel, then wait for completion

3. **Error Handling Strategy:**
   - IK validation before servo commands
   - Joint limit checks in forward kinematics
   - Early return on failure to prevent cascading errors
   - Servo "unload" command on abort to prevent overheating

4. **Timing System:**
   - Total cycle time: 1000ms (TOTAL_MS constant)
   - Step size: 1.2mm (STEP_SIZE) → ~90 steps per full stride
   - Per-step timing scaled by S(τ) for smooth motion
   - `delay()` synchronization between steps

### Development Environment

**Build System:** PlatformIO
- Platform: Teensy
- Framework: Arduino
- Monitor speed: 115200 baud

**Debugging Tools:**
- Serial monitor for position logging
- Per-leg error reporting with leg ID
- Phase transition markers
- Actual vs. commanded position verification

**Reproducibility Steps:**
1. Install PlatformIO
2. Clone repository
3. Connect Teensy 4.0 via USB
4. Connect servos: Serial1 → legs 1-3, Serial2 → legs 4-6
5. Power servos with 7.4V LiPo (separate from Teensy)
6. `pio run --target upload`
7. `pio device monitor` to observe execution

---

## Experimental Design

### Hypothesis

Time scaling in trajectory generation significantly affects motion smoothness and mechanical stress on the hexapod robot. We hypothesize that:
1. **Cubic (quadratic) time scaling** will produce the smoothest motion with lowest peak acceleration
2. **Trapezoidal scaling** will balance smoothness with faster completion time
3. **No scaling (linear)** will cause jerky motion and higher servo load

### Variables

**Independent Variable:** Time scaling function
- **Level 1:** No scaling (constant 1.0× speed throughout)
- **Level 2:** Quadratic scaling (max(1.0, 50(τ - 0.5)²) - slows at endpoints)
- **Level 3:** Trapezoidal scaling (10% ease-in/out, 80% constant speed)

**Dependent Variables:**
1. **Motion smoothness:** Qualitative assessment of leg movement fluidity
2. **Servo strain:** Observable vibration/judder during motion
3. **Gait stability:** Success rate of completing 5 consecutive walking cycles
4. **MCU stability:** Occurrence of crashes due to current spikes

**Controlled Variables:**
- Ground Z: -165mm (constant across all trials)
- Step height: 20mm
- Stride length: Auto-calculated (~108mm)
- Walking direction: +Y axis (forward)
- Number of steps: 5 cycles per trial

### Metrics

**Quantitative:**
- Cycle completion rate: % of successful gait cycles before failure
- Time per cycle: Total duration (ms) including pre-positioning
- Position error: Euclidean distance between commanded and actual leg positions

**Qualitative:**
- Motion smoothness: 5-point scale (1=jerky, 5=fluid)
- Mechanical noise: Audible servo strain assessment
- Visual appearance: Human observer rating of natural motion

### Justification

These experiments directly test the core hypothesis that trajectory smoothness impacts system reliability. The three scaling functions represent a spectrum from no smoothing (baseline) to heavy smoothing (quadratic), allowing us to identify the optimal balance between motion quality and computational overhead. The metrics were chosen to capture both objective performance (completion rate, timing) and subjective quality (smoothness, naturalness) since the ultimate goal is stable, visually appealing locomotion.

---

## Results and Discussion

### Qualitative Results Summary

| Time Scaling | Motion Smoothness | Time per Cycle | MCU Stability | Overall Assessment |
|--------------|------------------|----------------|---------------|--------------------|
| None (Linear) | Jerky, abrupt transitions | **Fastest** (~2.5s) | **Unstable** - occasional MCU crashes | Functional but risky |
| Quadratic | **Smoothest**, fluid motion | Slowest (~3.5s) | **Most stable** - no crashes | Best quality, acceptable speed |
| Trapezoidal | Smooth, minor transitions | Moderate (~3.0s) | **Stable** - no crashes | Good balance for speed |

### Detailed Qualitative Observations

**No Scaling (Linear - Constant Speed):**
- **Motion quality:** Jerky and abrupt, especially at gait start/end
- **Execution time:** Fastest (~2.5s per cycle)
- **Servo behavior:** Audible strain during sudden starts; servos work hardest
- **MCU stability:** Occasional crashes due to current spikes when all 18 servos start simultaneously
- **Assessment:** Fast but risky - not recommended for production use

**Quadratic Scaling (Ease-in/out):**
- **Motion quality:** Smoothest and most fluid; natural-looking insect-like gait
- **Execution time:** Slowest (~3.5s per cycle) due to prolonged ease-in/ease-out phases
- **Servo behavior:** Minimal mechanical stress; quiet operation
- **MCU stability:** Perfectly stable - zero crashes observed across all testing
- **Assessment:** Best motion quality with acceptable speed tradeoff; recommended for reliability

**Trapezoidal Scaling (Constant cruise with ramps):**
- **Motion quality:** Smooth overall with minor jerkiness at ramp transitions (10%/90% points)
- **Execution time:** Moderate (~3.0s per cycle) - good balance
- **Servo behavior:** Reasonable stress levels; occasional noise at transitions
- **MCU stability:** Stable - no crashes observed
- **Assessment:** Excellent middle-ground option when speed is important but smoothness still desired

### Interpretation

The results strongly support our hypothesis that time scaling improves locomotion quality, though with an important speed tradeoff:

**Key Findings:**

1. **Quadratic scaling provides best quality but slowest speed:**
   - Eliminates sudden starts/stops that cause servo strain and current spikes
   - Spreads acceleration over entire motion, reducing peak current draw
   - Zero MCU crashes demonstrate perfect stability for long-term operation
   - 40% slower than no scaling (~3.5s vs ~2.5s per cycle)

2. **Trapezoidal scaling offers practical balance:**
   - Only 10% ease-in/out zones provide sufficient smoothing
   - 20% faster than quadratic while maintaining stability
   - Minor jerkiness at transition points (10%/90%) is acceptable for most applications
   - **Recommended for time-sensitive applications** where smoothness is still important

3. **No scaling is risky despite speed advantage:**
   - Fastest execution but prone to MCU crashes from simultaneous servo starts
   - Not suitable for autonomous operation without human supervision
   - Excessive mechanical stress may reduce servo lifespan

**Practical Recommendation:** For this hexapod platform, trapezoidal scaling provides the best balance of speed and reliability. Quadratic scaling should be used when demonstration quality or long-term reliability is paramount.

### Limitations

1. **Single direction testing:** Only forward (+Y) walking was evaluated; lateral or turning motions may behave differently
2. **Controlled environment:** Flat, hard surface with no obstacles or terrain variation
3. **No load testing:** Empty robot; payload would increase inertia and affect optimal scaling parameters
4. **Limited sample size:** Only 5 cycles per condition due to manual reset requirements

**Workspace Shrinkage Impact:**
The 19% radius reduction at lifted heights (83mm → 67mm) forced aggressive stride reduction. This explains why some configurations approached workspace limits and failed—the safety margin becomes critical when legs are at extreme angles (120°, 240°) where asymmetry is most pronounced.

### Relation to Project Goals

The primary goal—"stable walking while respecting workspace constraints"—was successfully achieved using quadratic time scaling. The system demonstrates that:
- Analytical IK provides fast, deterministic control
- Pre-computed workspace analysis prevents runtime failures
- Time scaling is essential for hardware longevity and reliability

The position-aware pre-positioning system (smooth interpolation from current positions) eliminated the MCU crash issue entirely, proving that motion planning at multiple timescales (pre-positioning + gait execution) is necessary for real-world robustness.

---

## Reflection and Future Work

### Key Learnings

1. **Workspace geometry is height-dependent:** This non-obvious constraint required careful analysis and directly shaped the gait design. Future robot designs should consider this during mechanical design, perhaps using link lengths that maintain more uniform workspace across Z-heights.

2. **Hardware limitations drive software architecture:** The MCU crash problem forced us to rethink initialization, leading to the pre-positioning system. This taught the value of "graceful" motion—never assume the robot can instantly jump to a new configuration.

3. **Absolute positioning prevents drift:** Our initial relative-motion approach accumulated errors. Switching to absolute targets (calculated from workspace center) eliminated this entirely, demonstrating the importance of reference frames in long-duration autonomous operation.

### Future Improvements

**1. Dynamic Gait Adaptation**
Currently, stride length is fixed based on conservative workspace estimates. Implement real-time adjustment based on:
- Terrain slope (IMU feedback)
- Battery voltage (servo speed varies with voltage)
- Load detection (torque sensing)

**2. Omnidirectional Locomotion**
Extend beyond forward walking to enable:
- Lateral (crab) walking
- Rotation in place
- Smooth direction transitions (Bezier curves between gait orientations)

**3. Compliant Control**
Add impedance/force control for:
- Soft landing detection (reduce impact forces)
- Uneven terrain adaptation
- Object pushing without position tracking errors

**4. Higher-Level Behaviors**
Build upon stable walking primitive:
- Obstacle avoidance using sensors (ultrasonic, lidar)
- Path planning with ROS navigation stack
- Vision-based target following

**5. Energy Optimization**
Profile power consumption across scaling functions to find minimum-energy gait while maintaining stability. Consider:
- Stride frequency vs. stride length tradeoffs
- Static stability margins
- Regenerative braking during leg retraction

**6. Reinforcement Learning Control (In Progress)**
Currently developing an RL-based locomotion policy with sim-to-real transfer:
- Training in simulation environment (PyBullet/Isaac Gym)
- Domain randomization for robust real-world deployment
- Policy deployment to Teensy 4.0 for onboard inference
- Target: adaptive gait generation without hand-crafted trajectories

Additional ML applications:
- Terrain classifier (adjust gait parameters automatically)
- Predictive failure detection (servo temperature, position errors)

### Broader Impact

This project demonstrates that sophisticated locomotion is achievable on resource-constrained embedded systems without expensive sensors or actuators. The workspace analysis methodology and time-scaling framework are transferable to other legged robots, robotic arms, and multi-DOF systems. The emphasis on "graceful degradation" (pre-positioning, absolute positioning, conservative safety factors) provides a template for robust autonomous systems in uncertain environments.

---

