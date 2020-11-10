# FDS: Flight Dynamics Simulator

### **Library Requirements:**

- [x] `nlohmann/json`
- [x] `Eigen`
- [x] `boost`

### **Compilation Instructions:**

From within the `fds` directory, compile with `make`.
<br>
The `fds` directory can be cleaned with: `make clean`.

**Run the simulator:** `./FDS`

### **Altering Parameters:**

Aerodynamic derivatives and other aircraft parameters are defined in `fds/input/parameters.json`
<br>
The initial state of the aircraft is defined in `fds/input/state.json`.

Simulation parameters and global constants are defined within `fds/include/input.h` - upon altering
any of these parameters the code will need to be recompiled to ensure that changes are reflected in
the results.
