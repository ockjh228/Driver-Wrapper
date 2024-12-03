# ActuatorFeedback.msg
# Feedback for multiple actuators represented as arrays

float32[] position            # Array of actuator positions
float32[] velocity            # Array of actuator velocities
float32[] torque              # Array of actuator torques
float32[] current_motor       # Array of motor currents
float32[] voltage             # Array of motor voltages
float32[] temperature_motor   # Array of motor temperatures
float32[] temperature_core    # Array of core temperatures
uint32[] status_flags         # Array of status flags
uint32[] jitter_comm          # Array of communication jitters
uint32[] fault_bank_a         # Array of fault bank A values
uint32[] fault_bank_b         # Array of fault bank B values
uint32[] warning_bank_a       # Array of warning bank A values
uint32[] warning_bank_b       # Array of warning bank B values
