# Pipeline Hazard Handling

## Implemented Hazard Types:

Data Hazards (RAW): Detected and resolved using forwarding.

Load-Use Hazard: Pipeline stalling is used to resolve these hazards.

Control Hazards: Simple branch prediction and flushing are implemented.


# Forwarding Mechanism:

Forwarding from EX/MEM and MEM/WB stages to avoid data hazards.

Implements bypass logic for register values in dependent instructions.


# Stalling:

Load-use dependencies cause the pipeline to stall for one cycle.

Control hazards trigger pipeline flush and PC correction.


# Example Test Cases (SystemVerilog)

The testbench includes multiple scenarios:

1. Basic Arithmetic Operations (ADD, SUB, AND, OR, SLT, etc.)

2. Memory Operations (LW, SW)

3. Branch Instructions (BEQ, BNE)

4. Forwarding Mechanism

5. Load-Use Hazard Detection

6. Control Hazard Handling

7. Edge Case Handling (Memory Bounds, Register Conflicts)
7
