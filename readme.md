Pipeline Hazard Handling

Implemented Hazard Types:

Data Hazards (RAW): Detected and resolved using forwarding.

Load-Use Hazard: Pipeline stalling is used to resolve these hazards.

Control Hazards: Simple branch prediction and flushing are implemented.

Forwarding Mechanism:

Forwarding from EX/MEM and MEM/WB stages to avoid data hazards.

Implements bypass logic for register values in dependent instructions.

Stalling:

Load-use dependencies cause the pipeline to stall for one cycle.

Control hazards trigger pipeline flush and PC correction.

Example Test Cases (SystemVerilog)

The testbench includes multiple scenarios:

Basic Arithmetic Operations (ADD, SUB, AND, OR, SLT, etc.)

Memory Operations (LW, SW)

Branch Instructions (BEQ, BNE)

Forwarding Mechanism

Load-Use Hazard Detection

Control Hazard Handling

Edge Case Handling (Memory Bounds, Register Conflicts)
