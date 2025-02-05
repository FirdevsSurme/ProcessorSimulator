import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from enum import Enum
import re

class Format(Enum):
    R = "R"  # Register format
    I = "I"  # Immediate format
    J = "J"  # Jump format

class InstructionType:
    def __init__(self, format_type, opcode, funct=None):
        self.format = format_type
        self.opcode = opcode
        self.funct = funct

class Register:
    def __init__(self, name, number):
        self.name = name
        self.number = number
        self._value = 0

    @property
    def value(self):
        if self.number == 0:  # R0 is always 0
            return 0
        return self._value & 0xFFFF  # Just return the 16-bit value without sign conversion

    @value.setter
    def value(self, new_value):
        if self.number != 0:  # Can't modify R0
            self._value = new_value & 0xFFFF

class Memory:
    def __init__(self, size=512):
        self.size = size
        self.data = bytearray(size)

    def read_byte(self, address):
        if not (0 <= address < self.size):
            raise MemoryError(f"Invalid memory read at address {address}")
        return self.data[address]

    def write_byte(self, address, value):
        if not (0 <= address < self.size):
            raise MemoryError(f"Invalid memory write at address {address}")
        self.data[address] = value & 0xFF

    def read_word(self, address):
        if not (0 <= address < self.size - 1):
            raise MemoryError(f"Invalid memory read at address {address}")
        if address % 2 != 0:
            raise MemoryError(f"Unaligned memory access at address {address}")
        return (self.read_byte(address) << 8) | self.read_byte(address + 1)

    def write_word(self, address, value):
        if not (0 <= address < self.size - 1):
            raise MemoryError(f"Invalid memory write at address {address}")
        if address % 2 != 0:
            raise MemoryError(f"Unaligned memory access at address {address}")
        self.write_byte(address, (value >> 8) & 0xFF)
        self.write_byte(address + 1, value & 0xFF)

class PipelineStage:
    def __init__(self):
        self.instruction = None
        self.pc = 0
        self.data = {}
        self.control = {}
        self.stalled = False
        self.bubble = False

class ForwardingUnit:
    def __init__(self):
        self.ex_mem_rd = None
        self.mem_wb_rd = None
        self.ex_mem_reg_write = False
        self.mem_wb_reg_write = False

    def check_forwarding(self, id_ex_rs, id_ex_rt):
        forward_a = "00"  # No forwarding
        forward_b = "00"  # No forwarding

        # EX/MEM hazard
        if (self.ex_mem_reg_write and 
            self.ex_mem_rd is not None and 
            self.ex_mem_rd != 0):
            if self.ex_mem_rd == id_ex_rs:
                forward_a = "10"
            if self.ex_mem_rd == id_ex_rt:
                forward_b = "10"

        # MEM/WB hazard
        if (self.mem_wb_reg_write and 
            self.mem_wb_rd is not None and 
            self.mem_wb_rd != 0):
            if (self.mem_wb_rd == id_ex_rs and 
                not (self.ex_mem_reg_write and 
                     self.ex_mem_rd != 0 and 
                     self.ex_mem_rd == id_ex_rs)):
                forward_a = "01"
            if (self.mem_wb_rd == id_ex_rt and 
                not (self.ex_mem_reg_write and 
                     self.ex_mem_rd != 0 and 
                     self.ex_mem_rd == id_ex_rt)):
                forward_b = "01"

        return forward_a, forward_b

class HazardDetectionUnit:
    def __init__(self):
        self.id_ex_mem_read = False
        self.id_ex_rt = None
        self.if_id_rs = None
        self.if_id_rt = None
        self.hazard_history = []  # Hazard geçmişini tutmak için
        self.current_hazards = []  # Mevcut cycle'daki hazardlar

    def check_hazard(self, pipeline_stages):
        self.current_hazards = []
        
        id_stage = pipeline_stages['ID']
        ex_stage = pipeline_stages['EX']
        mem_stage = pipeline_stages['MEM']
        
        if not id_stage.instruction:
            return False

        # Data Hazards (RAW)
        if ex_stage.instruction:
            ex_dest = ex_stage.data.get('rd') if ex_stage.control.get('reg_dst') else ex_stage.data.get('rt')
            if ex_dest is not None and ex_stage.control.get('reg_write'):
                if id_stage.data.get('rs') == ex_dest:
                    self.current_hazards.append({
                        'type': 'RAW Hazard',
                        'description': f'Register R{ex_dest} is written in EX, needed in ID',
                        'solution': 'Forwarding from EX'
                    })

        # Load-Use Hazard
        if ex_stage.instruction and ex_stage.control.get('mem_read'):
            ex_rt = ex_stage.data.get('rt')
            if ex_rt is not None:
                if (id_stage.data.get('rs') == ex_rt or 
                    id_stage.data.get('rt') == ex_rt):
                    self.current_hazards.append({
                        'type': 'Load-Use Hazard',
                        'description': f'Loading into R{ex_rt}, needed in next instruction',
                        'solution': 'Pipeline Stall'
                    })
                    return True  # Stall gerekiyor

        # Hazard geçmişini güncelle
        if self.current_hazards:
            self.hazard_history.append({
                'cycle': len(self.hazard_history),
                'hazards': self.current_hazards.copy()
            })
            
        return False

    def get_current_hazards(self):
        return self.current_hazards

    def get_hazard_history(self):
        return self.hazard_history

class Pipeline:
    def __init__(self):
        self.stages = {
            'IF': PipelineStage(),
            'ID': PipelineStage(),
            'EX': PipelineStage(),
            'MEM': PipelineStage(),
            'WB': PipelineStage()
        }
        self.forwarding_unit = ForwardingUnit()
        self.hazard_detection = HazardDetectionUnit()
        self.cycle = 0
        self.stalled = False
        self.history = []

    def advance(self):
        self.history.append(self.get_state())
        
        # Move instructions forward (in reverse order)
        self.stages['WB'] = self.stages['MEM']
        self.stages['MEM'] = self.stages['EX']
        self.stages['EX'] = self.stages['ID']
        self.stages['ID'] = self.stages['IF']
        self.stages['IF'] = PipelineStage()
        
        self.cycle += 1

    def get_state(self):
        return {
            'cycle': self.cycle,
            'stages': {
                name: {
                    'instruction': stage.instruction.text if stage.instruction else None,
                    'pc': stage.pc,
                    'stalled': stage.stalled,
                    'bubble': stage.bubble,
                    'data': stage.data.copy(),
                    'control': stage.control.copy()
                } for name, stage in self.stages.items()
            }
        }

    def check_hazard(self):
        return self.hazard_detection.check_hazard(self.stages)

class Instruction:
    # Instruction set definition
    TYPES = {
        'add':  InstructionType(Format.R, "0000", "000"),
        'sub':  InstructionType(Format.R, "0000", "001"),
        'and':  InstructionType(Format.R, "0000", "010"),
        'or':   InstructionType(Format.R, "0000", "011"),
        'slt':  InstructionType(Format.R, "0000", "100"),
        'sll':  InstructionType(Format.R, "0000", "101"),
        'srl':  InstructionType(Format.R, "0000", "110"),
        'jr':   InstructionType(Format.R, "0000", "111"),
        'addi': InstructionType(Format.I, "0001"),
        'lw':   InstructionType(Format.I, "0010"),
        'sw':   InstructionType(Format.I, "0011"),
        'beq':  InstructionType(Format.I, "0100"),
        'bne':  InstructionType(Format.I, "0101"),
        'j':    InstructionType(Format.J, "0110"),
        'jal':  InstructionType(Format.J, "0111")
    }

    def __init__(self, text):
        self.text = text.split('#')[0].strip()
        self.type = None
        self.opcode = None
        self.rs = None
        self.rt = None
        self.rd = None
        self.shamt = None
        self.funct = None
        self.immediate = None
        self.target = None
        
        if self.text:
            self.parse_instruction()

    def parse_instruction(self):
        parts = self.text.replace(',', ' ').split()
        if not parts:
            return

        op = parts[0].lower()
        if op not in self.TYPES:
            raise ValueError(f"Unknown instruction: {op}")

        inst_type = self.TYPES[op]
        self.type = inst_type.format
        self.opcode = inst_type.opcode

        args = [arg.strip().upper() for arg in parts[1:]]

        try:
            if self.type == Format.R:
                if op in ['sll', 'srl']:
                    self.rd = self._parse_register(args[0])
                    self.rt = self._parse_register(args[1])
                    self.shamt = int(args[2])
                else:
                    self.rd = self._parse_register(args[0])
                    self.rs = self._parse_register(args[1])
                    self.rt = self._parse_register(args[2])
                self.funct = inst_type.funct

            elif self.type == Format.I:
                if op in ['lw', 'sw']:
                    self.rt = self._parse_register(args[0])
                    offset_base = args[1].split('(')
                    self.immediate = int(offset_base[0])
                    self.rs = self._parse_register(offset_base[1].replace(')', ''))
                else:  # addi
                    self.rt = self._parse_register(args[0])
                    self.rs = self._parse_register(args[1])
                    self.immediate = int(args[2])

        except (IndexError, ValueError) as e:
            raise ValueError(f"Error parsing instruction '{self.text}': {str(e)}")

    def _parse_register(self, reg_str):
        if not reg_str.startswith('R'):
            raise ValueError(f"Invalid register format: {reg_str}")
        reg_num = int(reg_str[1:])
        if not (0 <= reg_num <= 7):
            raise ValueError(f"Invalid register number: {reg_num}")
        return reg_num

class MIPSProcessor:
    def __init__(self):
        self.registers = [Register(f"R{i}", i) for i in range(8)]
        self.instruction_memory = Memory(512)
        self.data_memory = Memory(512)
        self.pc = 0
        self.pipeline = Pipeline()
        self.instructions = []

    def load_program(self, program_text):
        self.instructions = []
        self.pc = 0
        
        for line in program_text.split('\n'):
            line = line.strip()
            if line and not line.startswith('#'):
                instruction = Instruction(line)
                self.instructions.append(instruction)

    def fetch(self):
        if self.pc >= len(self.instructions) * 2:
            return None
            
        stage = self.pipeline.stages['IF']
        if not self.pipeline.stalled:
            stage.instruction = self.instructions[self.pc // 2]
            stage.pc = self.pc
            self.pc += 2
        return stage

    def decode(self, stage):
        if not stage.instruction:
            return

        inst = stage.instruction
        stage.data['op'] = inst.text.split()[0].lower()
        stage.data['rs'] = inst.rs
        stage.data['rt'] = inst.rt
        stage.data['rd'] = inst.rd
        stage.data['shamt'] = inst.shamt
        stage.data['immediate'] = inst.immediate

        print(f"\nDecode Stage:")
        print(f"Instruction: {inst.text}")
        print(f"rs: {stage.data['rs']}, rt: {stage.data['rt']}, rd: {stage.data['rd']}")
        print(f"immediate: {stage.data['immediate']}")

        # Set control signals based on instruction type
        if inst.type == Format.R:
            stage.control = {
                'reg_write': True,
                'mem_read': False,
                'mem_write': False,
                'alu_src': False,
                'reg_dst': True,
                'branch': False
            }
        elif inst.type == Format.I:
            if stage.data['op'] == 'lw':
                stage.control = {
                    'reg_write': True,
                    'mem_read': True,
                    'mem_write': False,
                    'alu_src': True,
                    'reg_dst': False,
                    'branch': False
                }
            elif stage.data['op'] == 'sw':
                stage.control = {
                    'reg_write': False,
                    'mem_read': False,
                    'mem_write': True,
                    'alu_src': True,
                    'reg_dst': False,
                    'branch': False
                }
            elif stage.data['op'] == 'addi':
                stage.control = {
                    'reg_write': True,
                    'mem_read': False,
                    'mem_write': False,
                    'alu_src': True,
                    'reg_dst': False,
                    'branch': False
                }
        print(f"Control signals: {stage.control}")

    def execute(self, stage):
        if not stage.instruction:
            return

        op = stage.data['op']
        rs = stage.data['rs']
        rt = stage.data['rt']
        
        # Get register values with forwarding
        rs_val = self.get_forwarded_value('rs', stage)
        rt_val = self.get_forwarded_value('rt', stage)

        print(f"\nExecute Stage:")
        print(f"Operation: {op}")
        print(f"Values: rs(R{rs})={rs_val}, rt(R{rt})={rt_val}")

        # ALU operations
        result = None
        if op == 'add':
            result = rs_val + rt_val
        elif op == 'addi':
            imm = stage.data['immediate']
            result = rs_val + imm
        elif op == 'sub':
            result = rs_val - rt_val
            if result < 0:
                result = (result + 0x10000)
        elif op == 'and':
            result = rs_val & rt_val
        elif op == 'sll':
            shamt = stage.data['shamt']
            result = rt_val << shamt
        elif op in ['lw', 'sw']:
            result = rs_val + stage.data['immediate']

        # Ensure 16-bit result
        if result is not None:
            stage.data['alu_result'] = result & 0xFFFF
            print(f"ALU Result: {stage.data['alu_result']}")

    def memory_access(self, stage):
        if not stage.instruction:
            return

        print(f"\nMemory Stage:")
        print(f"Instruction: {stage.instruction.text}")

        try:
            if stage.control.get('mem_read'):  # lw
                addr = stage.data['alu_result']
                value = self.data_memory.read_word(addr)
                stage.data['mem_data'] = value
                print(f"Load: Memory[{addr}] = {value}")
                
            elif stage.control.get('mem_write'):  # sw
                addr = stage.data['alu_result']
                value = self.registers[stage.data['rt']].value
                self.data_memory.write_word(addr, value)
                print(f"Store: Memory[{addr}] = {value}")
                
        except Exception as e:
            print(f"Memory Error: {str(e)}")
            raise

    def writeback(self, stage):
        if not stage.instruction:
            return

        print(f"\nWriteback Stage:")
        print(f"Instruction: {stage.instruction.text}")

        if stage.control.get('reg_write'):
            # Get value from memory or ALU
            if stage.control.get('mem_read'):  # lw instruction
                value = stage.data['mem_data']
            else:  # ALU operation
                value = stage.data['alu_result']

            # Determine destination register
            reg_num = stage.data['rd'] if stage.control.get('reg_dst') else stage.data['rt']
            
            # Write to register if not R0
            if reg_num != 0:
                old_value = self.registers[reg_num].value
                self.registers[reg_num].value = value
                print(f"Register Write: R{reg_num} = {value} (was {old_value})")

    def get_forwarded_value(self, reg_type, stage):
        reg_num = stage.data[reg_type]
        if reg_num is None:
            return 0
        
        # First check MEM stage for forwarding
        mem_stage = self.pipeline.stages['MEM']
        if (mem_stage.instruction and 
            mem_stage.control.get('reg_write')):
            dest_reg = mem_stage.data['rd'] if mem_stage.control.get('reg_dst') else mem_stage.data['rt']
            if dest_reg == reg_num:
                if mem_stage.control.get('mem_read'):
                    return mem_stage.data['mem_data']
                return mem_stage.data['alu_result']
        
        # Then check WB stage
        wb_stage = self.pipeline.stages['WB']
        if (wb_stage.instruction and 
            wb_stage.control.get('reg_write')):
            dest_reg = wb_stage.data['rd'] if wb_stage.control.get('reg_dst') else wb_stage.data['rt']
            if dest_reg == reg_num:
                if wb_stage.control.get('mem_read'):
                    return wb_stage.data['mem_data']
                return wb_stage.data['alu_result']
        
        # If no forwarding needed, get from register file
        return self.registers[reg_num].value

    def step(self):
        print("\n=== Pipeline Step ===")
        
        # Save current pipeline state
        current_stages = {
            'WB': self.pipeline.stages['WB'],
            'MEM': self.pipeline.stages['MEM'],
            'EX': self.pipeline.stages['EX'],
            'ID': self.pipeline.stages['ID'],
            'IF': self.pipeline.stages['IF']
        }
        
        # Process stages in reverse order
        if current_stages['WB'].instruction:
            print("WB:", current_stages['WB'].instruction.text)
            self.writeback(current_stages['WB'])
        
        if current_stages['MEM'].instruction:
            print("MEM:", current_stages['MEM'].instruction.text)
            self.memory_access(current_stages['MEM'])
        
        if current_stages['EX'].instruction:
            print("EX:", current_stages['EX'].instruction.text)
            self.execute(current_stages['EX'])
        
        if current_stages['ID'].instruction:
            print("ID:", current_stages['ID'].instruction.text)
            self.decode(current_stages['ID'])
        
        # Check for hazards before fetch
        should_stall = self.pipeline.check_hazard()
        if not should_stall:
            print("IF: Fetching next instruction")
            self.fetch()
        else:
            print("Pipeline stalled due to hazard")
            current_stages['IF'].stalled = True
        
        # Update pipeline stages
        self.pipeline.stages['WB'] = current_stages['MEM']
        self.pipeline.stages['MEM'] = current_stages['EX']
        self.pipeline.stages['EX'] = current_stages['ID']
        self.pipeline.stages['ID'] = current_stages['IF']
        self.pipeline.stages['IF'] = PipelineStage()
        
        self.pipeline.cycle += 1

    def check_hazards(self):
        hazards = []
        
        # Get current stages
        id_stage = self.pipeline.stages['ID']
        ex_stage = self.pipeline.stages['EX']
        mem_stage = self.pipeline.stages['MEM']
        
        if not id_stage.instruction:
            return hazards

        # Get current instruction's register dependencies
        curr_rs = id_stage.data.get('rs')
        curr_rt = id_stage.data.get('rt')
        
        # Check RAW hazards with EX stage
        if ex_stage.instruction:
            ex_dest = ex_stage.data.get('rd') if ex_stage.control.get('reg_dst') else ex_stage.data.get('rt')
            if ex_dest is not None and ex_stage.control.get('reg_write'):
                if curr_rs == ex_dest:
                    hazards.append({
                        'type': 'RAW Hazard',
                        'description': f'Register R{ex_dest} is being written in EX and read in ID',
                        'solution': 'Forwarding'
                    })
                if curr_rt == ex_dest:
                    hazards.append({
                        'type': 'RAW Hazard',
                        'description': f'Register R{ex_dest} is being written in EX and read in ID',
                        'solution': 'Forwarding'
                    })

        # Check RAW hazards with MEM stage
        if mem_stage.instruction:
            mem_dest = mem_stage.data.get('rd') if mem_stage.control.get('reg_dst') else mem_stage.data.get('rt')
            if mem_dest is not None and mem_stage.control.get('reg_write'):
                if curr_rs == mem_dest:
                    hazards.append({
                        'type': 'RAW Hazard',
                        'description': f'Register R{mem_dest} is being written in MEM and read in ID',
                        'solution': 'Forwarding'
                    })
                if curr_rt == mem_dest:
                    hazards.append({
                        'type': 'RAW Hazard',
                        'description': f'Register R{mem_dest} is being written in MEM and read in ID',
                        'solution': 'Forwarding'
                    })

        # Check Load-Use hazards
        if ex_stage.instruction and ex_stage.control.get('mem_read'):
            ex_dest = ex_stage.data.get('rt')
            if ex_dest is not None:
                if curr_rs == ex_dest or curr_rt == ex_dest:
                    hazards.append({
                        'type': 'Load-Use Hazard',
                        'description': f'Loading into R{ex_dest} in EX, needed in next instruction',
                        'solution': 'Pipeline Stall'
                    })

        return hazards

class MIPSSimulatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("16-bit MIPS Pipeline Simulator")
        self.processor = MIPSProcessor()
        self.create_gui()
        self.update_display()  # Initialize display with empty memory

    def create_gui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(expand=True, fill='both')
        
        # Left panel
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill='both', expand=True)
        
        # Code input
        code_frame = ttk.LabelFrame(left_frame, text="Assembly Code")
        code_frame.pack(fill='both', expand=True, padx=5)
        
        self.code_text = scrolledtext.ScrolledText(code_frame, width=40, height=20)
        self.code_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Control buttons
        control_frame = ttk.Frame(left_frame)
        control_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(control_frame, text="Load", command=self.load_program).pack(side=tk.LEFT, padx=2)
        ttk.Button(control_frame, text="Step", command=self.step).pack(side=tk.LEFT, padx=2)
        ttk.Button(control_frame, text="Run", command=self.run).pack(side=tk.LEFT, padx=2)
        ttk.Button(control_frame, text="Reset", command=self.reset).pack(side=tk.LEFT, padx=2)
        
        # Right panel
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill='both', expand=True)
        
        # Pipeline stages
        pipeline_frame = ttk.LabelFrame(right_frame, text="Pipeline Stages")
        pipeline_frame.pack(fill='both', expand=True, pady=5)
        
        self.stage_texts = {}
        for stage in ['IF', 'ID', 'EX', 'MEM', 'WB']:
            stage_frame = ttk.LabelFrame(pipeline_frame, text=stage)
            stage_frame.pack(side=tk.LEFT, fill='both', expand=True, padx=2)
            
            text = scrolledtext.ScrolledText(stage_frame, width=20, height=6)
            text.pack(fill='both', expand=True)
            self.stage_texts[stage] = text
        
        # Memory displays
        bottom_frame = ttk.Frame(right_frame)
        bottom_frame.pack(fill='both', expand=True)
        
        # Register display
        reg_frame = ttk.LabelFrame(bottom_frame, text="Registers")
        reg_frame.pack(side=tk.LEFT, fill='both', expand=True, padx=5)
        
        # Update register treeview to include hex and binary columns
        self.reg_tree = ttk.Treeview(reg_frame, columns=('Dec', 'Hex', 'Binary'), height=8)
        self.reg_tree.heading('#0', text='Register')
        self.reg_tree.heading('Dec', text='Decimal')
        self.reg_tree.heading('Hex', text='Hex')
        self.reg_tree.heading('Binary', text='Binary')
        
        # Configure column widths
        self.reg_tree.column('#0', width=80)  # Register name
        self.reg_tree.column('Dec', width=80)  # Decimal value
        self.reg_tree.column('Hex', width=80)  # Hex value
        self.reg_tree.column('Binary', width=150)  # Binary value
        
        # Add styling
        self.reg_tree.tag_configure('modified', background='yellow')
        self.reg_tree.pack(fill='both', expand=True)
        
        # Memory displays frame
        memories_frame = ttk.Frame(bottom_frame)
        memories_frame.pack(side=tk.RIGHT, fill='both', expand=True, padx=5)
        
        # Data Memory display
        data_mem_frame = ttk.LabelFrame(memories_frame, text="Data Memory")
        data_mem_frame.pack(fill='both', expand=True, pady=5)
        
        # Update data memory treeview to include decimal values
        self.data_mem_tree = ttk.Treeview(data_mem_frame, columns=('Dec', 'Hex', 'Binary'), height=8)
        self.data_mem_tree.heading('#0', text='Address')
        self.data_mem_tree.heading('Dec', text='Decimal')
        self.data_mem_tree.heading('Hex', text='Hex')
        self.data_mem_tree.heading('Binary', text='Binary')
        
        # Configure column widths for data memory
        self.data_mem_tree.column('#0', width=100)  # Address column
        self.data_mem_tree.column('Dec', width=80)  # Decimal value
        self.data_mem_tree.column('Hex', width=100)  # Hex value
        self.data_mem_tree.column('Binary', width=150)  # Binary value
        
        # Data Memory display styling
        self.data_mem_tree.tag_configure('modified', background='yellow')
        self.data_mem_tree.tag_configure('accessed', background='lightblue')
        self.data_mem_tree.pack(fill='both', expand=True)
        
        # Instruction Memory display
        inst_mem_frame = ttk.LabelFrame(memories_frame, text="Instruction Memory")
        inst_mem_frame.pack(fill='both', expand=True, pady=5)
        
        self.inst_mem_tree = ttk.Treeview(inst_mem_frame, columns=('Instruction'), height=8)
        self.inst_mem_tree.heading('#0', text='Address')
        self.inst_mem_tree.heading('Instruction', text='Instruction')
        self.inst_mem_tree.column('Instruction', width=200)
        self.inst_mem_tree.pack(fill='both', expand=True)
        
        # Instruction Memory display styling
        self.inst_mem_tree.tag_configure('current', background='lightgreen')
        self.inst_mem_tree.column('#0', width=100)  # Address column
        self.inst_mem_tree.column('Instruction', width=250)

        # Cycle counter
        cycle_frame = ttk.LabelFrame(right_frame, text="Cycle")
        cycle_frame.pack(fill='x', pady=5)
        self.cycle_label = ttk.Label(cycle_frame, text="0")
        self.cycle_label.pack(padx=5, pady=2)

        # Hazard Display
        hazard_frame = ttk.LabelFrame(right_frame, text="Hazard Information")
        hazard_frame.pack(fill='x', pady=5)
        
        # Current Hazards
        self.hazard_tree = ttk.Treeview(hazard_frame, 
                                       columns=('Type', 'Description', 'Solution'),
                                       height=4)
        self.hazard_tree.heading('#0', text='Cycle')
        self.hazard_tree.heading('Type', text='Hazard Type')
        self.hazard_tree.heading('Description', text='Description')
        self.hazard_tree.heading('Solution', text='Solution')
        
        self.hazard_tree.column('#0', width=60)
        self.hazard_tree.column('Type', width=100)
        self.hazard_tree.column('Description', width=250)
        self.hazard_tree.column('Solution', width=150)
        
        # Hazard renklendirme
        self.hazard_tree.tag_configure('raw_hazard', background='#ffcccb')
        self.hazard_tree.tag_configure('load_use_hazard', background='#ffdab9')
        self.hazard_tree.pack(fill='x', padx=5, pady=5)

    def update_display(self):
        # Update pipeline stages
        for stage_name, text_widget in self.stage_texts.items():
            text_widget.delete('1.0', tk.END)
            stage = self.processor.pipeline.stages[stage_name]
            
            if stage_name == 'IF':
                # IF stage specific display
                text_widget.insert(tk.END, "=== Fetch Stage ===\n\n")
                
                # Program henüz başlamadıysa
                if self.processor.pc == 0 and not stage.instruction:
                    text_widget.insert(tk.END, "Status: Ready to start\n")
                    text_widget.insert(tk.END, "PC: 0x0000\n")
                
                # Program devam ediyorsa
                elif self.processor.pc < len(self.processor.instructions) * 2:
                    next_inst_idx = self.processor.pc // 2
                    if next_inst_idx < len(self.processor.instructions):
                        next_inst = self.processor.instructions[next_inst_idx]
                        text_widget.insert(tk.END, f"Fetching Instruction:\n{next_inst.text}\n")
                        text_widget.insert(tk.END, f"Current PC: 0x{self.processor.pc:04x}\n")
                        text_widget.insert(tk.END, f"Next PC: 0x{self.processor.pc + 2:04x}\n")
                        
                        # Stall durumu varsa göster
                        if stage.stalled:
                            text_widget.insert(tk.END, "\nStatus: STALLED\n")
                            text_widget.insert(tk.END, "Reason: Pipeline Hazard\n")
                            text_widget.tag_configure('stall', background='#ffdab9')
                            text_widget.tag_add('stall', '1.0', 'end')
                
                # Program bittiyse
                else:
                    text_widget.insert(tk.END, "Status: Program completed\n")
                    text_widget.insert(tk.END, f"Final PC: 0x{self.processor.pc:04x}\n")
                    text_widget.insert(tk.END, f"Total Instructions: {len(self.processor.instructions)}\n")
                    text_widget.insert(tk.END, f"Total Cycles: {self.processor.pipeline.cycle}\n")
            else:
                # Diğer aşamaların mevcut gösterimi
                if stage.instruction:
                    text_widget.insert(tk.END, f"Instruction: {stage.instruction.text}\n")
                    text_widget.insert(tk.END, f"PC: 0x{stage.pc:04x}\n\n")
                    
                    if stage.data:
                        text_widget.insert(tk.END, "Data:\n")
                        for key, value in stage.data.items():
                            if isinstance(value, int):
                                text_widget.insert(tk.END, f"{key}: 0x{value:04x}\n")
                            else:
                                text_widget.insert(tk.END, f"{key}: {value}\n")
                    
                    if stage.control:
                        text_widget.insert(tk.END, "\nControl:\n")
                        for key, value in stage.control.items():
                            text_widget.insert(tk.END, f"{key}: {value}\n")

                # Hazard ve forwarding vurgulamaları
                if stage.stalled:
                    text_widget.tag_configure('hazard', background='#ffdab9')
                    text_widget.tag_add('hazard', '1.0', 'end')
                elif stage_name == 'IF' and stage.instruction:
                    text_widget.tag_configure('fetch', background='#e6ffe6')
                    text_widget.tag_add('fetch', '1.0', 'end')

        # Update registers with all formats
        self.reg_tree.delete(*self.reg_tree.get_children())
        for reg in self.processor.registers:
            value = reg.value
            hex_value = f"0x{value:04x}"  # 4-digit hex
            binary = format(value, '016b')  # 16-bit binary
            
            # Insert with all formats
            self.reg_tree.insert('', 'end', 
                               text=f'R{reg.number}', 
                               values=(str(value), hex_value, binary))
        
        # Update data memory with all formats
        self.data_mem_tree.delete(*self.data_mem_tree.get_children())
        for addr in range(0, self.processor.data_memory.size, 2):
            try:
                value = self.processor.data_memory.read_word(addr)
                # Convert to signed value if needed
                signed_value = value if value < 0x8000 else value - 0x10000
                hex_value = f"0x{value:04x}"
                binary = format(value & 0xFFFF, '016b')
                
                self.data_mem_tree.insert('', 'end', 
                                        text=f'0x{addr:04x}', 
                                        values=(str(signed_value), hex_value, binary))
            except:
                continue
        
        # Update instruction memory - show all instructions
        self.inst_mem_tree.delete(*self.inst_mem_tree.get_children())
        for i in range(256):  # 512 bytes / 2 = 256 possible instructions
            addr = i * 2
            inst = self.processor.instructions[i] if i < len(self.processor.instructions) else None
            if inst:
                self.inst_mem_tree.insert('', 'end',
                                        text=f'0x{addr:04x}',
                                        values=(inst.text,))
            else:
                self.inst_mem_tree.insert('', 'end',
                                        text=f'0x{addr:04x}',
                                        values=('--',))

        # Highlight current instruction in memory
        current_pc = self.processor.pc - 2  # Previous instruction
        if current_pc >= 0:
            for item in self.inst_mem_tree.get_children():
                addr = int(self.inst_mem_tree.item(item)['text'], 16)
                if addr == current_pc:
                    self.inst_mem_tree.selection_set(item)
                    self.inst_mem_tree.see(item)
                    break

        # Update memory access highlighting
        for stage in self.processor.pipeline.stages.values():
            if stage.instruction:
                if stage.control.get('mem_read') or stage.control.get('mem_write'):
                    addr = stage.data.get('alu_result')
                    if addr is not None:
                        for item in self.data_mem_tree.get_children():
                            mem_addr = int(self.data_mem_tree.item(item)['text'], 16)
                            if mem_addr == addr:
                                self.data_mem_tree.selection_set(item)
                                self.data_mem_tree.see(item)
                                break

        # Update cycle counter
        self.cycle_label.config(text=str(self.processor.pipeline.cycle))

        # Update hazard display
        self.hazard_tree.delete(*self.hazard_tree.get_children())
        current_hazards = self.processor.pipeline.hazard_detection.get_current_hazards()
        
        if current_hazards:
            for hazard in current_hazards:
                tag = 'raw_hazard' if 'RAW' in hazard['type'] else 'load_use_hazard'
                self.hazard_tree.insert('', 'end',
                                      text=str(self.processor.pipeline.cycle),
                                      values=(hazard['type'],
                                             hazard['description'],
                                             hazard['solution']),
                                      tags=(tag,))
        
        # Update pipeline stage displays with hazard information
        for stage_name, text_widget in self.stage_texts.items():
            stage = self.processor.pipeline.stages[stage_name]
            if stage.stalled:
                text_widget.tag_configure('hazard', background='#ffdab9')
                text_widget.tag_add('hazard', '1.0', 'end')
            elif stage_name == 'EX' and stage.data.get('forwarded_from'):
                text_widget.tag_configure('forward', background='#ccffcc')
                text_widget.tag_add('forward', '1.0', 'end')

    def load_program(self):
        code = self.code_text.get('1.0', tk.END).strip()
        if not code:
            return
            
        try:
            self.processor = MIPSProcessor()
            self.processor.load_program(code)
            self.update_display()
            messagebox.showinfo("Success", "Program loaded successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load program: {str(e)}")

    def step(self):
        try:
            self.processor.step()
            self.update_display()
        except Exception as e:
            messagebox.showerror("Error", f"Error during execution: {str(e)}")

    def run(self):
        try:
            while self.processor.pc < len(self.processor.instructions) * 2:
                self.step()
                self.root.update()
                self.root.after(100)
        except Exception as e:
            messagebox.showerror("Error", f"Error during execution: {str(e)}")

    def reset(self):
        self.processor = MIPSProcessor()
        self.update_display()

    def update_pipeline_display(self, stage_name, text_widget, stage):
        text_widget.delete('1.0', tk.END)
        if stage.instruction:
            text_widget.insert(tk.END, f"Instruction: {stage.instruction.text}\n")
            text_widget.insert(tk.END, f"PC: {stage.pc}\n\n")
            
            # Show hazard information
            if stage.stalled:
                text_widget.insert(tk.END, "STATUS: STALLED (Hazard)\n", 'hazard')
            elif stage.bubble:
                text_widget.insert(tk.END, "STATUS: BUBBLE\n", 'bubble')
            
            # Show forwarding information
            if 'forwarded_from' in stage.data:
                text_widget.insert(tk.END, 
                                 f"Forwarding: {stage.data['forwarded_from']}\n",
                                 'forward')

class BranchPredictor:
    def __init__(self):
        self.prediction_table = {}
        
    def predict(self, pc):
        return self.prediction_table.get(pc, False)
        
    def update(self, pc, taken):
        self.prediction_table[pc] = taken

def main():
    root = tk.Tk()
    app = MIPSSimulatorGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()