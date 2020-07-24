"""CPU functionality."""

import sys
import datetime

HLT = 0b00000001
LDI = 0b10000010
ST = 0b10000100
PRN = 0b01000111

MUL = 0b10100010
ADD = 0b10100000
AND = 0b10101000
OR = 0b10101010
XOR = 0b10101011
NOT = 0b01101001
SHL = 0b10101100
SHR = 0b10101101
MOD = 0b10100100

POP = 0b01000110
PUSH = 0b01000101
CALL = 0b01010000
RET = 0b00010001
JMP = 0b01010100
CMP = 0b10100111
JEQ = 0b01010101
JNE = 0b01010110

INT = 0b01010010
IRET = 0b00010011

class CPU:
    """Main CPU class."""

    def __init__(self):
        """Construct a new CPU."""
        self.reg = [0]*8
        self.reg[7] = 0xF4
        self.ram = [0]*256
        self.PC = 0
        self.FL = 0

        self.branchtable = {}
        self.branchtable[HLT] = self.hlt
        self.branchtable[LDI] = self.ldi
        self.branchtable[ST] = self.st
        self.branchtable[PRN] = self.prn
        self.branchtable[MUL] = self.mul
        self.branchtable[ADD] = self.add
        self.branchtable[AND] = self.and_
        self.branchtable[OR] = self.or_
        self.branchtable[XOR] = self.xor
        self.branchtable[NOT] = self.not_
        self.branchtable[SHL] = self.shl
        self.branchtable[SHR] = self.shr
        self.branchtable[MOD] = self.mod
        self.branchtable[POP] = self.pop
        self.branchtable[PUSH] = self.push
        self.branchtable[CALL] = self.call
        self.branchtable[RET] = self.ret
        self.branchtable[JMP] = self.jmp
        self.branchtable[CMP] = self.cmp
        self.branchtable[JEQ] = self.jeq
        self.branchtable[JNE] = self.jne
        self.branchtable[INT] = self.int
        self.branchtable[IRET] = self.iret

    def hlt(self):
        exit()

    def ldi(self):
        self.reg[self.ram_read(self.PC+1)] = self.ram_read(self.PC+2)
        self.PC += 2    

    def st(self):
        self.ram_write(
            self.reg[self.ram_read(self.PC+1)],
            self.reg[self.ram_read(self.PC+2)]
        )
        self.PC += 2

    def prn(self):
        print(self.reg[self.ram_read(self.PC+1)])
        self.PC += 1

    def mul(self):
        self.reg[self.ram_read(self.PC+1)] *= self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def add(self):
        self.reg[self.ram_read(self.PC+1)] += self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def and_(self):
        self.reg[self.ram_read(self.PC+1)] &= self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def or_(self):
        self.reg[self.ram_read(self.PC+1)] |= self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def xor(self):
        self.reg[self.ram_read(self.PC+1)] ^= self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def not_(self):
        self.reg[self.ram_read(self.PC+1)] = ~self.reg[self.ram_read(self.PC+1)]
        self.PC += 1

    def shl(self):
        self.reg[self.ram_read(self.PC+1)] <<= self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def shr(self):
        self.reg[self.ram_read(self.PC+1)] >>= self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def mod(self):
        self.reg[self.ram_read(self.PC+1)] %= self.reg[self.ram_read(self.PC+2)]
        self.PC += 2

    def pop(self):
        self.reg[self.ram_read(self.PC+1)] = self.ram_read(self.reg[7])
        self.reg[7] += 1
        self.PC += 1

    def push(self):
        self.reg[7] -= 1
        self.ram_write(
            self.reg[self.ram_read(self.PC+1)],
            self.reg[7]
        )
        self.PC += 1

    def call(self):
        # push address of next instruction to stack
        self.reg[7] -= 1
        self.ram_write(
            self.PC+2,
            self.reg[7]
        )
        # jump to the address at the given register
        self.PC = self.reg[self.ram_read(self.PC+1)]

    def ret(self):
        self.PC = self.ram_read(self.reg[7])
        self.reg[7] += 1

    def jmp(self):
        self.PC = self.reg[self.ram_read(self.PC+1)]

    def cmp(self):
        # compare the two registers by subtracting
        res = self.reg[self.ram_read(self.PC+1)] - self.reg[self.ram_read(self.PC+2)]
        # are they equal?
        if res == 0:
            self.FL = 0b001
        # is regA greater than regB?
        elif res > 0:
            self.FL = 0b010
        # is regA less than regB?
        else:
            self.FL = 0b100
        self.PC += 2

    def jeq(self):
        # Is the equal flag set?
        if self.FL & 0b001 != 0:
            self.PC = self.reg[self.ram_read(self.PC+1)]
        else:
            self.PC += 2

    def jne(self):
        # is the equal flag unset?
        if self.FL & 0b001 == 0:
            self.PC = self.reg[self.ram_read(self.PC+1)]
        else:
            self.PC += 2

    def int(self):
        pass

    def iret(self):
        # pop registers 6-0
        for i in range(6, -1, -1):
            self.reg[i] = self.ram_read(self.reg[7])
            self.reg[7] += 1
        # pop FL register
        self.FL = self.ram_read(self.reg[7])
        self.reg[7] += 1
        # pop PC register
        self.PC = self.ram_read(self.reg[7])
        self.reg[7] += 1
        # reset IS register to 0
        self.reg[6] = 0

    def load(self, program):
        """Load a program into memory."""

        address = 0

        for instruction in program:
            self.ram[address] = instruction
            address += 1


    def alu(self, op, reg_a, reg_b):
        """ALU operations."""

        if op == "ADD":
            self.reg[reg_a] += self.reg[reg_b]
        #elif op == "SUB": etc
        else:
            raise Exception("Unsupported ALU operation")

    def trace(self, ir, prev_ir):
        """
        Handy function to print out the CPU state. You might want to call this
        from run() if you need help debugging.
        """

        print(f"PC={self.PC}")
        print(f"previous instruction: {prev_ir:b}")
        print(f"next instruction: {ir:b}  {self.ram[self.PC+1]:b}  {self.ram[self.PC+2]:b}")
        for i, r in enumerate(self.reg):
            print(f"R{i}: {r:b}")
        print()
        return

    def run(self):
        """Run the CPU."""
        t1 = datetime.datetime.now()
        prev_ir = 0
        while True:

            t2 = datetime.datetime.now()
            d = t2 - t1
            if d.seconds >= 1:
                t1 = t2
                self.reg[6] |= 1

            # check interrupt
            if self.reg[6] & 1 != 0:
                # write PC to stack
                self.reg[7] -= 1
                self.ram_write(self.PC, self.reg[7])

                # write FL to stack
                self.reg[7] -= 1
                self.ram_write(self.FL, self.reg[7])

                # write registers to stack
                for i in range(7):
                    self.reg[7] -= 1
                    self.ram_write(self.reg[i], self.reg[7])

                self.PC = self.ram_read(0xF8)

            try:
                # read next command
                ir = self.ram_read(self.PC)

                # look it up in branch table and execute
                self.branchtable[ir]()

                # besides for special cases which directly set PC, increment PC
                if ir not in {CALL, RET, JMP, JEQ, JNE}:
                    self.PC += 1

                # store previous command for debugging
                prev_ir = ir

            except Exception as e:
                print(f"Error encountered: {e}")
                self.trace(ir, prev_ir)
                return

    def ram_read(self, mar):
        return self.ram[mar]

    def ram_write(self, mdr, mar):
        self.ram[mar] = mdr
