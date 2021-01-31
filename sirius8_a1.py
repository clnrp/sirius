#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Author Cleoner S. Pietralonga
# e-mail: cleonerp@gmail.com
# Apache License

"""
Main elements
BUS is the main bus, where data is placed to be sent to other parts of the circuit.
PC is the program counter, it is an 8-bit counter.
MAR is the memory address register, and allows you to read or write to a specific address in the RAM memory.
RAM is RAM memory.
IR is the instruction register, where the RAM data is stored to be processed.
Acc is an accumulator register used in arithmetic operations
B is the auxiliary register
F is flag register
Alu is logical and arithmetic unit
The stack allows you to handle the functions

Instruction cycles
Cycle is incremented at the falling edge of the clock, and the control word values are updated, and the data on
the bus is written to the registers at the rising edge.

Cycle 0: On the falling edge, PcOut, MarIn are activated, still on the falling edge of the clock the value of PC
is placed on the bus, and on the rising edge the value is loaded into MAR.

Cycle 1: On the falling edge, RamOut, IrIn and PcInc are activated, the RAM value is placed on the bus and does
not depend on the clock, and on the rising edge the value is loaded to the IR and also the PC is increased.

Cycle 2-4: depends on the instruction recorded in IR
Cycle 5: __EndCycle, for 4-cycle instructions the fifth clears the cycle counter
"""

from logicbit.logic import *
from logicbit.clock import *
from logicbit.utils import *
from logicbit.keyboard import *

import sys

class CounterSen4b: # Counter of 4 bits
    def __init__(self):
        self.__Ff0 = Flipflop("D", "UP")
        self.__Ff1 = Flipflop("D", "UP")
        self.__Ff2 = Flipflop("D", "UP")
        self.__Ff3 = Flipflop("D", "UP")

    def Act(self, Input, En, Sen, Load, Reset, Clk): # Sen = 1 increase and Sen = 0 decrease
        in0,in1,in2,in3 = Input
        q0 = self.__Ff0.GetQ()
        q1 = self.__Ff1.GetQ()
        q2 = self.__Ff2.GetQ()
        q3 = self.__Ff3.GetQ()

        s0 = Load.Not() + Reset      # s0.Not()=1 -> Load=1 and Reset=0
        s1 = s0.Not() + Reset        # s1.Not()=1 -> s1=0 and Reset=0

        Q0 = s0.Not()*in0 + s1.Not()*(q0.Not())
        Q1 = s0.Not()*in1 + s1.Not()*(Sen*(q1.Not()*q0 + q1*q0.Not()) + Sen.Not()*(q1.Not()*q0.Not() + q1*q0))
        Q2 = s0.Not()*in2 + s1.Not()*(Sen*(q2.Not()*q1*q0 + q2*q1.Not() + q2*q0.Not()) + Sen.Not()*(q2.Not()*q1.Not()*q0.Not() + q2*q1 + q2*q0))
        Q3 = s0.Not()*in3 + s1.Not()*(Sen*(q3.Not()*q2*q1*q0 + q3*q2.Not() + q3*q1.Not() + q3*q0.Not()) + Sen.Not()*(q3.Not()*q2.Not()*q1.Not()*q0.Not() + q3*q2 + q3*q1 + q3*q0))

        q0 = self.__Ff0.Operate(En*Q0+En.Not()*self.__Ff0.GetQ(), LogicBit(0), Clk)
        q1 = self.__Ff1.Operate(En*Q1+En.Not()*self.__Ff1.GetQ(), LogicBit(0), Clk)
        q2 = self.__Ff2.Operate(En*Q2+En.Not()*self.__Ff2.GetQ(), LogicBit(0), Clk)
        q3 = self.__Ff3.Operate(En*Q3+En.Not()*self.__Ff3.GetQ(), LogicBit(0), Clk)
        return [q0,q1,q2,q3]

    def Operate(self, Clk):
        In = [LogicBit(0) for bit in range(4)]
        En = LogicBit(1)
        Load = LogicBit(0)
        Reset = LogicBit(0)
        return self.Act(En, In, Load, Reset, Clk)

    def Read(self):
        Out = [0]*4
        Out[0] = self.__Ff0.GetQ()
        Out[1] = self.__Ff1.GetQ()
        Out[2] = self.__Ff2.GetQ()
        Out[3] = self.__Ff3.GetQ()
        return Out

class Reg8bTris12b:
    def __init__(self):
        self.__reg = Register8b()
        self.__tristate = TristateBuffer()

    def Act(self, Bus, EIn, EOut, Reset, Clk):
        A = self.__reg.Act(Bus[0:8], EIn, Reset, Clk)
        Dir = LogicBit(1)
        A = A + [LogicBit(0), LogicBit(0), LogicBit(0), LogicBit(0)]
        [A,B] = self.__tristate.Buffer(A, Bus, Dir, EOut) # Dir=1 and EOut=1 -> put A in B
        return B

    def Read(self):
        return self.__reg.Read()

class PC8bTris: # Program counter of 8 bits with tri-state
    def __init__(self):
        self.__pc8b = Counter8b()
        self.__tristate = TristateBuffer()

    def Act(self, Bus, EInc, EOut, Load, Reset, Clk):
        A = self.__pc8b.Act(Bus[0:8], EInc, Load, Reset, Clk)
        Dir = LogicBit(1)
        A = A + [LogicBit(0), LogicBit(0), LogicBit(0), LogicBit(0)]
        [A,B] = self.__tristate.Buffer(A, Bus, Dir, EOut) # Dir=1 and EOut=1 -> put A in B
        return B

    def Read(self):
        return self.__pc8b.Read()

class ALU8bTris12b:
    def __init__(self):
        self.__Alu = ALU8b()
        self.__tristate = TristateBuffer()

    def Act(self, Bus, A, B, Word, F, SumSub, Alu0, Alu1, AluOut, Clk):
        A,CarryBorrow = self.__Alu.Act(A, B, SumSub, Alu0, Alu1)
        Dir = LogicBit(1)
        A = A + [LogicBit(0), LogicBit(0), LogicBit(0), LogicBit(0)]

        # Update F register
        Zero = A[7].Not()*A[6].Not()*A[5].Not()*A[4].Not()*A[3].Not()*A[2].Not()*A[1].Not()*A[0].Not()
        Mask = Utils.VecBinToPyList([0, 0, 0, 0, 0, 0, 1, 1])
        Flags = [Zero,CarryBorrow]+Utils.VecBinToPyList([0, 0, 0, 0, 0, 0])
        F.Act(Flags, Mask, Word.FIn, LogicBit(0), Clk)

        [A,B] = self.__tristate.Buffer(A, Bus, Dir, AluOut) # Dir=1 and EOut=1 -> put A in B
        return B

class StackTris: # Stack with 16 bytes
    def __init__(self):
        self.__stack = Ram(4,8) # address 4 bits, data 8 bits
        self.__pc4b = CounterSen4b()
        self.__tristate = TristateBuffer()

    def Act(self, Bus, Cnt, Sen, Write, EnOut, Reset, Clk):
        Addr = self.__pc4b.Read()
        Fpush = Sen*(Addr[3]*Addr[2]*Addr[1]*Addr[0]).Not() # increase if Addr is not 1111
        Fpop = Sen.Not()*(Addr[3]+Addr[2]+Addr[1]+Addr[0])  # decrease if Addr is not 0000
        Addr = self.__pc4b.Act(Bus[0:4], Cnt*(Fpush + Fpop), Sen, LogicBit(0), Reset, Clk)
        Dir = LogicBit(1)
        Stack = self.__stack.Act(Addr, Bus, Write, Reset, Clk) # Write = 1 write memory
        Stack = Stack+[LogicBit(0),LogicBit(0),LogicBit(0),LogicBit(0)]
        [A,B] = self.__tristate.Buffer(Stack, Bus, Dir, EnOut) # Dir=1 and EOut=1 -> put A in B
        return B

class MarRegister: # Memory address register
    def __init__(self):
        self.__reg = Register8b() # 8-bits register

    def Act(self, Bus, MarIn, Reset, Clk):
        value = self.__reg.Act(Bus, MarIn, Reset, Clk)
        return value

    def Read(self):
        return self.__reg.Read()

class IR: # instruction register
    def __init__(self):
        self.__reg = Register(12) # 12 bits register
        self.__tristate = TristateBuffer()

    def Act(self, Bus, IRIn, IROut, Reset, Clk):
        Out = self.__reg.Act(Bus, IRIn, Reset, Clk)
        Dir = LogicBit(1)
        LSB = Out[0:8]  # 8 bits
        Code = Out
        A = LSB + [LogicBit(0), LogicBit(0), LogicBit(0), LogicBit(0)]
        [A,B] = self.__tristate.Buffer(A, Bus, Dir, IROut) # Dir=1 and IROut=1 -> put A in B
        return B,Code # B=Bus, Code go to instruction decoder

    def Read(self):
        return self.__reg.Read()

class InstDecoder: # instruction decoder
    def __init__(self):
        self.__CycleDec = BinaryDecoder()
        self.__InstrDec = BinaryDecoder()
        self.__cnt = Counter4b()
        self.__EndCycle = LogicBit(1)

    def Act(self, Word, Code, F, Clk):
        nClk = Clk.Not()
        Flag = F.Read()
        OpCode = Code[8:12]
        Input = [LogicBit(0),LogicBit(0),LogicBit(0),LogicBit(0)]
        CntBits = self.__cnt.Act(Input, LogicBit(1), LogicBit(0), self.__EndCycle, nClk) # EndCycle reset Counter
        Cycle = self.__CycleDec.Act(CntBits)
        [NOP,JUMP,LDA,SUM,SUB,AND,OR,XOR,LDC,BTR,CALL,RET] = self.__InstrDec.Act(OpCode)[:12]
        self.__EndCycle = Cycle[5] + Cycle[3]*(JUMP + LDA + LDC + BTR)   # Reset counter
        Word.PcOut = Cycle[0]+Cycle[2]*CALL
        Word.IrOut = Cycle[2]*(JUMP + LDA + SUM + SUB) + Cycle[4]*CALL
        Word.MarIn = Cycle[0]
        Word.Jump = Cycle[2]*JUMP + Cycle[3]*RET + Cycle[4]*CALL
        Word.RamOut = Cycle[1]
        Word.IrIn = Cycle[1]
        Word.PcInc = Cycle[1] + Cycle[2]*JUMP + Cycle[3]*RET + Cycle[4]*CALL + \
                     Cycle[2]*BTR*(Code[0]*Flag[0]+Code[1]*Flag[1]+Code[2]*Flag[2]+Code[3]*Flag[3]+Code[4]*Flag[4]+Code[5]*Flag[5]+Code[6]*Flag[6]+Code[7]*Flag[7])
        Word.AccIn = Cycle[2]*LDA + Cycle[4]*(SUM + SUB)
        Word.AccOut = Cycle[2]*LDC
        Word.BIn = Cycle[2]*(SUM + SUB)
        Word.CIn = Cycle[2]*LDC
        Word.FIn = Cycle[3]*(SUM + SUB + AND + OR + XOR)
        Word.AluOut = Cycle[4]*(SUM + SUB)
        Word.SumSub = (Cycle[3]+Cycle[4])*(SUM.Not() + SUB)
        Word.Alu0 = (Cycle[3]+Cycle[4])*(AND + XOR)
        Word.Alu1 = (Cycle[3]+Cycle[4])*(OR + XOR)
        Word.StkCnt = Cycle[2]*RET + Cycle[3]*CALL
        Word.StkWr = Cycle[2]*CALL
        Word.StkOut = Cycle[3]*RET
        Word.StkSen = Cycle[3]*CALL


        """"
        Cycle 0 -> PcOut e MarIn
        Cycle 1 -> RamOut, IrIn e PcInc.
        The control bits will be triggered on the falling edge of the clock.
        NOP  0000
        JUMP 0001, 2 -> IrOut, PcInc, Jump;
        LDA  0010, 2 -> IrOut, AccIn;
        SUM  0011, 2 -> IrOut, BIn;         3 -> SumSub=0, FIn;         4 -> SumSub=0, AluOut, AccIn
        SUB  0100, 2 -> IrOut, BIn;         3 -> SumSub=1, FIn;         4 -> SumSub=1, AluOut, AccIn;
        AND  0101, 2 -> IrOut, BIn;         3 -> Alu0=1, Alu1=0, FIn;   4 -> Alu0=1, Alu1=0, AluOut, AccIn
        OR   0110, 2 -> IrOut, BIn;         3 -> Alu0=0, Alu1=1, FIn;   4 -> Alu0=0, Alu1=1, AluOut, AccIn
        XOR  0111, 2 -> IrOut, BIn;         3 -> Alu0=1, Alu1=1, FIn;   4 -> Alu0=1, Alu1=1, AluOut, AccIn
        LDC  1000, 2 -> AccOut, CIn;
        BTR  1001, 2 -> PcInc # Opcode = 4bits, Register=4bits, Bit=3bits, SetClear=1  Max 16 register
        CALL 1010, 2 -> PcOut, StkWr;       3 -> StkCnt, StkSen;        4 -> IrOut, PcInc, Jump
        RET  1011, 2 -> StkCnt, 'StkSen;    3 -> StkOut, PcInc, Jump
        """
        #Printer(Cycle,"Cycles")
        return Word

    def getCount(self):
        return [str(value) for value in self.__cnt.Read()]

class Word:
    def __init__(self):
        self.Reset = LogicBit(0)    # Reset all
        self.PcInc = LogicBit(0)    # Enable increment of the Counter
        self.PcOut = LogicBit(0)    # Put PC on Bus
        self.Jump = LogicBit(0)     # Load Bus into PC
        self.AccIn = LogicBit(0)    # Load Bus into accumulator register
        self.AccOut = LogicBit(0)   # Put Acc into Bus
        self.BIn = LogicBit(0)      # Load Bus into B register
        self.CIn = LogicBit(0)      # Load Bus into C register
        self.FIn = LogicBit(0)      # Change F register
        self.FOut = LogicBit(0)     # Put F register into Bus
        self.SumSub = LogicBit(0)   # Enable sum operation in 0, and subtraction in 1
        self.Alu0 = LogicBit(0)     # Enable in AND and XOR operation
        self.Alu1 = LogicBit(0)     # Enable in OR and XOR operation
        self.AluOut = LogicBit(0)   # Put ALU data into Bus
        self.We = LogicBit(0)       # Write/Read Ram
        self.MarIn = LogicBit(0)    # Load Bus into MAR register
        self.RamOut = LogicBit(0)   # Put Ram data into Bus
        self.IrIn = LogicBit(0)     # Load Bus into IR register
        self.IrOut = LogicBit(0)    # Put IR register into Bus
        self.StkCnt = LogicBit(0)   # Enable increase or decrease Stack Counter
        self.StkWr = LogicBit(0)    # Write Pc value in the stack
        self.StkOut = LogicBit(0)   # Put top of the stack into Bus
        self.StkSen = LogicBit(0)   # select increase or decrease, when Sen = 1 is increase

    def print(self):
        values = vars(self)
        print({key: str(values[key]) for key in values.keys()})

def flogic(clock):
    args = sys.argv[1:]

    filename = "test2.o" #args[0]
    f = open(filename, "r")
    Code = f.read()
    f.close()

    Bus = [LogicBit(0) for bit in range(12)]  # initializes 12 bits of the Bus with 0
    Pc = PC8bTris()           # program counter of 8 bits with tri-state
    Mar = MarRegister()       # memory address register
    Ram = RamTris(8,12)       # RAM memory, 8 bits address and 12 bits of data
    Stack = StackTris()       # Stack with 16 bytes
    A = Reg8bTris12b()        # Accumulator register
    B = Register8b()          # B register
    F = Register8b_Sb()       # Flag register
    C = Register8b()          # C register
    Alu = ALU8bTris12b()      # 8-bit arithmetic and logic unit
    Ir = IR()                 # instruction register
    InstDec = InstDecoder()   # instruction decoder

    w = Word()  # Control word

    program = Utils.TextToBinArray(Code)
    # write program in ram
    for value, addr in zip(program, range(len(program))):
        addr = Utils.BinValueToPyList(addr,8)
        for Clk in [LogicBit(0),LogicBit(1)]:
            Ram.Act(value, addr, LogicBit(1), LogicBit(0), LogicBit(0), Clk)

    clock.Next()  # start with clock = 0
    clock.Run()
    while clock.GetState():
        Clk = clock.GetClock()

        Bus = Pc.Act(Bus, w.PcInc, w.PcOut, w.Jump, w.Reset, Clk)                    # Program counter, 8 bits
        Mar.Act(Bus[0:8], w.MarIn, w.Reset, Clk)                                     # Memory address 8 bits register
        Bus = Ram.Act(Bus, Mar.Read(), w.We, w.RamOut, LogicBit(0), Clk)             # RAM memory, 8 bits address and 12 bits of data
        Bus = Stack.Act(Bus, w.StkCnt, w.StkSen, w.StkWr, w.StkOut, w.Reset, Clk)    # Stack with 16 bytes
        Bus = A.Act(Bus, w.AccIn, w.AccOut, w.Reset, Clk)
        B.Act(Bus[0:8], w.BIn, w.Reset, Clk)
        C.Act(Bus[0:8], w.CIn, w.Reset, Clk)
        Bus = Alu.Act(Bus, A.Read(), B.Read(), w, F, w.SumSub, w.Alu0, w.Alu1, w.AluOut, Clk)
        Bus, Code = Ir.Act(Bus, w.IrIn, w.IrOut, w.Reset, Clk)                       # Instruction register, 12 bits
        InstDec.Act(w, Code, F, Clk)

        print("Clock:" + str(Clk) + ", cnt=" + str(InstDec.getCount()))
        w.print()  # print control word
        Printer(A.Read(), "A")
        Printer(B.Read(), "B")
        Printer(C.Read(), "C")
        Printer(F.Read(), "F")
        Printer(Pc.Read(), "Pc")
        Printer(Bus, "Bus")

clk = Clock(flogic,10,2)  # two samples per state
clk.start()  # initialize clock
#key = Keyboard(clk)
#key.start()  # initialize keyboard