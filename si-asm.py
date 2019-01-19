#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Author Cleoner S. Pietralonga
# e-mail: cleonerp@gmail.com
# Apache License

import sys, getopt

opcode_map = {
    "NOP":[0,"0000"],
    "JUMP":[2,"0001"],
    "LDA":[1,"0010"],
    "SUM":[1,"0011"],
    "SUB":[1,"0100"],
    "AND":[1,"0101"],
    "OR":[1,"0110"],
    "XOR":[1,"0111"],
    "LDC":[0,"1000"],
    "BTR":[1,"1001"],
    "CALL":[2,"1010"],
    "RETURN":[0,"1011"]
}

opcode_list = []     # list of opcodes
label_map = {}       # maps of label position
instruction_list = []     # list of binary code

def getByte(number):
    if("H'" in number): # hex number
        number = number.replace("H'","")
        number = int(number, 16)
        value  = bin(number)[2:].zfill(8)
    elif("B'" in number): # binary number
        number = number.replace("B'", "")
        number = int(number, 2)
        value = bin(number)[2:].zfill(8)
    else: # decimal number
        value = bin(int(number))[2:].zfill(8)
    return value

def count(text, token): # count token
    cnt = 0
    for c in text:
        if(c == token):
            cnt+=1
    return cnt

def remove_comments(text):
    atext = text
    cnt_comment = count(atext, '!')
    for i in range(cnt_comment):
        i1 = atext.find('!')
        i2 = atext[i1:].find('\n')
        if (i2 == -1):
            i2 = len(atext)
        i2 += i1
        result = atext[:i1] + atext[i2:]
        atext = result
    return atext

if __name__ == "__main__":
    args = sys.argv[1:]

    filename = args[0]
    f = open(filename, "r")
    #f = open("test1.asm", "r")
    text = f.read()
    f.close()
    text = remove_comments(text)

    list = text.split("\n")
    pc = 0   # program counter value
    for line in list:
        ls = line.strip().split(" ")
        if (len(ls) == 1 and ":" in ls[0]): # add the position label in map
            label_map.update({ls[0].replace(":",""): pc})
        elif (ls[0] in opcode_map):         # add opcode in list
            opcode_list.append([ls, pc])
            pc += 1

    file_data = ""
    for item in opcode_list:
        type =  opcode_map[item[0][0]][0]  # type of instruction
        if(type == 0):
            opcode = opcode_map[item[0][0]][1] # get opcode of the instruction
            instruction = opcode + getByte("0") # assemble the instruction
        elif(type == 1):
            opcode = opcode_map[item[0][0]][1]
            instruction = opcode + getByte(item[0][1]) # assemble the instruction
        elif(type == 2):                            # JUMP or CALL
            opcode = opcode_map[item[0][0]][1]
            instruction = opcode + getByte(str(label_map[item[0][1]])) # assemble the instruction
        #print(item, instruction)
        #instruction_list.append(instruction)
        file_data += instruction+"\n"

    print(file_data)
    f = open(filename.split(".")[0]+".o", "w")
    f.write(file_data)
    f.close()