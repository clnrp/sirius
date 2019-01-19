@t1 = 55
@t2 = H'1B

init:
  LDA H'FC      ! load value to the accumulator register

sum:
  SUM 1         ! sum 1 in the accumulator register
  BTR B'0001    ! test if accumulator register is zero, if yes jump one instruction
  JUMP sum      ! if different of zero, jump to sum label

end:
  SUM 3         ! sum 3 in the accumulator register
  LDC           ! load the value of the accumulator register to register C
  JUMP init     ! jump to init label
