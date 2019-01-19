@t1 = 55
@t2 = H'1B

main:
  LDA H'FC      ! load value to the accumulator register

sum:
  SUM 1         ! sum 1 in the accumulator register
  BTR B'0001    ! test if accumulator register is zero, if yes jump one instruction
  JUMP sum      ! if different of zero, jump to sum label
  CALL func1    ! call function 1
  SUM 2         ! sum 2 in the accumulator register
  CALL func3    ! call function 3
  JUMP main     ! jump to main label

func1:          ! function 1
  SUM 4         ! sum 4 in the accumulator register
  CALL func2    ! call function 2
  RETURN

func2:          ! function 2
  SUM 5         ! sum 5 in the accumulator register
  RETURN

func3:          ! function 3
  SUB 3         ! subtraction 3 in the accumulator register
  LDC           ! load the value of the accumulator register to register C
  RETURN