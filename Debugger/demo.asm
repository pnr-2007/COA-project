; RISC201 debugger demonstration
; Run with: load demo.asm, then run
; Expected final values: R1=17, R4=24, R5=14
ADD R1, R2, R3
ADD R4, R1, R3
SUB R5, R4, R2
HALT
