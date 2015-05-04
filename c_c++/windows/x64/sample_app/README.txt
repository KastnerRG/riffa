The testutil example application works with the Verilog chnl_tester module
which receives data and then sends data back to the workstation. 

To compile the example application:

gcc.exe -o testutil.exe testutil.c -L. -lriffa

The riffa.lib import library must be in the directory specified by the -L 
parameter (in this example, the current directory). The riffa.h header must be
in the same directory as testutil.c.
