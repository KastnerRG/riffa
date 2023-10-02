# This sample application works with the chnl_tester module which receives data
# and sends data back.

import array
import riffa

# printing FPGA(s) info
l = riffa.fpga_list()
print(l.__str__())

if l.num_fpgas == 0:
	print('no PCIe FPGA detected')
else:
	amt = 200
	sent = 0
	recv = 0
	dataSend = array.array('I', range(amt+1))
	dataSend = dataSend[1:]
	dataRecv = array.array('I', [0]*amt)
	
	fd = riffa.fpga_open(l.id[0])
	sent = riffa.fpga_send(fd, 0, dataSend, amt, 0, True, 0)
	if (sent != 0):
		recv = riffa.fpga_recv(fd, 0, dataRecv, 0)

	riffa.fpga_close(fd)
	print(dataSend)
	print(dataRecv)

