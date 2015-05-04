import edu.ucsd.cs.riffa.*;

public class SampleApp {

	public static void main(String[] args) {
		try {
			// Print out the installed RIFFA FPGAs
			FpgaInfo info = Fpga.list();
			System.out.println(info);

			// Exercise the chnl_tester module.
			int amt = 200;
			java.nio.ByteBuffer byteBufRecv = java.nio.ByteBuffer.allocateDirect(amt);
			java.nio.IntBuffer intBufRecv = byteBufRecv.asIntBuffer();
			java.nio.ByteBuffer byteBufSend = java.nio.ByteBuffer.allocateDirect(amt);
			java.nio.IntBuffer intBufSend = byteBufSend.asIntBuffer();

			// Initialize the byteBufSendfers for send and receive. Note that 
			// Java stores numbers in big endian format. The chnl_tester module
			// expects them in little endian. So we need to byte swap.
			byte tmp0, tmp1;
			for (int i=0; i < amt/4; i++) {
				intBufRecv.put(i, 0);
				intBufSend.put(i, i+1);
				tmp0 = byteBufSend.get((i*4)+0);
				tmp1 = byteBufSend.get((i*4)+1);
				byteBufSend.put((i*4)+0, byteBufSend.get((i*4)+3));
				byteBufSend.put((i*4)+1, byteBufSend.get((i*4)+2));
				byteBufSend.put((i*4)+2, tmp1);
				byteBufSend.put((i*4)+3, tmp0);
				byteBufSend.put(i*4, (byte)(i+1));
				System.out.printf("%d - %02x%02x%02x%02x\n", i+1,
					byteBufSend.get((i*4)+3), byteBufSend.get((i*4)+2), byteBufSend.get((i*4)+1), byteBufSend.get(i*4));
			}

			Fpga fpga = Fpga.open(0);
			int s = fpga.send(0, byteBufSend, amt/4, 0, true, 0);
			int r = fpga.recv(0, byteBufRecv, 0);
			fpga.close();

			System.out.println(s + " words sent");
			System.out.println(r + " words recvd");

			// If using the chnl_tester module, the values returned will be in
			// little endian format. Thus they'll be interpreted incorrectly in
			// Java (unless byte swapped of course).
			for (int i=0; i < amt/4; i++){
				System.out.printf("%d) %d - %02x%02x%02x%02x\n", i+1, intBufRecv.get(i),
					byteBufRecv.get((i*4)+3), byteBufRecv.get((i*4)+2), byteBufRecv.get((i*4)+1), byteBufRecv.get(i*4));
			}
		}
		catch (Exception ex) {
			ex.printStackTrace();
		}
	}

}
