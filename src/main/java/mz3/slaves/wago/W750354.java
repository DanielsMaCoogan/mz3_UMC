package mz3.slaves.wago;
import us.ihmc.etherCAT.master.Slave;

public class W750354 extends Slave
{
    static final int vendorID = 0x00000021;
    static final int productCode = 0x07500354;

    public W750354(int aliasAddress, int configAddress)
    {
        super(vendorID, productCode, aliasAddress, configAddress);
    }


}
