package mz3.test;

import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class IHMCcopy extends Slave
{
    private final static int vendor = 0x00000021;
    private final static int productCode = 0x07500354;

    private class Outputs extends RxPDO
    {
        public Outputs()
        {
            super(0x1601);
        }

        private final Bool[] out = array(new Bool[8]);
        private final Unsigned8 gap = new Unsigned8();
    }

    private final Outputs outputs = new Outputs();

    private class CyclicReceiveData extends RxPDO
    {
        public CyclicReceiveData()
        {
            super(0x16ff);
        }
        private final Bool[] filler = array(new Bool[16]);
        private final Unsigned16 Diagnostics = new Unsigned16();
    }

    private final CyclicReceiveData cyclicReceiveData = new CyclicReceiveData();

    private class CyclicTrasmitData extends TxPDO
    {
        public CyclicTrasmitData()
        {
            super(0x1aff);
        }
        private final Bool[] filler = array(new Bool[16]);
        private final Unsigned16 Diagnostics = new Unsigned16();
    }

    private final CyclicTrasmitData cyclicTrasmitData = new CyclicTrasmitData();

    private class Inputs extends TxPDO
    {
        public Inputs()
        {
            super(0x1a00);
        }

        private final Bool[] in = array(new Bool[8]);
        private final Unsigned8 gap = new Unsigned8();
        //private final Float32 busVoltage = new Float32();
    }

    private final Inputs inputs = new Inputs();

    public IHMCcopy(int aliasAddress, int position)
    {
        super(vendor, productCode, aliasAddress, position);

        registerSyncManager(new SyncManager(2, false));
        registerSyncManager(new SyncManager(3, false));
        sm(2).registerPDO(cyclicReceiveData);
        sm(2).registerPDO(outputs);
        sm(3).registerPDO(cyclicTrasmitData);
        sm(3).registerPDO(inputs);
    }


    public boolean getDigitalInput(int input)
    {
        return inputs.in[input].get();
    }

    public void setOutput(int output, boolean value)
    {
        outputs.out[output].set(value);
    }

    /*
    public float getBusVoltage()
    {
        return inputs.busVoltage.get();
    }


    public long getIMU0StatusWord()
    {
        return imuData.status.get();
    }


    public int getIMU0PacketCounter()
    {
        return imuData.packetCounter.get();
    }


    public long getIMU0SampleTime()
    {
        return imuData.sampleTime.get();
    }


    public double getIMU0qx()
    {
        return imuData.qx.get();
    }


    public double getIMU0qy()
    {
        return imuData.qy.get();
    }


    public double getIMU0qz()
    {
        return imuData.qz.get();
    }


    public double getIMU0qs()
    {
        return imuData.qs.get();
    }


    public double getIMU0xdd()
    {
        return imuData.xdd.get();
    }


    public double getIMU0ydd()
    {
        return imuData.ydd.get();
    }


    public double getIMU0zdd()
    {
        return imuData.zdd.get();
    }


    public double getIMU0wx()
    {
        return imuData.wx.get();
    }


    public double getIMU0wy()
    {
        return imuData.wy.get();
    }


    public double getIMU0wz()
    {
        return imuData.wz.get();
    }
    */

}
