package mz3.examples;

import us.ihmc.etherCAT.master.EtherCATRealtimeThread;
import mz3.test.IHMCcopy;
import mz3.slaves.yaskawa.SGD7S;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;

import java.io.IOException;

public class RespondToInput extends EtherCATRealtimeThread {

    private final SGD7S xAxis = new SGD7S(1, 0);
    //private final SGD7S yAxis = new SGD7S(0, 1);
    private int counter = 0;
    private int xTargetPos = 0;
    private int yTargetPos = 0;
    private int positionIncrement = 350;
    private int startupTorque = 10;
    private int startupMaxTorque = 100;
    private int targetTorque = 100;
    private int maxTorque = 1000;
    private int currentOutput = 0;
    private boolean posReached = true;
    private SGD7S.ControlWord lastKnownControlWord = null;
    public RespondToInput() throws IOException
    {
        super("enp0s25", new PriorityParameters(PriorityParameters.getMaximumPriority()), new MonotonicTime(0, 1000000), true, 100000);
        registerSlave(xAxis);
        //registerSlave(yAxis);
        registerSlave(ioModule);
        enableTrace();
        // Test SDO reading
    }
    private final IHMCcopy ioModule = new IHMCcopy(1, 1);
    //Wago Coupler combined unit

    public static void main(String[] args) throws IOException
    {
        mz3.examples.RespondToInput respondToInput = new mz3.examples.RespondToInput();
        respondToInput.start();
        respondToInput.join();
    }

    @Override
    protected void deadlineMissed()
    {

    }

    @Override
    protected void doControl() {

        xAxis.doStateControl();
        //yAxis.doStateControl();
        if (counter == 100) {
            xAxis.enableDrive();
            //yAxis.enableDrive();
        }
        if (counter == 200) {
            xTargetPos = xAxis.getActualPosition();
            xAxis.setMaxTorque(startupMaxTorque);
            xAxis.setTargetTorque((short)startupTorque);
            //yTargetPos = yAxis.getActualPosition();
            //yAxis.setMaxTorque(startupMaxTorque);
            //yAxis.setTargetTorque((short)startupTorque);
        }
        if (counter == 300) {
            xAxis.setModeOfOperation(SGD7S.YaskawaModeOfOperation.CYCLIC_SYNCHRONOUS_POSITION);
            //yAxis.setModeOfOperation(SGD7S.YaskawaModeOfOperation.CYCLIC_SYNCHRONOUS_POSITION);
        }
        if (counter == 400) {
            System.out.println("at counter : " + counter + " current pos : " + xAxis.getActualPosition() + " sw : " + xAxis.getStatusVar());
            xAxis.setTargetPosition(xTargetPos);
            //yAxis.setTargetPosition(yTargetPos);
        }
        if (counter == 500) {
            xTargetPos = xAxis.getActualPosition();
            xAxis.setMaxTorque(maxTorque);
            xAxis.setTargetTorque((short)targetTorque);
            //yTargetPos = yAxis.getActualPosition();
            //yAxis.setMaxTorque(maxTorque);
            //yAxis.setTargetTorque((short)targetTorque);
        }
        if (counter == 600) {
            System.out.println("at counter : " + counter + " current pos : " + xAxis.getActualPosition() + " sw : " + xAxis.getStatusVar());
            xAxis.setTargetPosition(xTargetPos);
            //yAxis.setTargetPosition(yTargetPos);
        }
        if (counter > 600) {
            if (ioModule.getDigitalInput(0)) {
                xTargetPos = xTargetPos + positionIncrement;
                xAxis.setTargetPosition(xTargetPos);
            }
            else if (ioModule.getDigitalInput(2)) {
                xTargetPos = xTargetPos - positionIncrement;
                xAxis.setTargetPosition(xTargetPos);
            }
            if (ioModule.getDigitalInput(4)) {
                yTargetPos = yTargetPos + positionIncrement;
                //yAxis.setTargetPosition(yTargetPos);
            }
            else if (ioModule.getDigitalInput(6)) {
                yTargetPos = yTargetPos - positionIncrement;
                //yAxis.setTargetPosition(yTargetPos);
            }
            if(counter % 50 == 0){
                ioModule.setOutput(currentOutput++,false);
            if (currentOutput == 8) {currentOutput=0;}
                ioModule.setOutput(currentOutput,true);
            }
        }
        counter++;
    }

    @Override
    protected void workingCounterMismatch(int expected, int actual)
    {
        System.out.println("workin counter mismatch");

    }

    @Override
    protected void datagramLost()
    {
        // TODO Auto-generated method stub

    }

    @Override
    protected void doReporting()
    {
        // TODO Auto-generated method stub

    }

}

