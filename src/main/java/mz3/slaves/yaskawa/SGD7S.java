package mz3.slaves.yaskawa;

import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

import java.io.IOException;

public class SGD7S extends Slave {

   static final int vendorID = 0x00000539;
   static final int productCode = 0x02200301;
   static final int assignActivate = 0x0300;
   private int SGD7Scounter = 0;

   // single bit parameters as part of controlword in addition to those already provided
   private static final int MASK_BIT0 = 0x1; // bit 0
   private static final int MASK_BIT1 = 0x2; // bit 1
   private static final int MASK_BIT2 = 0x4; // bit 2
   private static final int MASK_BIT3 = 0x8; // bit 3
   private static final int MASK_BIT4 = 0x10; // bit 4
   private static final int MASK_BIT5 = 0x20; // bit 5
   private static final int MASK_BIT6 = 0x40; // bit 6
   private static final int MASK_BIT7 = 0x80; // bit 7
   private static final int MASK_BIT8 = 0x100; // bit 8
   private static final int MASK_BIT9 = 0x200; // bit 9
   private static final int MASK_BIT10 = 0x400; // bit 10
   private static final int MASK_BIT11 = 0x800; // bit 11
   private static final int MASK_BIT12 = 0x1000; // bit 12
   private static final int MASK_BIT13 = 0x2000; // bit 13
   private static final int MASK_BIT14 = 0x4000; // bit 14
   private static final int MASK_BIT15 = 0x8000; // bit 15

   private static final int noAction = Integer.parseInt("000", 2);
   private static final int shutdown = Integer.parseInt("00000110", 2);
   private static final int switchon = Integer.parseInt("0111", 2);
   private static final int disableVoltage = Integer.parseInt("0101", 2);
   private static final int enableOperation = Integer.parseInt("001111", 2);
   private static final int quickStopActive = Integer.parseInt("1011", 2);
   private static final int disableOp = Integer.parseInt("0111", 2);
   private static final int faultReset = Integer.parseInt("10000000", 2);
   private static final int STATUS_NOTREADYTOSWITCHON = 0x0;
   private static final int MASK_NOTREADYTOSWITCHON = 0x4F;
   private static final int STATUS_SWITCHONDISABLED = 0x40;
   private static final int MASK_SWITCHONDISABLED = 0x4F;
   private static final int STATUS_READYTOSWITCHON = 0x21;
   private static final int MASK_READYTOSWITCHON = 0x6F;
   private static final int STATUS_SWITHON = 0x233;
   private static final int MASK_SWITCHON = 0x27f;
   private static final int STATUS_OPERATIONENABLED = 0x237;
   private static final int MASK_OPERATIONENABLED = 0x27f;
   private static final int STATUS_QUICKSTOPACTIVE = 0x7;
   private static final int MASK_QUICKSTOPACTIVE = 0x6F;
   private static final int STATUS_FAULT = 0x8;
   private static final int MASK_FAULT = 0x8;

   private int previousControlWord = 0x0;
   private int previousStatusWord = 0x0;

   private boolean requestToMoveToNewSetPositionBoolean = false;
   private boolean waitingOnSetpointAcknowledgedBoolean = false;
   private boolean setpointAcknowledgedBoolean = false;
   private boolean readyForNewSetPointBoolean = true;
   private boolean waitingToReachNewPositionBoolean = false;
   private boolean PositionReachedBoolean = true;
   private boolean onRouteBoolean = false;
   private ControlWord command = null;
   private boolean enableDrive = false;

   //Status Register Event Locations
   private final long UNDER_VOLTAGE = 16;
   private final long OVER_VOLTAGE = 5;
   private final long STO_DISABLED = 7;
   private final long CURRENT_SHORT = 11;
   private final long OVER_TEMPERATURE = 13;
   private final long MOTOR_ENABLED = 3;
   private final long MOTOR_FAULT = 64;
   private final long CURRENT_LIMITED = 8192;

   private RPDO_1600 rpdo_1600 ;
   private RPDO_1601 rpdo_1601 ;
   private TPDO_1a00 tpdo_1a00 ;
   private TPDO_1a01 tpdo_1a01 ;

   private StatusWord state ;

   private boolean errorCodeRead = false;
   //private final ReadSDO yaskawaErrorCodeSDO;
   private int yaskawaErrorCode = 0;

   private boolean readYaskawaErrorCode = true;

   public enum ControlWord {
      NOACTION, SHUTDOWN, SWITCHON, DISABLEVOLTAGE, ENABLEOPERATION, QUICKSTOPACTIVE, DISABLEOP, FAULTRESET
   }

   public enum StatusWord {
      NOTREADYTOSWITCHON, SWITCHONDISABLED, READYTOSWITCHON, SWITCHEDON, OPERATIONENABLE, QUICKSTOPACTIVE, FAULT, RUNNING
   }

   public enum YaskawaModeOfOperation {
      NO_MODE(0),
      PROFILE_POSITION_MODE(1),
      SPOSING(2), // TODO testing
      PROFILED_VELOCITY_MODE(3),
      TORQUE_PROFILED_MODE(4),
      HOMING_MODE(6),
      INTERPOLATED_POSITION_MODE(7),
      CYCLIC_SYNCHRONOUS_POSITION(8), // DSP402
      CYCLIC_SYNCHRONOUS_VELOCITY(9), // DSP402
      CYCLIC_SYNCHRONOUS_TORQUE(10); // DSP402

      private final byte modeByte;

      private YaskawaModeOfOperation(int mode) {
         this.modeByte = (byte) mode;
      }

      public byte getMode() {
         return modeByte;
      }
   }

   private long maxDriveTorque;
   private long profileJerk;

   private class RPDO_1600 extends RxPDO {
      public RPDO_1600() {
         super(0x1600);
      }

      Unsigned16 controlWord = new Unsigned16();
      Signed32 targetPosition = new Signed32();
      Signed32 targetVelocity = new Signed32();
      Signed16 targetTorque = new Signed16();
      Unsigned16 maxTorque = new Unsigned16();
      Signed8 modeOfOperation = new Signed8();
      Unsigned8 padding = new Unsigned8();
      Unsigned16 touchProbeFunction = new Unsigned16();
   }


   private class RPDO_1601 extends RxPDO {
      public RPDO_1601() {
         super(0x1601);
      }

      Unsigned16 controlWord = new Unsigned16();
      Signed32 targetPosition = new Signed32();
   }

   private class RPDO_1602 extends RxPDO {
      public RPDO_1602() {
         super(0x1602);
      }

      Unsigned16 controlWord = new Unsigned16();
      Signed32 targetVelocity = new Signed32();
   }

   private class RPDO_1603 extends RxPDO {
      public RPDO_1603() {
         super(0x1603);
      }

      Unsigned16 controlWord = new Unsigned16();
      Signed16 targetTorque = new Signed16();
   }

   private class TPDO_1a00 extends TxPDO {
      public TPDO_1a00() {
         super(0x1a00);
      }

      Unsigned16 statusWord = new Unsigned16();
      Signed32 positionActualValue = new Signed32();
      Signed16 torqueActualValue = new Signed16();
      Signed32 followingErrorActualValue = new Signed32();
      Signed8 modesOfOperationDisplay = new Signed8();
      Unsigned8 padding = new Unsigned8();
      Unsigned16 touchProbeStatus = new Unsigned16();
      Signed32 touchProbeValue = new Signed32();
   }

   private class TPDO_1a01 extends TxPDO {
      public TPDO_1a01() {
         super(0x1a01);
      }

      Unsigned16 statusWord = new Unsigned16();
      Signed32 positionActualValue = new Signed32();

   }

   private class TPDO_1a02 extends TxPDO {
      public TPDO_1a02() {
         super(0x1a02);
      }

      Unsigned16 statusWord = new Unsigned16();
      Signed32 positionActualValue = new Signed32();
   }

   private class TPDO_1a03 extends TxPDO {
      public TPDO_1a03() {
         super(0x1a03);
      }

      Unsigned16 statusWord = new Unsigned16();
      Signed32 positionActualValue = new Signed32();
      Signed16 torqueActualValue = new Signed16();
   }

   //**********Private functions******************

   private void setControlWord(ControlWord control) {
      int cmd = 0;
      switch (control) {
         case NOACTION:
            cmd = noAction;
            break;
         case SHUTDOWN:
            cmd = shutdown;
            break;
         case SWITCHON:
            cmd = switchon;
            break;
         case DISABLEVOLTAGE:
            cmd = disableVoltage;
            break;
         case ENABLEOPERATION:
            cmd = enableOperation;
            if (requestToMoveToNewSetPositionBoolean) {
               cmd = cmd | MASK_BIT4;
               requestToMoveToNewSetPositionBoolean = false;
               waitingOnSetpointAcknowledgedBoolean = true;
            }
            break;
         case QUICKSTOPACTIVE:
            cmd = quickStopActive;
            break;
         case DISABLEOP:
            cmd = disableOp;
            break;
         case FAULTRESET:
            cmd = faultReset;
            break;
         default:
            throw new RuntimeException("Invalid command");
      }

      rpdo_1600.controlWord.set(cmd);
      cmd = 0;
      command = null;
   }

   // determine state of bit either: risingedge 2, failing edge -1, 0 (and no change), 1 (and no change)
   private int BitState(int bitNumber, int previousWord, int currentWord) {
      int bs = 0;
      int mask = 0;
      switch (bitNumber) {
         case 0:
            mask = MASK_BIT0;
            break;
         case 1:
            mask = MASK_BIT1;
            break;
         case 2:
            mask = MASK_BIT2;
            break;
         case 3:
            mask = MASK_BIT3;
            break;
         case 4:
            mask = MASK_BIT4;
            break;
         case 5:
            mask = MASK_BIT5;
            break;
         case 6:
            mask = MASK_BIT6;
            break;
         case 7:
            mask = MASK_BIT7;
            break;
         case 8:
            mask = MASK_BIT8;
            break;
         case 9:
            mask = MASK_BIT9;
            break;
         case 10:
            mask = MASK_BIT10;
            break;
         case 11:
            mask = MASK_BIT11;
            break;
         case 12:
            mask = MASK_BIT12;
            break;
         case 13:
            mask = MASK_BIT13;
            break;
         case 14:
            mask = MASK_BIT14;
            break;
         case 15:
            mask = MASK_BIT15;
            break;
      }
      if ((currentWord & mask) == mask) {
         if ((previousWord & mask) == 0x0) {
            bs = 2;
         } else {
            bs = 1;
         }
      } else if ((currentWord & mask) == 0x0) {
         if ((previousWord & mask) == mask) {
            bs = -1;
         } else {
            bs = 0;
         }
      }
      return bs;
   }
   private StatusWord getStatus() {
      int statusWord = tpdo_1a00.statusWord.get();

      state = StatusWord.NOTREADYTOSWITCHON;

      if ((statusWord & MASK_NOTREADYTOSWITCHON) == STATUS_NOTREADYTOSWITCHON) {
         state = StatusWord.NOTREADYTOSWITCHON;
      } else if ((statusWord & MASK_SWITCHONDISABLED) == STATUS_SWITCHONDISABLED) {
         state = StatusWord.SWITCHONDISABLED;
      } else if ((statusWord & MASK_READYTOSWITCHON) == STATUS_READYTOSWITCHON) {
         state = StatusWord.READYTOSWITCHON;
      } else if ((statusWord & MASK_SWITCHON) == STATUS_SWITHON) {
         state = StatusWord.SWITCHEDON;
      } else if ((statusWord & MASK_OPERATIONENABLED) == STATUS_OPERATIONENABLED) {
         state = StatusWord.OPERATIONENABLE;
         int posReachedInt = BitState(10, previousStatusWord, statusWord);
         switch (posReachedInt) {
            case 2:
               onRouteBoolean = false;
               waitingToReachNewPositionBoolean = false;
               PositionReachedBoolean = true;
               break;
            case -1:
               onRouteBoolean = true;
               waitingToReachNewPositionBoolean = true;
               readyForNewSetPointBoolean = true;
               break;
            case 1:
               onRouteBoolean = false;
               break;
            case 0:
               onRouteBoolean = true;
               break;
         }
         int AcknowledgeInt = BitState(12, previousStatusWord, statusWord);
         switch (AcknowledgeInt) {
            case 2:
               setpointAcknowledgedBoolean = true;
               break;
            case -1:
               waitingOnSetpointAcknowledgedBoolean = false;
               break;
            case 1:
               break;
            case 0:
               break;
         }
      } else if ((statusWord & MASK_QUICKSTOPACTIVE) == STATUS_QUICKSTOPACTIVE) {
         state = StatusWord.QUICKSTOPACTIVE;
      } else if ((statusWord & MASK_FAULT) == STATUS_FAULT) {
         state = StatusWord.FAULT;
      } else {
      }
      return state;
   }

   //*****************main looped function
   public void doStateControl() {
      state = getStatus();

      switch (state) {
         case NOTREADYTOSWITCHON: {
            break;
         }
         case SWITCHONDISABLED: {

            if (command == ControlWord.FAULTRESET) {
               command = ControlWord.NOACTION;
            } else if (enableDrive) {
               command = ControlWord.SHUTDOWN;
            }
            break;
         }
         case READYTOSWITCHON: {
            command = ControlWord.SWITCHON;
            break;
         }
         case SWITCHEDON: {
            if (enableDrive) {
               command = ControlWord.ENABLEOPERATION;
            } else {
               command = ControlWord.DISABLEVOLTAGE;
            }
            break;
         }
         case OPERATIONENABLE: {
            if (!enableDrive) {
               command = ControlWord.DISABLEVOLTAGE;
            } else {
               command = ControlWord.ENABLEOPERATION;
            }
            break;
         }
         case QUICKSTOPACTIVE: {

            break;
         }
         case FAULT: {

            if (command == ControlWord.FAULTRESET) {
               command = ControlWord.NOACTION;
            } else {
//               System.err.println("Drive fault detected, resetting");
               command = ControlWord.FAULTRESET;
            }
            enableDrive = false;

            break;
         }
         default: {
            throw new RuntimeException("Unknown state");
         }
      }
      if (command != null) {
         setControlWord(command);
      }
      previousControlWord =rpdo_1600.controlWord.get();
      previousStatusWord =tpdo_1a00.statusWord.get();
   }

   //**********Public functions******************
   public void disableYaskawaErrorCodeReading() {
      readYaskawaErrorCode = false;
   }
   public ControlWord getCurrentControlword() {
      return command;
   }

   public boolean isPositionReached() {
      return PositionReachedBoolean;
   }

   public boolean isReadyForNewSetPointBoolean() {
      return readyForNewSetPointBoolean;
   }

   public boolean isOnRoute() {
      return onRouteBoolean;
   }

   public void setNextPosition() {
      requestToMoveToNewSetPositionBoolean = true;
      PositionReachedBoolean=false;
   }

   public void enableDrive() {
      enableDrive = true;
   }

   public void disableDrive() {
      enableDrive = false;
   }

   public void setEnableDrive(boolean enable) {
      enableDrive = enable;
   }

   public boolean isDriveOperational() {
      return getStatus() == StatusWord.OPERATIONENABLE;
   }
   public boolean isReady() {
      int statusWord = tpdo_1a00.statusWord.get();
      return isOperational() && statusWord != 0 && getStatus() != StatusWord.NOTREADYTOSWITCHON;
   }

   public void setMaxTorque(int torque) {
      rpdo_1600.maxTorque.set(torque);
   }

   public void setTargetTorque(short torque) {
      rpdo_1600.targetTorque.set(torque);
   }

   public void setTargetPosition(int position) {
      rpdo_1600.targetPosition.set(position);
      readyForNewSetPointBoolean=false;
      PositionReachedBoolean =false;
   }

   public int getTargetPosition() {
      return rpdo_1600.targetPosition.get();
   }

   public void setTargetVelocity(int velocity) {
      rpdo_1600.targetVelocity.set(velocity);
   }
   public int getTargetVelocity() {
      return rpdo_1600.targetVelocity.get();
   }

   public void setModeOfOperation(YaskawaModeOfOperation mode) {
      rpdo_1600.modeOfOperation.set(mode.getMode());
      PositionReachedBoolean = false;
   }

   public boolean isYaskawaOperational() {
      return this.isOperational();
   }

   public int getActualPosition() {
      return tpdo_1a00.positionActualValue.get();
   }

   public int getStatusVar() {
      return tpdo_1a00.statusWord.get();
   }

   public double getActualTorque() {
      return (((double) tpdo_1a00.torqueActualValue.get() * (double) maxDriveTorque) / 1000000.0);
   }

   //********************Initiation routine************************
   public SGD7S(int alias, int position) throws IOException {
      super(vendorID, productCode, alias, position);
      //***************Receive PDOs***************************************
      rpdo_1600 = new RPDO_1600();
      rpdo_1601 = new RPDO_1601();
      //private final RPDO_1602 rpdo_1602 = new RPDO_1602();
      //private final RPDO_1603 rpdo_1603 = new RPDO_1603();
      //**************Transmit PDOs****************************************
      tpdo_1a00 = new TPDO_1a00();
      tpdo_1a01 = new TPDO_1a01();
      //private final TPDO_1a02 TPDO_1a02 = new TPDO_1a02();
      //private final TPDO_1a03 TPDO_1a03 = new TPDO_1a03();


      //Sync Manager, only SM2 and SM3 are used for PDOs.  SM0 and SM1 are used for the mailbox mechanism (See: EtherCAT_Application_Manual.pdf)
      for (int i = 2; i < 4; i++) {
         registerSyncManager(new SyncManager(i, true));
      }
      //sm(2).registerPDO(rpdo_1601);
      sm(2).registerPDO(rpdo_1600);
      //sm(3).registerPDO(tpdo_1a01);
      sm(3).registerPDO(tpdo_1a00);
   }
   @Override
   protected void shutdown()
   {
      disableDrive();
      doStateControl();
   }

   @Override
   protected boolean hasShutdown()
   {
      return !isDriveOperational();
   }

   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      writeSDO(0x60a4, 0x1, 25);

      profileJerk = readSDOUnsignedInt(0x60a4, 0x1);
      System.out.println(toString() + " Profile jerk " + profileJerk);

      if(dcEnabled)
      {
         configureDCSync0(true, cycleTimeInNs, 0);
      }
      else
      {
         configureDCSync0(false, 0, 0);
      }
   }
}
