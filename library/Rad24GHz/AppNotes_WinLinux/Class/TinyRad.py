# ADF24Tx2RX8 -- Class for 24-GHz Radar
#
# Copyright (C) 2015-11 Inras GmbH Haderer Andreas
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD license.  See the LICENSE file for details.

from . import DevAdf5904
from . import DevAdf5901
from . import DevAdf4159
from . import UsbADI
from    numpy import *
import weakref

class TinyRad(UsbADI.UsbADI):

    def __init__(self, stConType='Usb', *args):
        # Call Constructor of DemoRad Class
        super(TinyRad, self).__init__(stConType, *args)
        self.DebugInf                   =   0
        #>  Object of first receiver (DevAdf5904 class object)
        self.Adf_Rx                =   []
        #>  Object of transmitter (DevAdf5901 class object)
        self.Adf_Tx                 =   []
        #>  Object of transmitter Pll (DevAdf4159 class object)
        self.Adf_Pll                    =   []

        self.Rf_USpiCfg_Pll_Chn         =   2
        self.Rf_USpiCfg_Tx_Chn          =   1
        self.Rf_USpiCfg_Rx_Chn          =   0

        self.Rf_fStrt                   =   24e9
        self.Rf_fStop                   =   24.256e9
        self.Rf_TRampUp                 =   256e-6
        self.Rf_TRampDo                 =   256e-6
        self.Rf_fs                      =   1.0e6

        self.Rf_VcoDiv                  =   2

        self.stRfVers                   =   '1.0.0'

        self.Seq = []
        self.FrmMeasSiz = 1

        # Initialize Receiver

        dUSpiCfg                        =   dict()
        dUSpiCfg                        =   {   "Mask"          : 7,
                                                "Chn"           : self.Rf_USpiCfg_Rx_Chn
                                            }
        self.Adf_Rx = DevAdf5904.DevAdf5904(weakref.ref(self), dUSpiCfg)
        dUSpiCfg                        =   dict()
        dUSpiCfg                        =   {   "Mask"          : 7,
                                                "Chn"           : self.Rf_USpiCfg_Tx_Chn
                                            }

        self.Adf_Tx                     =   DevAdf5901.DevAdf5901(weakref.ref(self), dUSpiCfg)

        dUSpiCfg                        =   dict()
        dUSpiCfg                        =   {   "Mask"          : 7,
                                                "Chn"           : self.Rf_USpiCfg_Pll_Chn
                                            }
        self.Adf_Pll                    =   DevAdf4159.DevAdf4159(weakref.ref(self), dUSpiCfg)
        
        self.Computation.Enable();
        self.Computation.SetParam('fStrt', self.Rf_fStrt);
        self.Computation.SetParam('fStop', self.Rf_fStop);
        self.Computation.SetParam('TRampUp', self.Rf_TRampUp);

    def SetFileParam(self, stKey, Val, DataType='STRING'):
        if isinstance(stKey,str):
            self.ConSetFileParam(stKey, Val, DataType);
        else:
            print('Key is not of type string');
        
    def GetFileParam(self, stKey, stType='UNKNOWN'):   
        if isinstance(stKey,str):
            return self.ConGetFileParam(stKey, stType);
        else:
            print('Key is not of type string')
            return [];
            
    # DOXYGEN ------------------------------------------------------
    #> @brief Get Version information of Adf24Tx2Rx8 class
    #>
    #> Get version of class
    #>      - Version string is returned as string
    #>
    #> @return  Returns the version string of the class (e.g. 0.5.0)
    def     RfGetVers(self):
        return self.stVers

    # DOXYGEN ------------------------------------------------------
    #> @brief Get attribute of class object
    #>
    #> Reads back different attributs of the object
    #>
    #> @param[in]   stSel: String to select attribute
    #>
    #> @return      Val: value of the attribute (optional); can be a string or a number
    #>
    #> Supported parameters
    #>      -   <span style="color: #ff9900;"> 'TxPosn': </span> Array containing positions of Tx antennas
    #>      -   <span style="color: #ff9900;"> 'RxPosn': </span> Array containing positions of Rx antennas
    #>      -   <span style="color: #ff9900;"> 'ChnDelay': </span> Channel delay of receive channels
    def Get(self, *varargin):
        if len(varargin) > 0:
            stVal       = varargin[0]
            if stVal == 'TxPosn':
                Ret     =   zeros(2)
                Ret[0]  =   -18.654e-3
                Ret[1]  =   0.0e-3
                return Ret
            elif stVal == 'RxPosn':
                Ret     =   arange(4)
                Ret     =   Ret*6.2170e-3 + 32.014e-3
                return Ret
            elif stVal == 'B':
                Ret     =   (self.Rf_fStop - self.Rf_fStrt)
                return  Ret
            elif stVal == 'kf':
                Ret     =   (self.Rf_fStop - self.Rf_fStrt)/self.Rf_TRampUp
                return  Ret
            elif stVal == 'kfUp':
                Ret     =   (self.Rf_fStop - self.Rf_fStrt)/self.Rf_TRampUp
                return  Ret
            elif stVal == 'kfDo':
                Ret     =   (self.Rf_fStop - self.Rf_fStrt)/self.Rf_TRampDo
                return  Ret
            elif stVal == 'fc':
                Ret     =   (self.Rf_fStop + self.Rf_fStrt)/2
                return  Ret
            elif stVal == 'fs':
                if (self.cType == 'RadServe') and (self.cReplay > -1):
                    Ret = self.ConGet(stVal, 'DOUBLE');
                    self.Rf_fs = Ret;
                else:
                    Ret     =   self.Rf_fs
                return  Ret
            elif stVal == 'NrChn':
                if (self.cType == 'RadServe') and (self.cReplay > -1):
                    Ret = self.ConGet(stVal, 'INT');
                    self.Computation.SetNrChn(Ret);
                else:
                    Ret = self.Rad_NrChn;
                return Ret
            elif stVal == 'N':
                if (self.cType == 'RadServe') and (self.cReplay > -1):
                    Ret = self.ConGet(stVal, 'INT');
                else:
                    Ret = self.Rad_N
                return Ret
            elif stVal == 'FileSize':
                if (self.cType == 'RadServe') and (self.cReplay > -1):
                    Ret = int(self.ConGet(stVal, 'DOUBLE'));
                else:
                    Ret = 0;
                return  Ret
            elif stVal == 'MeasStart':
                if (self.cType == 'RadServe') and (self.cReplay > -1):
                    Ret = self.ConGet(stVal, 'DOUBLE');
                else:
                    Ret = 0;
                return  Ret
            elif stVal == 'ExtensionSize':
                if (self.cType == 'RadServe') and (self.cReplay > -1):
                    Ret = self.ConGet(stVal, 'DOUBLE');
                else:
                    Ret = 0;
                return  Ret
        return -1

    # DOXYGEN ------------------------------------------------------
    #> @brief Enable all receive channels
    #>
    #> Enables all receive channels of frontend
    #>
    #>
    #> @note: This command calls the Adf4904 objects Adf_Rx
    #>        In the default configuration all Rx channels are enabled. The configuration of the objects can be changed before
    #>        calling the RxEna command.
    #>
    #> @code
    #> CfgRx1.Rx1      =   0;
    #> Brd.Adf_Rx1.SetCfg(CfgRx1);
    #> CfgRx2.All      =   0;
    #> Brd.Adf_Rx2.SetCfg(CfgRx2);
    #> @endcode
    #>  In the above example Chn1 of receiver 1 is disabled and all channels of receiver Rx2 are disabled
    def     RfRxEna(self):
        self.RfAdf5904Ini(1);

    # DOXYGEN ------------------------------------------------------
    #> @brief Configure receivers
    #>
    #> Configures selected receivers
    #>
    #> @param[in]   Mask: select receiver: 1 receiver 1; 2 receiver 2
    #>
    def     RfAdf5904Ini(self, Mask):
        if Mask == 1:
            self.Adf_Rx.Ini();
            if self.DebugInf > 10:
                print('Rf Initialize Rx1 (ADF5904)')
        else:
            pass

    # DOXYGEN -------------------------------------------------
    #> @brief Displays status of frontend in Matlab command window
    #>
    #> Display status of frontend in Matlab command window
    def     BrdDispSts(self):
        self.BrdDispInf()

    #def     BrdGetData(self):
    #    return self.Dsp_GetChirp(self.StrtIdx,self.StopIdx)

    def     RfGetChipSts(self):
        lChip   =   list()
        Val     =   self.Fpga_GetRfChipSts(1)
        print(Val)
        if Val[0] == True:
            Val     =   Val[1]
            if len(Val) > 2:
                if Val[0] == 202:
                    lChip.append(('Adf4159 PLL', 'No chip information available'))
                    lChip.append(('Adf5901 TX', 'No chip information available'))
                    lChip.append(('Adf5904 RX', 'No chip information available'))
        return lChip

    # DOXYGEN ------------------------------------------------------
    #> @brief Set attribute of class object
    #>
    #> Sets different attributes of the class object
    #>
    #> @param[in]     stSel: String to select attribute
    #>
    #> @param[in]     Val: value of the attribute (optional); can be a string or a number
    #>
    #> Supported parameters
    #>              - Currently no set parameter supported
    def RfSet(self, *varargin):
        if len(varargin) > 0:
            stVal   =   varargin[0]


    # DOXYGEN ------------------------------------------------------
    #> @brief Enable transmitter
    #>
    #> Configures TX device
    #>
    #> @param[in]   TxChn
    #>                  - 0: off
    #>                  - 1: Transmitter 1 on
    #>                  - 2: Transmitter 2 on
    #> @param[in]   TxPwr: Power register setting 0 - 256; only 0 to 100 has effect
    #>
    #>
    def     RfTxEna(self, TxChn, TxPwr):
        TxChn       =   (TxChn % 3)
        TxPwr       =   (TxPwr % 2**8)
        if self.DebugInf > 10:
            stOut   =   "Rf Initialize Tx (ADF5901): Chn: " + str(TxChn) + " | Pwr: " + str(TxPwr)
            print(stOut)
        dCfg            =   dict()
        dCfg["TxChn"]   =   TxChn
        dCfg["TxPwr"]   =   TxPwr


        self.Adf_Tx.SetCfg(dCfg)
        self.Adf_Tx.Ini()


    def RfMeas(self, dCfg):  
        dAdarCfg = dict()        
        dAdarCfg['X'] = 3
        dAdarCfg['R'] = 5
        dAdarCfg['N'] = 76
        dAdarCfg['M'] = 100
        RetVal  =   self.Dsp_CfgAdarPll(dAdarCfg)
        if RetVal[1]   == 1:
            print('ADAR PLL Locked')
        else:
            print('ADAR PLL not locked')
        

        #Check configuration
        if dCfg['Perd'] <= dCfg['N']*1e-6 + 20e-6:
            print('Period is to short: increase period > TRampUp + 20us')
        
        
        if (4*dCfg['N']*dCfg['FrmMeasSiz']*dCfg['CycSiz']*len(dCfg['Seq'])) > int('0x60000',16):
            print('Required memory greater than DSP buffer size: decrease CycSiz, N, FrmMeasSiz')
        
        
        if dCfg['FrmMeasSiz'] > dCfg['FrmSiz']:
            print('Number of frames to record > number of frames')

        self.RfAdf4159Ini(dCfg)
        self.Dsp_CfgMeas(dCfg)
        self.Dsp_StrtMeas(dCfg)     
        
        self.ConSetFileParam('N', int(self.Rad_N), 'INT');
        self.ConSet('Mult', self.FrmMeasSiz * len(self.Seq));    

        self.Computation.SetParam('FuSca', self.FuSca);
        self.Computation.SetParam('fs', self.Rf_fs);
        
    def BrdRst(self):
        DspCmd = zeros(2, dtype='uint32')
        Cod = int('0x9031',16)
        DspCmd[0] = 1
        DspCmd[1] = 0
        Ret     = self.CmdExec(0, Cod, DspCmd)
        return Ret
        
    def Dsp_CfgMeas(self, dCfg):
            
        self.Seq         =   dCfg['Seq']
        self.FrmMeasSiz  =   dCfg['FrmMeasSiz']
        self.Dsp_CfgMeasSiz(dCfg)
        Ret     =   self.Dsp_CfgMeasSeq(dCfg)
        return Ret
        
        
    def Dsp_CfgAdarPll(self, dCfg):
        DspCmd = zeros(6, dtype='uint32')
        Cod = int('0x9031',16)
        DspCmd[0] = 1
        DspCmd[1] = 12
        DspCmd[2] = dCfg['X']
        DspCmd[3] = dCfg['R']
        DspCmd[4] = dCfg['N']
        DspCmd[5] = dCfg['M']
        Ret     = self.CmdExec(0, Cod, DspCmd)
        return Ret
            
    def Dsp_SetAdarReg(self, Regs):
        DspCmd =   zeros(2 + len(Regs), dtype='uint32')
        Regs = Regs.flatten() 
        Cod = int('0x9031',16)
        DspCmd[0]  = 1
        DspCmd[1]  = 10
        DspCmd[2:] = Regs
        Ret     = self.CmdExec(0, Cod, DspCmd)
        return Ret
        
    def Dsp_CfgMeasSeq(self, dCfg):

        Seq = dCfg['Seq']
        # Rotate by one because state refers to next measurement state
        Seq = [Seq[1:], [Seq[0]]]
        Seq = concatenate(Seq)
        DspCmd = zeros(2 + len(Seq), dtype='uint32')
        Cod = int('0x9031',16)
        DspCmd[0] = 1
        DspCmd[1] = 3
        DspCmd[2:] = Seq
        Ret     = self.CmdExec(0, Cod, DspCmd)
        return Ret
        
    def Dsp_CfgMeasSiz(self, dCfg):
                        
        DspCmd = zeros(6, dtype='uint32')
        Cod = int('0x9031',16)
        DspCmd[0] = 1
        DspCmd[1] = 2
        DspCmd[2] = len(dCfg['Seq'])
        DspCmd[3] = dCfg['FrmSiz']
        DspCmd[4] = dCfg['FrmMeasSiz']
        DspCmd[5] = dCfg['CycSiz']
        Ret     = self.CmdExec(0, Cod, DspCmd)
        return Ret
            
            
    def Dsp_StrtMeas(self, dCfg):
            
        self.Rad_N = dCfg['N']
        DspCmd = zeros(4, dtype='uint32')
        Cod = int('0x9031',16)
        DspCmd[0] = 1
        DspCmd[1] = 1
        DspCmd[2] = floor(dCfg['Perd']/10e-9)
        DspCmd[3] = dCfg['N']
        Ret     = self.CmdExec(0, Cod, DspCmd)
        return Ret
  
    def Dsp_GetDmaErr(self):
        DspCmd = zeros(4, dtype='uint32')
        Cod = int('0x9033',16)
        DspCmd[0] = 1
        DspCmd[1] = 1
        Ret     = self.CmdExec(0, Cod, DspCmd)
        return Ret
                  
    def BrdGetData(self, NrPack=1, Extension=False):
        NumSeq = len(self.Seq) * self.FrmMeasSiz * NrPack;
        if (self.cReplay > -1):
            NumSeq = 1 * NrPack;
        Len = self.Rad_NrChn * self.Rad_N * NumSeq;
        
        if self.cType == 'Usb':
            DspCmd = zeros(4, dtype='uint32')
            Cod = int('0x9032',16)
            DspCmd[0] = 1
            DspCmd[1] = 1
            DspCmd[2] = Len
            #Cfg.N*4*numel(Cfg.Seq)*Cfg.FrmMeasSiz;
            self.usbLen   = Len;
            self.usbState = 1; # start cmd send
            self.CmdSend(0, Cod, DspCmd);
            self.usbState = 2; # start get data
            UsbData = self.ConGetUsbData(Len * 2);
            Data    = reshape(UsbData, (int(Len/4), 4));

            self.usbState = 3;  # cmd Recv
            self.CmdRecv();

            self.usbState = 0; # do nothing
            
        elif self.cType == 'RadServe':
            #----------------------------------------------------------
            # Open Data Port
            #----------------------------------------------------------
            if self.hConDat < 0:
                if not Extension:
                    self.GetDataPort()
                    self.cDataIdx = 0;
                else:
                    self.GetExtensionPort(Extension)
                    self.cDataIdx = 0;
                if self.cDataOpened < 0:
                    quit();
            
            if (not Extension) and self.Computation.GetDataType() <= 1 and (self.cReplayExt < 1):
                UsbData             =   self.ConGetData(Len * 2)                
                UsbData             =   double(UsbData);
                Data                =   zeros((int(self.Rad_N) * int(NumSeq), int(self.Rad_NrChn)))
                UsbData             =   UsbData.reshape(NumSeq, self.Rad_NrChn, self.Rad_N);
                for Seq in range(0, NumSeq):
                    SeqStrt      = int( (Seq)     * self.Rad_N);
                    SeqStop      = int( (Seq + 1) * self.Rad_N);
                    for Chn in range(0, self.Rad_NrChn):
                        Data[SeqStrt:SeqStop, Chn] = UsbData[Seq, Chn, :];
                self.cDataIdx = self.cDataIdx + 1;
            elif not Extension:
                self.cDataIdx = self.cDataIdx + 1 * self.cExtMult;
                
                return self.Computation.GetData(1);
            else:
                Data                =   self.ConGetData(1 * self.cExtSize * 2);
                self.cDataIdx       =   self.cDataIdx + self.cExtMult * 1;
        
        return Data
    
    # DOXYGEN ------------------------------------------------------
    #> @brief Initialize PLL with selected configuration
    #>
    #> Configures PLL
    #>
    #> @param[in]   Cfg: structure with PLL configuration
    #>      -   <span style="color: #ff9900;"> 'fStrt': </span> Start frequency in Hz
    #>      -   <span style="color: #ff9900;"> 'fStrt': </span> Stop frequency in Hz
    #>      -   <span style="color: #ff9900;"> 'TRampUp': </span> Upchirp duration in s
    #>
    #> %> @note Function is obsolete in class version >= 1.0.0: use function Adf_Pll.SetCfg() and Adf_Pll.Ini()
    def     RfAdf4159Ini(self, Cfg):
        self.Adf_Pll.SetCfg(Cfg);
        self.Adf_Pll.Ini();

    # DOXYGEN ------------------------------------------------------
    #> @brief <span style="color: #ff0000;"> LowLevel: </span> Generate hanning window
    #>
    #> This function returns a hanning window.
    #>
    #>  @param[in]   M: window size
    #>
    #>  @param[In]   N (optional): Number of copies in second dimension
    def     hanning(self, M, *varargin):
        #m   =   [-(M-1)/2: (M-1)/2].';
        m       =   linspace(-(M-1)/2,(M-1)/2,M)  
        Win     =   0.5 + 0.5*cos(2*pi*m/M)
        if len(varargin) > 0:
            N       =   varargin[0]
            Win     =   broadcast_to(Win,(N,M)).T
        return Win
