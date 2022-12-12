from .AppNotes_WinLinux.Class import TinyRad
import time as time
import numpy as np
from pyqtgraph.Qt import QtGui
from scipy.io import savemat
import pyqtgraph as pg

def connect(action='visualize', duration=10, filename=''):
    measuring = action == 'measure'
    if filename == '':
        filename = time.strftime("Rad24GHz %Y-%m-%d %H-%M-%S.mat", time.localtime())

    maxRange = 10
    chirpDur = 125
    c0 = 3e8
    switchPer = 625e-6
    measRounds = 100
    modF = 1 / (2 * switchPer)
    tDelay = 0.05

    # Setup Connection
    Brd = TinyRad.TinyRad('Usb', '127.0.0.1')
    Brd.BrdRst()

    # Software Version
    Brd.BrdDispSwVers()

    # Configure Receiver
    Brd.RfRxEna()
    TxPwr = 100

    # Configure Transmitter (Antenna 0 - 2, Pwr 0 - 100)
    Brd.RfTxEna(2, TxPwr)

    CalDat = Brd.BrdGetCalDat()['Dat']

    # Configure Measurements
    # Cfg.Perd: time between measuremnets: must be greater than 1 us*N + 10us
    # Cfg.N: number of samples collected: 1e66 * TRampUp = N; if N is smaller
    # only the first part of the ramp is sampled; if N is larger than the
    dCfg = dict()
    dCfg['fStrt'] = 24.00e9
    dCfg['fStop'] = 24.25e9
    dCfg['TRampUp'] = chirpDur * 1e-6
    dCfg['Perd'] = dCfg['TRampUp'] + 100e-6
    dCfg['N'] = 100
    dCfg['Seq'] = [1]
    dCfg['CycSiz'] = 2
    dCfg['FrmSiz'] = 256
    dCfg['FrmMeasSiz'] = 256
    frmMeasSiz = dCfg['FrmMeasSiz']

    Brd.RfMeas(dCfg)
    time.sleep(1)

    # Read actual configuration
    N = int(Brd.Get('N'))
    NrChn = int(Brd.Get('NrChn'))
    fs = Brd.Get('fs')
    Perd = dCfg['Perd']

    # I have no idea why there are duplicate lines
    N = int(Brd.Get('N'))
    TRampUp = Brd.Get('TRampUp')

    # Processing of range profile
    Win2D = Brd.hanning(N, int(frmMeasSiz))
    ScaWin = sum(Win2D[:, 0])
    NFFT = 2**10
    NFFTVel = 2**8
    kf = (dCfg['fStop'] - dCfg['fStrt']) / dCfg['TRampUp']
    vRange = np.arange(NFFT) / NFFT * fs * c0 / (2*kf)
    fc = (dCfg['fStop'] + dCfg['fStrt']) / 2

    RMin = 0.2
    RMax = min(maxRange, (N / TRampUp) * c0 / (4 * 250e6 / TRampUp))
    RMinIdx = np.argmin(np.abs(vRange - RMin))
    RMaxIdx = np.argmin(np.abs(vRange - RMax))
    vRangeExt = vRange[RMinIdx:RMaxIdx]

    WinVel2D = Brd.hanning(int(frmMeasSiz), len(vRangeExt))
    ScaWinVel = sum(WinVel2D[:, 0])
    WinVel2D = WinVel2D.transpose()

    vFreqVel = np.arange(-NFFTVel//2, NFFTVel//2)/NFFTVel*(1/dCfg['Perd'])
    vVel = vFreqVel*c0/(2*fc)

    # Qt Visualization Configurations
    if measuring:
        DataAll = None
        Timestamps = np.array([])
    else:
        App = QtGui.QApplication([])

        Widget = pg.GraphicsLayoutWidget(title='24GHz Radar Live Visualization')
        RPlotItem = Widget.addPlot(row=1, col=1)
        VPlotItem = Widget.addPlot(row=1, col=2)
        RImageItem = pg.ImageItem()
        RPlotItem.addItem(RImageItem)
        VImageItem = pg.ImageItem()
        VPlotItem.addItem(VImageItem)

        RPlotItem.setLabel('left', 'Range', units='m')
        RPlotItem.setLabel('bottom', 'Chirps', units='chirp')
        RImageItem.setColorMap('CET-L9')

        VPlotItem.setLabel('left', 'Range', units='m')
        VPlotItem.setLabel('bottom', 'Vel Bins', units='bins')
        VImageItem.setColorMap('CET-L9')

        RTranslate = QtGui.QTransform()
        RTranslate.scale(1, (RMax - RMin) / len(vRangeExt))
        RTranslate.translate(1, RMin)
        VTranslate = QtGui.QTransform()
        VTranslate.scale((vVel[-1] - vVel[0])/len(vVel), (RMax - RMin) / len(vRangeExt))
        VTranslate.translate(vVel[0], RMin)

        RImageItem.setTransform(RTranslate)
        VImageItem.setTransform(VTranslate)

        Widget.show()

    # Measure and calculate Range Doppler Map
    # frameSize = int(duration / (frmMeasSiz * Perd))
    # Instead of using a fixed frameSize, use time.time() compared to duration
    print("[Rad24GHz] Started {0}".format("Measuring" if measuring else "Visualizing"))
    startTime = time.time()
    while True:
        if time.time() - startTime > duration:
            break

        DataFrame = Brd.BrdGetData()

        if measuring:
            curr = time.time()
            strcurr = time.strftime("%Y-%m-%d %H:%M:%S:", time.localtime(curr)) + str(int(curr % 1 * 1000))
            Timestamps = np.append(Timestamps, [strcurr])
            reshaped = DataFrame.reshape(DataFrame.shape[0], 1, DataFrame.shape[1])
            DataAll = np.array(reshaped, dtype='float64') if DataAll is None else np.append(DataAll, reshaped, axis=1)
        else:
            Data = np.reshape(DataFrame[:, 0], (N, frmMeasSiz), order='F')
            RP = 2 * np.fft.fft(np.multiply(Data, Win2D), n=NFFT, axis=0) / ScaWin * Brd.FuSca
            RPExt = RP[RMinIdx:RMaxIdx, :]
            RD = np.fft.fft(np.multiply(RPExt, WinVel2D), n=NFFTVel, axis=1) / ScaWinVel

            RImageItem.setImage(np.abs(RPExt).transpose()) 
            RPlotItem.setAspectLocked(False)
            VImageItem.setImage(np.abs(RD).transpose())
            VPlotItem.setAspectLocked(False)

            pg.QtGui.QApplication.processEvents()

    if measuring:
        print('[Rad24GHz] Finished Measuring after {0} seconds'.format(time.time() - startTime))
        print('[Rad24GHz] Collected data in shape {0}'.format(DataAll.shape))
    else:
        print('[Rad24GHz] Finished {0} after {1} seconds'
              .format('Visualizing', time.time() - startTime))

    if measuring:
        save = {
            "Data": DataAll,
            "dtime": Timestamps,
            # "Brd": Brd, # Brd not used in procesing code, but in collectData
            "Cfg": dCfg,
            "N": N,
            "NrChn": NrChn,
            "fs": fs,
            "measRounds": DataAll.shape[1],
            "CalDat": CalDat
        }
        # print(save['Data'].dtype)
        savemat(filename, save)
    
    del Brd

if __name__ == '__main__':
    connect()