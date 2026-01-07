# mmw_parse_script.py - RADAR DATA COLLECTION ONLY
import serial
import time
import numpy as np
import datetime
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from parser_mmw_demo import parser_one_mmw_demo_output_packet

# Try to import the warning system
try:
    from real_time_warning_system import AdvancedRealTimeWarningSystem
    WARNING_SYSTEM_AVAILABLE = True
except ImportError:
    print("⚠️ real_time_warning_system module not found. Install scikit-learn: pip install scikit-learn")
    WARNING_SYSTEM_AVAILABLE = False

# Configuration
configFileName = 'xwr68xx_profile_2025_11_26T17_55_29_777.cfg'
DEBUG = False

# Constants
maxBufferSize = 2**15
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15, dtype='uint8')
byteBufferLength = 0
magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
detObj = {}
frameData = {}
currentIndex = 0
word = [1, 2**8, 2**16, 2**24]

# Global variables
csv_file = None
csv_writer = None
frame_counter = 0
warning_system = None

# Initialize warning system
if WARNING_SYSTEM_AVAILABLE:
    try:
        warning_system = AdvancedRealTimeWarningSystem('radar_classifier_models.pkl')
        print("✓ Real-time warning system initialized")
    except Exception as e:
        print(f"⚠️ Warning system error: {e}")
        warning_system = None

# Serial configuration function
def serialConfig(configFileName):
    global CLIport, Dataport
    CLIport = serial.Serial('COM13', 115200)
    Dataport = serial.Serial('COM14', 921600)

    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i + '\n').encode())
        print(i)
        time.sleep(0.01)
    return CLIport, Dataport

# Parse configuration file
def parseConfigFile(configFileName):
    configParameters = {}
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    
    for i in config:
        splitWords = i.split(" ")
        numRxAnt = 4
        numTxAnt = 3
        
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
                
            digOutSampleRate = int(splitWords[11])
            
        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = int(splitWords[5])
    
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters

# Read and parse radar data
def readAndParseData14xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength, csv_file, csv_writer, frame_counter
    
    magicOK = 0
    dataOK = 0
    frameNumber = 0
    detObj = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype='uint8')
    byteCount = len(byteVec)

    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
    
    if byteBufferLength > 16:
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]
        startIdx = []
        
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
        
        if startIdx:
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]), dtype='uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
            
            if byteBufferLength < 0:
                byteBufferLength = 0

            totalPacketLen = np.matmul(byteBuffer[12:12+4], word)
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    if magicOK:
        readNumBytes = byteBufferLength
        allBinData = byteBuffer
        
        totalBytesParsed = 0
        numFramesParsed = 0

        parser_result, headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, \
        detectedX_array, detectedY_array, detectedZ_array, detectedV_array, detectedRange_array, \
        detectedAzimuth_array, detectedElevation_array, detectedSNR_array, detectedNoise_array = \
            parser_one_mmw_demo_output_packet(allBinData[totalBytesParsed::1], readNumBytes-totalBytesParsed, DEBUG)

        if parser_result == 0:
            totalBytesParsed += (headerStartIndex + totalPacketNumBytes)
            numFramesParsed += 1
            
            # Initialize CSV writer
            global csv_file, csv_writer
            if csv_file is None:
                import csv
                csv_file = open('mmw_demo_output.csv', 'w', newline='')
                csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                csv_writer.writerow(["timestamp", "frame", "DetObj#", "x", "y", "z", "v", "snr", "noise"])
                print("CSV file initialized. Starting data capture...")
            
            # Write data to CSV
            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            for obj in range(numDetObj):
                csv_writer.writerow([
                    current_time, frame_counter, obj,
                    detectedX_array[obj], detectedY_array[obj], detectedZ_array[obj],
                    detectedV_array[obj], detectedSNR_array[obj], detectedNoise_array[obj]
                ])
            
            # Create detObj for visualization
            detObj = {
                "numObj": numDetObj,
                "x": detectedX_array,
                "y": detectedY_array,
                "z": detectedZ_array,
                "v": detectedV_array,
                "range": detectedRange_array,
                "snr": detectedSNR_array,
                "noise": detectedNoise_array
            }
            
            frame_counter += 1
            csv_file.flush()
            dataOK = 1
            
            if DEBUG and numDetObj > 0:
                print(f"Frame {frame_counter-1}: {numDetObj} objects")
        
        shiftSize = totalPacketNumBytes
        byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
        byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]), dtype='uint8')
        byteBufferLength = byteBufferLength - shiftSize
        
        if byteBufferLength < 0:
            byteBufferLength = 0

    return dataOK, frameNumber, detObj

# PyQt5 Widget for visualization
class MyWidget(pg.GraphicsLayoutWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        
        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)
        
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.start()
        self.timer.timeout.connect(self.onNewData)
        
        self.plotItem = self.addPlot(title="Radar Point Cloud - Real-time")
        self.plotItem.setLabel('left', 'Y Position (m)')
        self.plotItem.setLabel('bottom', 'X Position (m)')
        self.plotItem.showGrid(x=True, y=True, alpha=0.3)
        self.plotItem.setXRange(-10, 10)
        self.plotItem.setYRange(0, 20)
        
        self.plotDataItem = self.plotItem.plot([], pen=None, 
            symbolBrush=(255, 0, 0), symbolSize=8, symbolPen=None)
        
        # Warning text display
        self.warningText = pg.TextItem("", color=(255, 0, 0), anchor=(0, 0))
        self.plotItem.addItem(self.warningText)
        self.warningText.setPos(-9, 18)

    def setData(self, x, y):
        self.plotDataItem.setData(x, y)
        
        # Update warning text
        if warning_system and hasattr(warning_system, 'frame_counter'):
            warning_count = len([h for h in warning_system.warning_history 
                               if h['frame'] == warning_system.frame_counter - 1])
            if warning_count > 0:
                self.warningText.setText(f"⚠️ {warning_count} warnings")
                self.warningText.setColor((255, 0, 0))
            else:
                self.warningText.setText("✓ All clear")
                self.warningText.setColor((0, 255, 0))

    def update(self):
        global detObj
        x = []
        y = []
        
        dataOk, frameNumber, detObj = readAndParseData14xx(Dataport, configParameters)
        
        if dataOk and len(detObj["x"]) > 0:
            x = detObj["x"]
            y = detObj["y"]
            
            # Process with warning system
            if warning_system:
                result = warning_system.process_radar_frame(detObj)
                # Print warnings to console
                if result['warnings'] or result['critical_alerts']:
                    print(f"\n=== Frame {result['frame']} Warnings ===")
                    for warning in result['warnings']:
                        print(f"⚠️  WARNING: {warning['object_type']} at {warning['distance']}")
                    for alert in result['critical_alerts']:
                        print(f"🚨 CRITICAL: {alert['object_type']} at {alert['distance']}!")
            
            print(f"Frame {frame_counter}: {len(x)} objects detected")
        
        return dataOk, x, y

    def onNewData(self):
        dataOk, newx, newy = self.update()
        if dataOk:
            self.setData(newx, newy)

# Main function
def main():
    global configParameters, warning_system
    
    # Configure serial port
    CLIport, Dataport = serialConfig(configFileName)
    
    # Get configuration parameters
    configParameters = parseConfigFile(configFileName)
    
    print("\n" + "="*60)
    print("TI IWR6843 Radar - Real-time Monitoring System")
    print("="*60)
    print(f"Configuration: {configFileName}")
    print(f"CSV Output: mmw_demo_output.csv")
    if warning_system:
        print(f"ML Warning System: ✓ Active")
    else:
        print(f"ML Warning System: ⚠️ Not available")
    print("\nPress Ctrl+C or close window to stop")
    print("="*60 + "\n")
    
    # Create Qt application
    app = QtWidgets.QApplication([])
    pg.setConfigOptions(antialias=True)
    
    # Create and show window
    win = MyWidget()
    win.show()
    win.resize(1000, 800)
    win.setWindowTitle("TI IWR6843 Radar - Real-time Point Cloud with ML Warnings")
    
    try:
        app.exec_()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Cleanup
        print("\n" + "="*60)
        print("Shutting down radar system...")
        
        # Stop radar
        CLIport.write(('sensorStop\n').encode())
        time.sleep(0.1)
        
        # Close CSV file
        global csv_file
        if csv_file is not None:
            csv_file.close()
            print(f"✓ Data saved to mmw_demo_output.csv")
            print(f"  Total frames: {frame_counter}")
        
        # Save warning system data
        if warning_system:
            warning_system.save_session_data('radar_session_with_warnings.csv')
            print(warning_system.get_system_status())
        
        # Close serial ports
        CLIport.close()
        Dataport.close()
        print("✓ Serial connections closed")
        print("="*60)
        print("Program terminated successfully.")

if __name__ == "__main__":
    main()