#
#
#
#
#
#
#

from PyQt4 import QtCore, QtGui
import PyQt4.Qwt5 as Qwt5


import communicator as com
import numpy as np
import time

sampletime = 0.0384

Stopped = False


try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)
        

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.setFixedSize(580, 579)
        self.textBrowser = QtGui.QTextBrowser(Form)
        self.textBrowser.setGeometry(QtCore.QRect(30, 30, 256, 71))
        self.textBrowser.setObjectName(_fromUtf8("textBrowser"))
        
        self.lineEdit_port = QtGui.QLineEdit(Form)
        self.lineEdit_port.setGeometry(QtCore.QRect(310, 30, 113, 27))
        self.lineEdit_port.setObjectName(_fromUtf8("lineEdit_port"))
        
        self.label_2 = QtGui.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(430, 30, 70, 21))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        
        self.groupBox = QtGui.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(30, 140, 261, 121))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        
        self.pushButton = QtGui.QPushButton(self.groupBox)
        self.pushButton.setGeometry(QtCore.QRect(160, 35, 91, 34))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.pushButton.clicked.connect(self.SingleMeasure)
        
        self.progressBar = QtGui.QProgressBar(self.groupBox)
        self.progressBar.setGeometry(QtCore.QRect(10, 80, 231, 31))
        self.progressBar.setProperty("value", 0)
        self.progressBar.setObjectName(_fromUtf8("progressBar"))
        self.progressBar.setMinimum(0)
        
        self.label = QtGui.QLabel(self.groupBox)
        self.label.setGeometry(QtCore.QRect(80, 40, 70, 21))
        self.label.setObjectName(_fromUtf8("label"))
        
        self.lineEdit_samples = QtGui.QLineEdit(self.groupBox)
        self.lineEdit_samples.setGeometry(QtCore.QRect(10, 37, 61, 27))
        self.lineEdit_samples.setObjectName(_fromUtf8("lineEdit_samples"))
        self.lineEdit_samples.setToolTip("Number of samples for time-averaged measurement. Recommend 1000 - 5000")
        
        self.groupBox_2 = QtGui.QGroupBox(Form)
        self.groupBox_2.setGeometry(QtCore.QRect(300, 140, 251, 121))
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        
        self.pushButton_continuous = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_continuous.setGeometry(QtCore.QRect(10, 77, 112, 34))
        self.pushButton_continuous.setObjectName(_fromUtf8("pushButton_continuous"))
        self.pushButton_continuous.clicked.connect(self.Continuous)
        
        self.label_3 = QtGui.QLabel(self.groupBox_2)
        self.label_3.setGeometry(QtCore.QRect(95, 42, 70, 21))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        
        self.lineEdit = QtGui.QLineEdit(self.groupBox_2)
        self.lineEdit.setGeometry(QtCore.QRect(10, 37, 80, 27))
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))
        
        self.pushButton_stop = QtGui.QPushButton(self.groupBox_2)
        self.pushButton_stop.setGeometry(QtCore.QRect(130, 77, 112, 34))
        self.pushButton_stop.setObjectName(_fromUtf8("pushButton_stop"))
        self.pushButton_stop.clicked.connect(self.Stop)
        
        self.pushButton_checkport = QtGui.QPushButton(Form)
        self.pushButton_checkport.setGeometry(QtCore.QRect(310, 65, 112, 34))
        self.pushButton_checkport.setObjectName(_fromUtf8("pushButton_checkport"))
        self.pushButton_checkport.clicked.connect(self.checkPort)
            
        self.qwtPlot = Qwt5.QwtPlot(Form)
        self.qwtPlot.setGeometry(QtCore.QRect(20, 280, 521, 281))
        self.qwtPlot.setAxisTitle(Qwt5.QwtPlot.xBottom,"Time / s")
        self.qwtPlot.setAxisTitle(Qwt5.QwtPlot.yLeft,"Phase / degrees")
        self.qwtPlot.setAxisMaxMajor(Qwt5.QwtPlot.yLeft, 4)
        self.qwtPlot.setAxisMaxMinor(Qwt5.QwtPlot.yLeft, 0)
        self.qwtPlot.setObjectName(_fromUtf8("qwtPlot"))
        


        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Polarimeter Control - Kinetics", None))
        self.lineEdit_port.setText(_translate("Form", "COM5", None))
        self.label_2.setText(_translate("Form", "Port", None))
        self.groupBox.setTitle(_translate("Form", "Single Measurement", None))
        self.pushButton.setText(_translate("Form", "Run", None))
        self.label.setText(_translate("Form", "Samples", None))
        self.lineEdit_samples.setText(_translate("Form", "100", None))
        self.groupBox_2.setTitle(_translate("Form", "Continuous Measurement", None))
        self.pushButton_continuous.setText(_translate("Form", "Run", None))
        self.label_3.setText(_translate("Form", ".csv", None))
        self.lineEdit.setText(_translate("Form", "filename", None))
        self.pushButton_stop.setText(_translate("Form", "Stop", None))
        self.pushButton_checkport.setText(_translate("Form", "Check Port", None))
        self.textBrowser.setText("Hello! Check port before performing measurements.")
        
        
    def SingleMeasure(self):
        
        port = str(self.lineEdit_port.text())
        samples = int(self.lineEdit_samples.text()) #number of samples to consider for average
        self.progressBar.setMaximum(samples)        
        
        readings = np.array([])     #numpy array of inputs from microcontroller
        
        try:
            while True: 
                reading = com.ReadDevice(port)      
                readings = np.append(readings, reading)     #array of readings
                self.progressBar.setValue(len(readings))
                
                if len(readings) == samples:
                    average = np.mean(readings)
                    stdev = np.std(readings)
                    outputString = str(round(average,5)) + " +/- " + str(round(stdev, 5))
                    self.textBrowser.setText(outputString)  ##reading goes in browser
                    break
        except:
            self.textBrowser.setText("Reading failed! Have you checked the port?")
            
            
            
    def Stop(self):             #bit of a hack. Slot function stops continuous measurement via global
        global Stopped
        Stopped = True       
    
            
                
    def Continuous(self):       #Reads from device as fast as possible and saves to file vs time. Double exponential smoothing
        self.textBrowser.setText("Initialising...") 
        time.sleep(1)
        readings = np.array([])  
        times = np.array([])
        curve = Qwt5.QwtPlotCurve("Curve 1")
        self.qwtPlot.detachItems()
        filename =  str(self.lineEdit.text())+".csv"
        fout = open(filename,"w")
        port = str(self.lineEdit_port.text()) #which port
        

        
        try:
                
            
			reading = com.ReadDevice(port) 
			readings = np.append(readings, reading)
                

        except:
            self.textBrowser.setText("Reading failed! Have you checked the port?")
            
        begin = time.clock()            #reference 0 time
        
        global Stopped

        try:
            
            while Stopped != True:  #breaks from loop when global Stopped becomes true
                self.pushButton_continuous.setEnabled(False)    #disables start button
                QtGui.QApplication.processEvents()      #checks for button presses (stop button)
                reading = com.ReadDevice(port)
                 
                now = time.clock()-begin 
                readings = np.append(readings, reading)
                times = np.append(times, now)
                outputstring = str(now) + " " + str(round(reading,5)) + "\n" #left column - time in seconds, right column - phase reading
                fout.write(outputstring)
                self.textBrowser.setText(str(reading))
                curve.setData(times, readings)
                curve.attach(self.qwtPlot)
                self.qwtPlot.replot()
                
                
                           
                
            self.textBrowser.setText("Stopped recording")   
            self.pushButton_continuous.setEnabled(True)    #re-enables start button
            fout.close()        
            Stopped = False                 #re-initialise
            
            
        except:
            self.pushButton_continuous.setEnabled(True)
            self.textBrowser.setText("Reading failed! Have you checked the port?")
        
        
        
        
    def checkPort(self):
        port = str(self.lineEdit_port.text())
        check = com.isOpen(port)    #port checking function in communicator
        if check=="ready":
            self.textBrowser.setText("Ready!")
        elif check=="not_ready":
            self.textBrowser.setText("Not ready. Wait for device, or check correct port is selected.")
        else:
            self.textBrowser.setText("Not connected!")
        


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Form = QtGui.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

