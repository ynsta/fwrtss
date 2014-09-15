#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (c) 2014, Stany MARCEL <stanypub@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
#


# SETUP:
#
# Instal pip with your distribution (apt-get install python-pip for example)
# package installer then run :
#
# sudo pip install PySide
# sudo pip install pyqtgraph
# sudo pip install pyserial
# sudo pip install crcmod

import serial
import struct
import crcmod
import ctypes
import numpy as np
import PySide
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import time

import os
from cStringIO import StringIO
from base64 import b64decode
from gzip import GzipFile
import tempfile

VERSION     = '0.3'
DESCRIPTION = 'FWRTSS plot (C)Stany MARCEL 2014 (https://www.ynsta.org)'

FWRTSS_MESSAGE_FORMAT = [
    ('id',        'B'),
    ('pmode',     'B'),
    ('count',     'H'),
    ('period_ms', 'I'),
    ('tempdc',    'f'),
    ('target',    'f'),
    ('pwm_dc',    'f'),
    ('offset',    'f'),
    ('cycle_ms',  'I'),
    ('crc',       'I'),
]

# Generated with gzip -c icon.sv | base64
ICON_DATA = '''
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
AAA=
'''


class FWRTSSMessage:
    def __init__(self,
                 bigendian = False):
        self._fields_desc = FWRTSS_MESSAGE_FORMAT
        self._format = ''.join([fmt for (_, fmt) in self._fields_desc])
        self._fields = [f for (f, _) in self._fields_desc]
        num = 0
        self._index = {}
        for field in self._fields:
            self._index[field] = num
            num = num + 1
        self._data = self._nulldata()
        self._is_big = bigendian

    def _get_format(self):
        return self._format

    def _get_fields(self):
        return self._fields

    def _encode(self, datas):
        return struct.pack(self._format, *datas)

    def _decode(self, buf):
        return list(struct.unpack(self._format, buf))

    def _nullbuff(self):
        return self.getsize() * '\x00'

    def _nulldata(self):
        return self._decode(self._nullbuff())

    def _getidx(self, field):
        assert field in self._index.keys(), \
            'ERROR: unknown field %s not in (%s)' % (
                field, ','.join([str(_x) for _x in self._index.keys()]))
        return self._index[field]

    def setfield(self, field, val):
        idx = self._getidx(field)
        self._data[idx] = val

    def setfields(self, field_dict):
        for field, val in field_dict.items():
            idx = self._getidx(field)
            self._data[idx] = val

    def getfield(self, field):
        idx = self._getidx(field)
        return self._data[idx]

    def getfields(self):
        out = dict()
        for key, index in self._index.items():
            out[key] = self._data[index]
        return out

    def setbuf(self, buf):
        self._data = self._decode(buf)

    def getbuf(self):
        return self._encode(self._data)

    def getdata(self):
        return self._data

    def getfieldoffset(self, field):
        idx = self._getidx(field)
        if self._is_big:
            fmt = '>'
        else:
            fmt = '<'
        for i in range(0, idx):
            fmt = fmt + self._fields_desc[i][1]
        return struct.calcsize(fmt)

    def getsize(self):
        return struct.calcsize(self._format)


class FWRTSSError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        if type(self.value) == str:
            return self.value
        else:
            return repr(self.value)




class FWRTSSReader:

    def __init__(self, port):

        try:
            import termios
            fd = open(port)
            tmp = termios.tcgetattr(fd.fileno())
            termios.tcsetattr(fd.fileno(), termios.TCSADRAIN, tmp)
            fd.close()
        except (ImportError, IOError, termios.error):
            pass

        self._msg = FWRTSSMessage()

        self._ser = None

        self._buf = []

        self._crc = crcmod.mkCrcFun(0x104c11db7, initCrc=0xFFFFFFFF, xorOut=0, rev=False)

        try:
            self._ser = serial.Serial(port, 115200)
            self._ser.flush()
            self._ser.flushInput()
            self._ser.flushOutput()
            self._ser.timeout = 0.2
        except serial.serialutil.SerialException, e:
            raise FWRTSSError(str(e))

    def __del__(self):
        if self._ser:
            self._ser.close()


    def read(self):

        def _swap(buf):
            return ''.join([buf[i+3]+buf[i+2]+buf[i+1]+buf[i] for i in range(0, len(buf), 4)])

        def _checkcrc32(buf, crc):
            val = self._crc(_swap(buf[:-4]))
            return ctypes.c_uint32(val).value == ctypes.c_uint32(crc).value

        size = self._msg.getsize()

        try:
            self._buf += self._ser.read()
        except serial.serialutil.SerialException:
            time.sleep(0.1)

        if not self._buf or len(self._buf) < size:
            return None

        sbuf = ''.join(self._buf[0:size])
        self._msg.setbuf(sbuf)

        msg_id  = self._msg.getfield('id')
        msg_crc = self._msg.getfield('crc')

        while msg_id != 0xAA or not _checkcrc32(self._buf[0:size], msg_crc):
            if len(self._buf) <= size:
                return None
            self._buf = self._buf[1:]
            sbuf = ''.join(self._buf[0:size])
            self._msg.setbuf(sbuf)
            msg_id  = self._msg.getfield('id')
            msg_crc = self._msg.getfield('crc')

        self._buf = self._buf[size:]

        fields = self._msg.getfields()

        return fields



class EbdData:

    def __init__(self, data):
        fd, fname = tempfile.mkstemp()
        gzd = GzipFile(mode='r', fileobj=StringIO(b64decode(data)))
        os.write(fd, gzd.read())
        os.close(fd)
        gzd.close()
        self.name = fname

    def __del__(self):
        os.unlink(self.name)

    def __str__(self):
        return self.name


if __name__ == '__main__':

    import argparse
    import sys
    parser = argparse.ArgumentParser(description=DESCRIPTION, version=VERSION)
    parser.add_argument('device', nargs=1,
                        type=argparse.FileType('rw'),
                        help='fwrtss serial device')
    parser.add_argument('-t',
                        type=int,
                        default=5,
                        dest='sec',
                        help='display depth in seconds')

    args = parser.parse_args()

    if args.sec <= 0:
        args.sec = 1

    device = args.device[0].name
    args.device[0].close()

    try:
        rd = FWRTSSReader(device)
    except FWRTSSError as e:
        sys.sterr.write("Serial error\n\r")
        sys.exit(1)

    icon = EbdData(ICON_DATA)

    app = QtGui.QApplication([])
    view = pg.GraphicsWindow(title=('fwrtss (%s)' % device))
    l = pg.GraphicsLayout(border=(100,100,100))
    view.setCentralItem(l)
    view.show()
    view.resize(800,800)
    view.setWindowIcon(QtGui.QIcon(str(icon)))

    datas = {}
    curve = {}
    label = {}
    texts = {}
    cp    = {}
    arrow = {}

    colors = {
        'tempdc' : (255,0,0),
        'target' : (0,255,0),
        'pwm_dc' : (0,255,255),
        'offset' : (20,100,255),
    }

    CURVE_A = 0.65

    l.addLabel('<b>FWRTSS PLOT ' + VERSION + ' by Stany MARCEL (%s)</b>' % device,
               col=0, row=0, colspan=2)

    l.nextRow()

    # == MODE LABEL ==
    label['pmode'] = l.addLabel('fwrtss mode : Unknown', angle=-90, col=0, row=1, rowspan=3)
    texts['pmode'] = {
        0 : 'fwrtss mode : Settings',
        1 : 'fwrtss mode : Run',
        2 : 'fwrtss mode : Backup',
        3 : 'fwrtss mode : Calibration',
    }

    msg = rd.read()
    while not msg:
        msg = rd.read()

    period = msg['period_ms'] / 1000.0
    x = np.arange(-(args.sec) + period, period, period)

    # == PLOT TEMPERATURES ==
    p1 = l.addPlot(name='Plot1', title='Temperatures', col=1, row=1)
    p1.addLegend()
    p1.setLabel('left', 'temperature', units='oC')
    p1.setLabel('bottom', 'time', units='s')
    p1.setRange(xRange=(-(args.sec),0), yRange=(0,500))
    p1.showGrid(x=False, y=True, alpha=0.5)
    datas['tempdc'] = [0] * int(args.sec / period)
    curve['tempdc'] = p1.plot(x=x, y=datas['tempdc'], pen=colors['tempdc'], name='curent')
    curve['tempdc'].setOpacity(CURVE_A)
    datas['target'] = [0] * int(args.sec / period)
    curve['target'] = p1.plot(x=x, y=datas['target'], pen=colors['target'], name='target')
    curve['target'].setOpacity(CURVE_A)
    cp['tempdc'] = pg.CurvePoint(curve['tempdc'])
    p1.addItem(cp['tempdc'])
    label['tempdc'] = pg.TextItem(text="", anchor=(1, 0.25), color=colors['tempdc'], angle=90)
    label['tempdc'].setParentItem(cp['tempdc'])

    cp['target'] = pg.CurvePoint(curve['target'])
    p1.addItem(cp['target'])
    label['target'] = pg.TextItem(text="", anchor=(1, -0.25), color=colors['target'], angle=90)
    label['target'].setParentItem(cp['target'])

    l.nextRow()

    # == PLOT DUTY CYCLE ==
    p2 = l.addPlot(name='Plot2', title='Duty cycle', col=1, row=2)
    p2.setXLink('Plot1')
    p2.setLabel('left', 'DC', units='%')
    p2.setLabel('bottom', 'time', units='s')
    p2.setRange(yRange=(0,100))
    p2.showGrid(x=False, y=True, alpha=0.5)

    datas['pwm_dc'] = [0] * int(args.sec / period)
    curve['pwm_dc'] = p2.plot(x=x, y=datas['pwm_dc'], pen=colors['pwm_dc'])
    curve['pwm_dc'].setOpacity(CURVE_A)

    cp['pwm_dc'] = pg.CurvePoint(curve['pwm_dc'])
    p2.addItem(cp['pwm_dc'])
    label['pwm_dc'] = pg.TextItem(text="", anchor=(0.9, 1), color=colors['pwm_dc'])
    label['pwm_dc'].setParentItem(cp['pwm_dc'])

    l.nextRow()

    # == PLOT TEMPERATURE OFFSET ==
    p3 = l.addPlot(name='Plot3', title='Temperature offset', col=1, row=3)
    p3.setXLink('Plot1')
    p3.setLabel('left', 'temperature', units='oC')
    p3.setLabel('bottom', 'time', units='s')
    p3.setRange(xRange=(-(args.sec),0), yRange=(-50,50))
    p3.showGrid(x=False, y=True, alpha=0.5)
    datas['offset'] = [0] * int(args.sec / period)
    curve['offset'] = p3.plot(x=x, y=datas['offset'], pen=colors['offset'], name='offset')
    curve['offset'].setOpacity(CURVE_A)

    cp['offset'] = pg.CurvePoint(curve['offset'])
    p3.addItem(cp['offset'])
    label['offset'] = pg.TextItem(text="", anchor=(0.9, 1), color=colors['offset'])
    label['offset'].setParentItem(cp['offset'])

    # == LAYOUT STRETCH ==
    l.layout.setRowStretchFactor(0, 0)
    l.layout.setRowStretchFactor(1, 6)
    l.layout.setRowStretchFactor(2, 4)
    l.layout.setRowStretchFactor(3, 1)

    # == UPDATE FUNC ==
    def update_curves():
        global datas, curve

        msg = rd.read()
        if not msg:
            return

        for k in datas.keys():
            datas[k] = datas[k][1:] + [ msg[k] ]
            curve[k].setData(x=x, y=datas[k])

        for k in cp.keys():
            cp[k].setPos(len(x))

        for k in label.keys():
            val = msg[k]
            if k in texts and val in texts[k]:
                label[k].setText('<b>' + texts[k][val] + '</b>')
            else:
                if type(val) == float:
                    label[k].setText(text='%.3f' % val, color=colors[k])
                else:
                    label[k].setText(text=str(val), color=colors[k])

        app.processEvents()

    timer = QtCore.QTimer()
    timer.timeout.connect(update_curves)
    timer.start(0)

    QtGui.QApplication.instance().exec_()
