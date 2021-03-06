#from __future__ import absolute_import, division, print_function
#import importlib
import threading
import serial

import numpy as np
import joblib

from pprzlink.message import PprzMessage
from pprzlink.pprz_transport import PprzTransport


class SerialMessagesInterface(threading.Thread):
    def __init__(self, callback, verbose=False, device='/dev/ttyUSB0', baudrate=115200,
                 msg_class='telemetry', interface_id=0):
        threading.Thread.__init__(self)
        self.callback = callback
        self.verbose = verbose
        self.msg_class = msg_class
        self.id = interface_id
        self.running = True
        try:
            self.ser = serial.Serial(device, baudrate, timeout=1.0)
        except serial.SerialException:
            print("Error: unable to open serial port '%s'" % device)
            exit(0)
        self.trans = PprzTransport(msg_class)

    def stop(self):
        print("End thread and close serial link")
        self.running = False
        self.ser.close()

    def shutdown(self):
        self.stop()

    def __del__(self):
        try:
            self.ser.close()
        except:
            pass

    def send(self, msg, sender_id,receiver_id = 0, component_id = 0):
        """ Send a message over a serial link"""
        if isinstance(msg, PprzMessage):
            data = self.trans.pack_pprz_msg(sender_id, msg, receiver_id, component_id)
            self.ser.write(data)
            self.ser.flush()

#    def collect_data(self,msg):
#        if msg.name == 'IMU_GYRO_SCALED':
#            print('GYRO received',msg.get_field(0), msg.get_field(1), msg.get_field(2) )
#        if msg.name == 'IMU_ACCEL_SCALED':
#            print('ACCEL received',msg.get_field(0), msg.get_field(1), msg.get_field(2) )

    def run(self):
        """Thread running function"""
        try:
            while self.running:
                # Parse incoming data
                c = self.ser.read(1)
                if len(c) == 1:
                    if self.trans.parse_byte(c):
                        (sender_id, receiver_id, component_id, msg) = self.trans.unpack()
                        #self.collect_data(msg)
                        if self.verbose:
                            print("New incoming message '%s' from %i (%i) to %i" % (msg.name, sender_id, component_id, receiver_id))
                        # Callback function on new message
                        if self.id == receiver_id:
                            self.callback(sender_id, msg)

        except StopIteration:
            pass

class Model():
    def __init__(self, saved_model_filename, saved_scaler_filename, verbose=False):
        self.model = joblib.load(saved_model_filename) # model can be saved with joblib.dump(clf,'fname.joblib')
        self.scaler = joblib.load(saved_scaler_filename)
        self.interface = None
        self.verbose = verbose
        self.fault_info  = 0
        self.fault_type = 0
        self.fault_class = 0

    def set_interface(self,interface):
        self.interface = interface

    def predict(self,X):
        #print('X in predict:',X, X.shape)
        X_scaled = self.scaler.transform(X.reshape(1,-1))
        #print('X scaled:',X_scaled, X_scaled.shape)
        self.fault_info = self.model.predict(X_scaled)
        self.send_pprz_fault_info()

    def send_pprz_fault_info(self):
        if self.verbose: print('Sending the FAULT_INFO')
        set_fault = PprzMessage('datalink', 'FAULT_INFO')
        set_fault['info'] = self.fault_info
        set_fault['type'] = 1
        set_fault['class']= 1
        self.interface.send(set_fault, 0)



class Data_Collector():
    def __init__(self, model=None, dimension=6, history_step=2, verbose=False):
        self.verbose = verbose
        self.model = model
        self.history_step = history_step
        self.dimension=dimension
        self.data = np.zeros([self.dimension*self.history_step])
        self.X = np.zeros([self.dimension])
        self.msg_received = np.zeros([2])

    def set_model(self, model):
        self.model = model

    def parse_msg(self, sender_id, msg):
        if msg.name == 'IMU_GYRO':
            if self.verbose: print('GYRO received',msg.get_field(0), msg.get_field(1), msg.get_field(2) )
            self.set_gyro_data(msg)
        if msg.name == 'IMU_ACCEL':
            if self.verbose: print('ACCEL received',msg.get_field(0), msg.get_field(1), msg.get_field(2) )
            self.set_accel_data(msg)

    def set_accel_data(self,msg):
        n = int(self.msg_received[0]*self.dimension)
        nn = int(n + self.dimension/2)
        #print('accel:',n,nn)
        self.data[n:nn] = msg.get_field(0), msg.get_field(1), msg.get_field(2) 
        self.msg_received[0] += 1
        self.check_data_ready()

    def set_gyro_data(self,msg):
        n = int(self.msg_received[0]*self.dimension + self.dimension/2)
        nn = int(n + self.dimension/2)
        #print('gyro:',n,nn)
        self.data[n:nn] = msg.get_field(0), msg.get_field(1), msg.get_field(2) 
        self.msg_received[1] += 1
        self.check_data_ready()

    def reset_msg(self):
        self.msg_received[:] = 0

    def send_data_to_model(self,X):
        if self.model != None :
            self.model.predict(X)
        else:
            print('No prediction model assigned !')
            pass

    def check_data_ready(self):
        if (self.msg_received[0]>=self.history_step) & (self.msg_received[1]>=self.history_step):
            self.X = self.data.copy()
            self.send_data_to_model(self.X)
            self.reset_msg()
            if self.verbose: print('X :',self.X)
            

def svm_callback(s,msg):
    pass
    # print("Here we are finally !")
    # try:
    # if msg.name == 'PPRZ_MODE':
    #     print(msg['ap_mode'])
    # print(msg.name)
    #if msg.name == 'IMU_GYRO_SCALED':
    #    print(msg.get_field(0), msg.get_field(1), msg.get_field(2) )#msg)
        # print(msg.get_field(1), msg.gq)
    # except:
    #     pass

#telemetry.PPRZ_MODE {ap_mode : 2, ap_gaz : 0, ap_lateral : 0, ap_horizontal : 0, if_calib_mode : 0, mcu1_status : 6, }
#telemetry.IMU_GYRO {gp : 0.00634765625, gq : -0.02392578125, gr : -0.00634765625, }
#telemetry.IMU_ACCEL {ax : 0.001953125, ay : -0.2919921875, az : -9.634765625, }


def main():
    '''
    run test program as a module to avoid namespace conflicts with serial module:
    
    python -m pprzlink.serial

    pprzlink should be installed in a python standard path or included to your PYTHONPATH
    '''
    import time
    import argparse
    from pprzlink import messages_xml_map
    import numpy as np

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", help="path to messages.xml file")
    parser.add_argument("-c", "--class", help="message class", dest='msg_class', default='telemetry')
    parser.add_argument("-d", "--device", help="device name", dest='dev', default='/dev/serial0')
    parser.add_argument("-b", "--baudrate", help="baudrate", dest='baud', default=115200, type=int)
    parser.add_argument("-id", "--ac_id", help="aircraft id (receiver)", dest='ac_id', default=42, type=int)
    parser.add_argument("--interface_id", help="interface id (sender)", dest='id', default=0, type=int)
    args = parser.parse_args()
    messages_xml_map.parse_messages(args.file)

    #model_filename = 'svm_model_1.joblib'
    #scaler_filename = 'svm_scaler_1.joblib'
    model_filename = 'svm_model_binary_r05.joblib'
    scaler_filename = 'svm_scaler_binary_r05.joblib'

    model = Model(model_filename, scaler_filename)
    collector = Data_Collector(model=model, dimension=6, history_step=10, verbose=False)
    serial_interface = SerialMessagesInterface(collector.parse_msg, device=args.dev,
                                               baudrate=args.baud, msg_class=args.msg_class, interface_id=args.id, verbose=False)
    model.set_interface(serial_interface)

    print("Starting serial interface on %s at %i baud" % (args.dev, args.baud))
    try:
        serial_interface.start()

        # give the thread some time to properly start
        time.sleep(0.1)

        # send some datalink messages to aicraft for test
        #ac_id = args.ac_id
        #print("sending ping")
        #ping = PprzMessage('datalink', 'PING')
        #serial_interface.send(ping, 0)

        #get_setting = PprzMessage('datalink', 'GET_SETTING')
        #get_setting['index'] = 0
        #get_setting['ac_id'] = ac_id
        #serial_interface.send(get_setting, 0)

        # change setting with index 0 (usually the telemetry mode)
        #print('Sending the FAULT_INFO')
        #set_fault = PprzMessage('datalink', 'FAULT_INFO')
        #set_fault['info'] = 2
        #set_fault['type'] = 1
        #set_fault['class'] = 1
        #serial_interface.send(set_fault, 0)

        # block = PprzMessage('datalink', 'BLOCK')
        # block['block_id'] = 3
        # block['ac_id'] = ac_id
        # serial_interface.send(block, 0)



        while serial_interface.isAlive():
            serial_interface.join(1)
    except (KeyboardInterrupt, SystemExit):
        print('Shutting down...')
        serial_interface.stop()
        exit()


if __name__ == '__main__':
    main()
