# Author: Matteo Vanin
#parser of response message for a command type of message
import socket as sck   #for sockets
import sys      #for exit
import struct
import math
import time
import xml.dom.minidom as minidom
import signal

class Mocap(object):

    def __init__(self, host=None, port=None, info=0):
        #set IP and PORT of the Qtm PC
        if host is None:
            host = 'sml-qualisys.ddns.net'
        if port is None:
            port = 22224
        self.host = host
        self.port = port
        #create socket connection
        self.socket = self._create_connection(host,port,info)

    def _create_connection(self,host,port,printinfo):
        #create socket
        try:
            s = sck.socket(sck.AF_INET, sck.SOCK_STREAM)
        except sck.error,e:
            raise Exception('Failed to create mocap socket\nError' + str(e))

        if printinfo:
            print ('\nSocket Created \n')

        #create a socket connection
        s.settimeout(1.)
        try:
            s.connect((host , port))
        except sck.error,e:
            raise Exception('Failed to connect to mocap socket\nError' + str(e))


        if printinfo:
            print ('Socket Connected on host ' + host + ', port ' + str(port) + '\n')

        #Parse the WELCOME MESSAGE (always 35 Bytes)
        msg = _parser_comm(s)
        if printinfo:
            print('---Qualysis message:---')
            print(msg['message'] + '\n')

        # set the communication protocol version to 1.11
        str_to_send = 'Version 1.11'
        msg = self._build_packet(str_to_send,1);
        s.sendall(msg)
        # Parse the VERSION MESSAGE
        msg = _parser_comm(s)
        if printinfo:
            print('---Qualysis message:---')
            print(msg['message'] + '\n')
        return s

    def _clean_socket(self,s):
        try:
            try:
                while True:
                    msg_size_bytes = s.recv(1) #receive the size of the package
            except sck.timeout:
                pass
        except sck.error,e:
            raise Exception('Failed to clean mocap socket\nError' + str(e))

    def _build_packet(self,data,type):
        if sys.version_info > (2, 8):
            data_bytes = bytes(data,'UTF-8')
        else:
            data_bytes = bytearray(data,'UTF-8')

        data_len = len(data_bytes)
        packet_size = data_len + 9 #message size plus 8B of header and 1B of \x00 trailer
        header_size = struct.pack('>l',packet_size)
        header_type = struct.pack('>l',type)
        msg_to_send = header_size + header_type + data_bytes + b'\x00'
        return msg_to_send

    def _send_command(self,command):
        msg = self._build_packet(command,1)
        self.socket.sendall(msg)
        # if command == 'Close':
        #     #self.socket.close()
        #     return
        # else:
        return _parser_comm(self.socket)

    def _start_measurement(self):
        reply = self._send_command('New')
        if reply['message'] == 'You must be master to issue this command\x00':
            reply = self._send_command('TakeControl sml')
            if reply['message'] == 'You are now master\x00':
                reply = self._send_command('New')
            else:
                raise Exception('Warning: TakeControl sml fail\nReply: ' + reply['message'] +"\nEverything might works anyway")
        else:
            raise Exception('Warning: New fail\nReply: ' + reply['message'] +"\nEverything might works anyway")


    def _stop_measurement(self):
        self._send_command('Close')
        #self._send_command('ReleaseControl')
        #print reply

    def close(self):
        self.socket.close()

    def ask_for_6DOFinfo(self):
        str_to_send = 'GetCurrentFrame 6DEuler'
        msg = self._build_packet(str_to_send,1)
        try:
            self.socket.sendall(msg)
            return True
        except sck.error,e:
            raise Exception('Communication error\nError:' + str(e))

    def ask_for_3Dinfo(self):
        str_to_send = 'GetCurrentFrame 3D'
        msg = self._build_packet(str_to_send,1)
        try:
            self.socket.sendall(msg)
            return True
        except sck.error,e:
            raise Exception('Communication error\nError:' + str(e))

    def ask_for_6DOFinfoStream(self,frequency):
        if frequency == -1:
            str_to_send = 'StreamFrames AllFrames 6DEuler'
        else:
            str_to_send = 'StreamFrames Frequency:50 6DEuler'
        msg = self._build_packet(str_to_send,1)
        try:
            self.socket.sendall(msg)
            return True
        except sck.error,e:
            raise Exception('Communication error\nError:' + str(e))

    def ask_for_6DOFinfoStream_UDP(self,frequency):
        port = 5000
        socket = self.connect_to_UDP(port)
        if frequency == -1:
            str_to_send = 'StreamFrames AllFrames UDP:'+str(port)+' 6DEuler'
        else:
            str_to_send = 'StreamFrames Frequency:'+str(frequency)+' UDP:'+str(port)+' 6DEuler'
        msg = self._build_packet(str_to_send,1)
        try:
            self.socket.sendall(msg)
        except sck.error,e:
            raise Exception('Communication error\nError:' + str(e))
        return socket

    def connect_to_UDP(self,port):
        socket = sck.socket(sck.AF_INET, sck.SOCK_DGRAM)
        socket.bind(("", port))
        socket.settimeout(1.)
        return socket

    def stop_Streaminfo(self):
        str_to_send = 'StreamFrames Stop'
        msg = self._build_packet(str_to_send,1)
        try:
            self.socket.sendall(msg)
            return True
        except sck.error,e:
            raise Exception('Communication error\nError:' + str(e))

    def find_available_bodies(self, printinfo=True):
        if self.ask_for_6DOFinfo() == None:
            return None
        msg = _parser_comm(self.socket)
        if msg == None:
            return None
        valid = []
        #print msg['type']
        if msg['type']=='No more data' or msg['type']=='Event':
            return valid
            # raise Exception('No more data available. Check if the QTM server is running.')
        b = msg['bodies']

        if printinfo:
            print ('---Valid bodies in the workspace:---')
        for ii in range(len(b)): #foreach body in the configuration file, check if the body is in the workspace
            if not math.isnan(sum(b[ii])):
                valid.append(ii+1)
                if printinfo:
                    print('body nr. '  + str(ii+1) +  ' is valid\nx= ' + str(b[ii][0]) + '\ny= ' + str(b[ii][1]) + '\n')
        if len(valid) == 0 and printinfo:
            print "There are no valid bodies in the workspace."
        return [valid,msg]

    def ask_for_param(self):
        str_to_send = 'GetParameters All'
        msg = self._build_packet(str_to_send,1)
        try:
            self.socket.sendall(msg)
            return True
        except sck.error,e:
            raise Exception('Failed to get parameters\nError' + str(e))

    def get_parameters(self, printinfo=True):
        if self.ask_for_param() == None:
            return None
        msg = _parser_comm(self.socket)
        if msg == None:
            return None
        valid = []
        #print msg['type']
        if msg['type']=='No more data' or msg['type']=='Event':
            return valid
            # raise Exception('No more data available. Check if the QTM server is running.')

        return msg

    def get_list_bodies(self):
        xml_data = self.get_parameters()
        xml_str = xml_data['message'][0:-1]

        xml_dom = minidom.parseString(xml_str)
        bb = xml_dom.getElementsByTagName("Body")

        names = []
        for b in bb:
            alist=b.getElementsByTagName('Name')
            for a in alist:
                name = a.childNodes[0].nodeValue
                names.append(name)

        return names

    def get_id_from_name(self,name):
        names = self.get_list_bodies()
        try:
            return names.index(name)+1
        except ValueError:
            raise NameError(name + " is not defined in Qualisys, not in "+str(names))
            return -1

    def get_updated_bodies(self):
        try:
            [valid_bodies,bodies_info] = self.find_available_bodies(printinfo=False)
        except sck.error,e:
            raise Exception('Qualisys connection down\nError: ' + str(e))
        if valid_bodies == None:
            return 'off'

        bodies_list = []
        for body in valid_bodies:
            # new_pose = Body(self,body,'g').getPose(bodies_info)
            new_pose = Body(self,body,bodytype='a').getPose(bodies_info)
            if new_pose == None or new_pose == 'off':
                return new_pose
            bodies_list.append(new_pose)
        return bodies_list

    def get_body(self,body_id):

        try:
            [valid_bodies,bodies_info] = self.find_available_bodies(printinfo=False)
        except sck.error,e:
            raise Exception('Qualisys connection down\nError: ' + str(e))

        # if body_id is part of found bodies print pose, otherwise return off
        if body_id in valid_bodies:
            return Body(self,body_id,bodytype='a').getPose(bodies_info)
        else:
            return 'off'




class Body(object):

    def __init__(self, mocap, bodynr, bodytype=None):
        self.mocap = mocap
        self.bodynr = bodynr
        self.bodytype = bodytype
        self.ready = True

        if bodytype in ["ground","g"]:
            self.dtype = "xya"
        elif bodytype in ["air","a"] or (bodytype is None):
            self.dtype = "xyza"
        else:
            raise Exception("Invalid body type: must be either 'ground' (or 'g') or 'air' (or 'a')")

    def __repr__(self):
        dof = self.getPose()
        if self.dtype == "xya":
            return "Body nr {0} (ground vehicle) is at x = {1} mm, y = {2} mm, with yaw = {3} degrees".format(self.bodynr,dof['x'],dof['y'],dof['yaw'])
        elif self.dtype == "xyza":
            return "Body nr {0} (air vehicle) is at x = {1} mm, y = {2} mm, z = {3} mm, with angles of {4}, {5} and {6} degrees".format(self.bodynr,dof['x'],dof['y'],dof['z'],dof['a1'],dof['a2'],dof['a3'])

    def getPose(self,msg=None):
        socket = self.mocap.socket
        datatype = self.dtype
        if msg == None:
            self.mocap.ask_for_6DOFinfo()
            msg = _parser_comm(socket)
            if msg == None:
                return 'off'
        bodies=msg['bodies']
        timestamp=msg['timestamp']
        try:
            mybody = bodies[self.bodynr-1]
        except sck.error:
            return None
        x = mybody[0]/1000.
        y = mybody[1]/1000.
        z = mybody[2]/1000.
        roll = mybody[3]
        pitch = mybody[4]
        yaw = mybody[5]
        if datatype == 'xy':
            dof = {'x':x,'y':y}
        elif datatype == 'xya':
            dof = {'x':x,'y':y,'yaw':yaw}
        elif datatype == 'xyz':
            dof = {'x':x,'y':y,'z':z}
        elif datatype == 'xyza':
            dof = {'x':x,'y':y,'z':z,'roll':roll,'pitch':pitch,'yaw':yaw}
        else:
            raise Exception('Invalid data type request')
        dof['ts']=timestamp
        dof['id']=self.bodynr
        return dof

def recv_basic(socket,size):
    total_data=[]
    s = 0
    while s<size:
        try:
            data = socket.recv(size-s)
        except sck.timeout,e:
            print "wait data"
        if not data: break
        total_data.append(data)
        s += len(data)
    return ''.join(total_data),s

def _parser_comm(socket):
    msg = {'size':None, 'type':None, 'message':None, 'bodies':None, 'timestamp':None}
    rcvd_size = 0
    try:
        msg_size_bytes = socket.recv(4) #receive the size of the package
        rcvd_size += 4
    except sck.error,e:
        raise Exception('Qualisys connection down\nError: ' + str(e))

    try:
        msg_size = struct.unpack('>l', msg_size_bytes)[0] #get the decimal message size
    except sck.error,e:
        raise Exception('Qualisys connection down\nError: ' + str(e))

    msg['size']=msg_size

    msg_type_bytes = socket.recv(4) #receive the type of message
    rcvd_size += 4

    msg_type_code = struct.unpack('>l', msg_type_bytes)[0] #get the message type code
    msg_types = ['Error','Command','XML','Data','No more data','C3D file','Event']

    try:
        msg_type = msg_types[msg_type_code]
    except ValueError:
        print "msg_type_code: " + str(msg_type_code)
        raise Exception('unexpexted type of message, see protocol documentation')

    msg['type']=msg_type
    if msg_type_code == 3:
        # parse the rest of the header
        header_timestamp = socket.recv(8)
        rcvd_size += 8

        timestamp = struct.unpack('>q', header_timestamp)[0]
        header_framenumber = socket.recv(4)
        rcvd_size += 4

        #print(struct.unpack('>l', header_framenumber)[0] )
        header_componentcount = socket.recv(4)
        rcvd_size += 4

        nr_componentcount = struct.unpack('>l', header_componentcount)[0]
        for ii in range(nr_componentcount): #only one iteration if you request the current frame
            component_size = socket.recv(4)
            rcvd_size += 4

            nr_comp_size = struct.unpack('>l', component_size)[0]
            component_type = socket.recv(4)
            rcvd_size += 4

            nr_comp_type = struct.unpack('>l', component_type)[0]
            if nr_comp_type != 6:
                raise Exception('requested data type not manageable by the parser')
            body_count = struct.unpack('>l',socket.recv(4))[0]
            rcvd_size += 4

            useless_info = socket.recv(4)
            rcvd_size += 4

            bodies = [0]*body_count
            for jj in range(body_count):
                x = struct.unpack('>f',socket.recv(4))[0]
                y = struct.unpack('>f',socket.recv(4))[0]
                z = struct.unpack('>f',socket.recv(4))[0]
                rcvd_size += 12

                a1 = struct.unpack('>f',socket.recv(4))[0]
                a2 = struct.unpack('>f',socket.recv(4))[0]
                a3 = struct.unpack('>f',socket.recv(4))[0]
                rcvd_size += 12

                bodies[jj] = [x, y, z, a1, a2, a3]
            msg['bodies']=bodies
            msg['timestamp']=timestamp

    elif msg_type_code!=4:

        qtm_message_bytes,r = recv_basic(socket,msg_size-8) #receive the message (size+type are 8B)
        rcvd_size += r
        qtm_message = qtm_message_bytes.decode("UTF-8")
        msg['message']=qtm_message

    if rcvd_size!=msg_size:
        print msg_size-rcvd_size," bytes not received"
    return msg
