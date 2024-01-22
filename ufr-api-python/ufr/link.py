# =======================================================================================
#  Header
# =======================================================================================

import os
import ctypes
from pathlib import Path

# _base_path = str( Path(__file__).parent.resolve() )

# =======================================================================================
#  Link
# =======================================================================================

class Link(ctypes.Structure):
    dll = ctypes.CDLL(f"libufr.so")
    # dll.urf_sys_set_ld_path( bytes(_base_path, 'utf-8') );

    dll.lt_type.argtypes = [ ctypes.c_void_p ]
    dll.lt_type.restype =  ctypes.c_int32

    dll.lt_state.argtypes = [ ctypes.c_void_p ]
    dll.lt_state.restype =  ctypes.c_int32

    dll.lt_size.argtypes = [ ctypes.c_void_p ]
    dll.lt_size.restype =  ctypes.c_size_t

    dll.lt_size_max.argtypes = [ ctypes.c_void_p ]
    dll.lt_size_max.restype =  ctypes.c_size_t

    dll.lt_start.argtypes = [ ctypes.c_void_p ]
    dll.lt_start.restype = ctypes.c_int32

    dll.lt_start_publisher.argtypes = [ ctypes.c_void_p ]
    dll.lt_start.restype = ctypes.c_int32

    dll.lt_start_subscriber.argtypes = [ ctypes.c_void_p ]
    dll.lt_start.restype = ctypes.c_int32

    # dll.lt_stop.argtypes = [ ctypes.c_void_p ]
    # dll.lt_stop.restype = ctypes.c_int32

    dll.lt_close.argtypes = [ ctypes.c_void_p ]
    dll.lt_close.restype = ctypes.c_int32

    # dll.lt_puts.argtypes = [ ctypes.c_void_p, ctypes.c_char_p  ]
    # dll.lt_puts.restype = ctypes.c_int32

    dll.lt_api_name.argtypes = [ ctypes.c_void_p ]
    dll.lt_api_name.restype =  ctypes.c_char_p

    dll.lt_new_ptr.argtypes = [ ctypes.c_void_p, ctypes.c_char_p ]
    dll.lt_new_ptr.restype =  ctypes.c_int32

    _fields_ = [
        ('gtw_api', ctypes.c_void_p),
        ('gtw_obj', ctypes.c_void_p),
        ('gtw_pvt', ctypes.c_void_p),
        ('ecr_api', ctypes.c_void_p),
        ('ecr_obj', ctypes.c_void_p),
        ('dcr_api', ctypes.c_void_p),
        ('dcr_obj', ctypes.c_void_p)
    ]

    def __init__(self, text: str):
        Link.dll.lt_new_ptr( ctypes.pointer(self), bytes(text,'utf-8') )

    """
    def __init__(self, **args):
        text = ""
        for arg, val in args.items():
            text += f"@{arg} {val} "
        print(text)
        Link.dll.lt_new_ptr( ctypes.pointer(self), bytes(text,'utf-8') )
    """

    def __del__(self):
        # self.close()
        pass

    def lt_type_code(self):
        return Link.dll.lt_type( ctypes.pointer(self) )

    def lt_type(self):
        type_code = Link.dll.lt_type( ctypes.pointer(self) )
        if type_code == 0:
            return "Error"
        if type_code == 1:
            return "File"
        if type_code == 2:
            return "Dir"
        if type_code == 7:
            return "Socket"
        if type_code == 9:
            return "Factory"
        return "Invalid"

    def state(self):
        state_code = Link.dll.lt_state( ctypes.pointer(self) )
        if state_code == 0:
            return "Reset"
        if state_code == 1:
            return "Ready"
        if state_code == 2:
            return "Busy"
        if state_code == 3:
            return "Timeout"
        if state_code == 4:
            return "Error"
        if state_code == 5:
            return "Abort"
        return "Invalid"

    def start(self):
        error_code = Link.dll.lt_start( ctypes.pointer(self) )
        if error_code != 0:
            raise Exception("error no start")
    
    def start_publisher(self):
        error_code = Link.dll.urf_start_publisher( ctypes.pointer(self) )
        if error_code != 0:
            raise Exception("error no start")
        
    def start_subscriber(self):
        error_code = Link.dll.urf_start_subscriber( ctypes.pointer(self) )
        if error_code != 0:
            raise Exception("error no start")

    def stop(self):
        Link.dll.lt_stop( ctypes.pointer(self) )

    def close(self):
        Link.dll.lt_close( ctypes.pointer(self) )

    def __str__(self):
        api_name = Link.dll.lt_api_name( ctypes.pointer(self) ).decode('utf-8')
        return api_name

    def recv(self):
        Link.dll.lt_recv( ctypes.pointer(self) )

    def read(self):
        buffer = (ctypes.c_ubyte * 1024)()
        Link.dll.lt_read( ctypes.pointer(self), ctypes.pointer(buffer), 1024 )
        return bytes(buffer)

    def write(self, value):
        Link.dll.lt_write( ctypes.pointer(self), bytes(value, 'utf-8'), len(value) )

    def putln(self, *args):
        for arg in args:
            if type(arg) == int:
                Link.dll.lt_put( ctypes.pointer(self), bytes('i', 'utf-8'), arg )
            elif type(arg) == float:
                Link.dll.lt_put( ctypes.pointer(self), bytes('f', 'utf-8'), arg )
            elif type(arg) == str:
                Link.dll.lt_put( ctypes.pointer(self), bytes('s', 'utf-8'), bytes(arg, 'utf-8') )
            else:
                Exception(f"The variable {arg} is not allowed to serialize")
        Link.dll.lt_put( ctypes.pointer(self), bytes('\n', 'utf-8') )
        

    def get(self, format: str):
        resp = []
        for c in format:
            if c == 'i':
                var = ctypes.c_int32(0)
                Link.dll.lt_get(ctypes.pointer(self), bytes('i', 'utf-8'), ctypes.byref(var))
                resp.append(var.value)
            elif c == 'f':
                var = ctypes.c_float(0)
                Link.dll.lt_get(ctypes.pointer(self), bytes('f', 'utf-8'), ctypes.byref(var))
                resp.append(var.value)
            elif c == 's':
                buffer = (ctypes.c_ubyte * 1024)()
                Link.dll.lt_get(ctypes.pointer(self), bytes('s', 'utf-8'), ctypes.pointer(buffer))
                text = bytes(buffer).decode('utf-8').rstrip('\0')
                resp.append(text)
            elif c == '^':
                Link.dll.lt_get(ctypes.pointer(self), bytes('^', 'utf-8'))
        return resp


class CDev(Link):
    def read(self):
        buffer = (ctypes.c_ubyte * 256)()
        Link.dll.lt_read( ctypes.pointer(self), ctypes.pointer(buffer), 256 )
        return bytes(buffer).decode()

    def write(self, value):
        Link.dll.lt_write( ctypes.pointer(self), bytes(value, 'utf-8'), len(value) )


class File(CDev):
    def size(self):
        return Link.dll.lt_size( ctypes.pointer(self) )

    def seek(self):
        pass

    def __str__(self):
        api_name = Link.dll.lt_api_name( ctypes.pointer(self) ).decode('utf-8')

        data = {}
        data[".state"] = self.state()
        data['.size'] = self.size()

        return api_name+" "+str(data)
    
def Subscriber(text: str):
    link = Link(text)
    link.start_subscriber()
    return link

def Publisher(text: str):
    link = Link(text)
    link.start_publisher()
    return link

def urf_input(format: str):
    resp = []
    for c in format:
        if c == 'i':
            var = ctypes.c_int32(0)
            Link.dll.urf_input(bytes('i', 'utf-8'), ctypes.byref(var))
            resp.append(var.value)
        elif c == 'f':
            var = ctypes.c_float(0)
            Link.dll.urf_input(bytes('f', 'utf-8'), ctypes.byref(var))
            resp.append(var.value)
        elif c == 's':
            buffer = (ctypes.c_ubyte * 1024)()
            Link.dll.urf_input(bytes('s', 'utf-8'), ctypes.pointer(buffer))
            text = bytes(buffer).decode('utf-8').rstrip('\0')
            resp.append(text)
            # raise Exception("error")
        elif c == '^':
            Link.dll.urf_input(bytes('^', 'utf-8'))
    return resp

def urf_output(format: str, *args):
    c_args = []
    for i in range( len(format) ):
        c = format[i]
        if c == '\n':
            break
        elif c == 'i':
            c_args.append( ctypes.c_int32(args[i]) )
        elif c == 'f':
            c_args.append( ctypes.c_float(args[i]) )
        elif c == 's':
            c_args.append( bytes(args[i], 'utf-8') )
    Link.dll.urf_output( bytes(format, 'utf-8'), *c_args)

