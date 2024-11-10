# =======================================================================================
#  Header
# =======================================================================================

import os
import ctypes
from pathlib import Path
import numpy as np

# _base_path = str( Path(__file__).parent.resolve() )

UFR_OK = 0

UFR_START_BLANK=0
UFR_START_SERVER=1
UFR_START_CLIENT=3
UFR_START_PUBLISHER=4
UFR_START_SUBSCRIBER=5

# =======================================================================================
#  Link
# =======================================================================================

class Link(ctypes.Structure):
    dll = ctypes.CDLL(f"libufr.so")
    # dll.urf_sys_set_ld_path( bytes(_base_path, 'utf-8') );

    dll.ufr_link_with_type.argtypes = [ ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32 ]
    dll.ufr_link_with_type.restype = ctypes.c_int32

    dll.ufr_close.argtypes = [ ctypes.c_void_p ]
    dll.ufr_close.restype = ctypes.c_int32

    dll.ufr_link.argtypes = [ ctypes.c_void_p, ctypes.c_char_p ]
    dll.ufr_link.restype =  ctypes.c_int32

    _fields_ = [
        ('gtw_api', ctypes.c_void_p),
        ('gtw_shr', ctypes.c_void_p),
        ('gtw_obj', ctypes.c_void_p),
        ('enc_api', ctypes.c_void_p),
        ('enc_obj', ctypes.c_void_p),
        ('dcr_api', ctypes.c_void_p),
        ('dcr_obj', ctypes.c_void_p),

        ('dcr_api_s0', ctypes.c_void_p),
        ('dcr1_obj_s0', ctypes.c_void_p),

        ('type_started', ctypes.c_ubyte),
        ('log_level', ctypes.c_ubyte),
        ('log_ident', ctypes.c_ubyte),
        ('status', ctypes.c_ubyte),
        ('status2', ctypes.c_ubyte),

        ('put_count', ctypes.c_ushort),

        ('slot_gtw', ctypes.c_ubyte),
        ('slot_ecr', ctypes.c_ubyte),
        ('slot_dcr', ctypes.c_ubyte),

        ('errstr', ctypes.c_ubyte * 180)
    ]

    def __init__(self, text: str, type: int):
        error = Link.dll.ufr_link_with_type( ctypes.pointer(self), bytes(text,'utf-8'), type )
        if error != UFR_OK:
            error_msg = bytes(self.errstr).decode('utf-8').rstrip('\0')
            raise Exception(error_msg)

    def __del__(self):
        # self.close()
        pass

    def close(self):
        Link.dll.ufr_close( ctypes.pointer(self) )

    def __str__(self):
        api_name = Link.dll.ufr_api_name( ctypes.pointer(self) ).decode('utf-8')
        return api_name

    def recv(self):
        Link.dll.ufr_recv( ctypes.pointer(self) )

    def read(self):
        size = Link.dll.ufr_size_bytes( ctypes.pointer(self) )
        buffer = ( ctypes.c_ubyte * size )()
        total = Link.dll.ufr_read( ctypes.pointer(self), ctypes.pointer(buffer), size )
        # return bytes(buffer)
        # print(total)

        res = np.zeros(total, dtype=np.ubyte)
        for i in range(total):
            res[i] = buffer[i]
        #     print( hex(res[i]), end=' ')
        # print()
        # print(res.shape)
        # np.resize( res, (total,) )
        return res

    def is_error(self):
        return Link.dll.ufr_link_is_error( ctypes.pointer(self) )
        

    def write(self, value):
        Link.dll.ufr_write( ctypes.pointer(self), bytes(value, 'utf-8'), len(value) )

    def put(self, format, *args):
        index = 0
        for c in format:
            if c == '\n':
                Link.dll.ufr_put( ctypes.pointer(self), bytes('\n', 'utf-8') )
                continue

            elif c == 'i':
                arg = args[index]
                value = ctypes.c_int64(arg)
                Link.dll.ufr_put (
                    ctypes.pointer(self),
                    bytes('i', 'utf-8'),
                    value
                )

            elif c == 'f':
                arg = args[index]
                value = ctypes.c_float(arg)
                Link.dll.ufr_put ( 
                    ctypes.pointer(self),
                    bytes('f', 'utf-8'),
                    value 
                )

            elif c == 's':
                arg = args[index]
                Link.dll.ufr_put ( 
                    ctypes.pointer(self),
                    bytes('s', 'utf-8'),
                    bytes(arg, 'utf-8')
                )

            else:
                Exception(f"The variable {arg} is not allowed to serialize")
            index += 1


    def putln(self, *args):
        for arg in args:
            if type(arg) == int:
                value = ctypes.c_int64(arg)
                Link.dll.ufr_put (
                    ctypes.pointer(self), 
                    bytes('i', 'utf-8'), 
                    value
                )
            elif type(arg) == float:
                value = ctypes.c_float(arg)
                Link.dll.ufr_put ( 
                    ctypes.pointer(self), 
                    bytes('f', 'utf-8'), 
                    value 
                )
            elif type(arg) == str:
                Link.dll.ufr_put ( 
                    ctypes.pointer(self), 
                    bytes('s', 'utf-8'), 
                    bytes(arg, 'utf-8') 
                )
            else:
                Exception(f"The variable {arg} is not allowed to serialize")
        Link.dll.ufr_put( ctypes.pointer(self), bytes('\n', 'utf-8') )
        

    def get(self, format: str):
        resp = []
        for c in format:
            if c == 'i':
                var = ctypes.c_int32(0)
                Link.dll.ufr_get(ctypes.pointer(self), bytes('i', 'utf-8'), ctypes.byref(var))
                resp.append(var.value)
            elif c == 'f':
                var = ctypes.c_float(0)
                Link.dll.ufr_get(ctypes.pointer(self), bytes('f', 'utf-8'), ctypes.byref(var))
                resp.append(var.value)
            elif c == 's':
                buffer = (ctypes.c_ubyte * 1024)()
                Link.dll.ufr_get(ctypes.pointer(self), bytes('s', 'utf-8'), ctypes.pointer(buffer))
                text = bytes(buffer).decode('utf-8').rstrip('\0')
                resp.append(text)
            elif c == '^':
                Link.dll.ufr_get(ctypes.pointer(self), bytes('^', 'utf-8'))
            elif c == '\n':
                Link.dll.ufr_get(ctypes.pointer(self), bytes('\n', 'utf-8'))
            else:
                Exception(f"The variable {c} is not allowed to unpack")
        # case just one, return scalar value
        if len(resp) == 1:
            return resp[0]
        else:
            return resp
    
def Subscriber(text: str):
    return Link(text, UFR_START_SUBSCRIBER)

def Publisher(text: str):
    return Link(text, UFR_START_PUBLISHER)

def Server(text: str):
    return Link(text, UFR_START_SERVER)

def Client(text: str):
    return Link(text, UFR_START_CLIENT)






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