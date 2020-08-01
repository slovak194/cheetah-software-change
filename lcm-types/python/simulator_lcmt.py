"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class simulator_lcmt(object):
    __slots__ = ["vb", "rpy", "timesteps", "time", "quat", "R", "omegab", "omega", "p", "v", "vbd", "q", "qd", "qdd", "tau", "f_foot", "p_foot"]

    __typenames__ = ["double", "double", "int64_t", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"]

    __dimensions__ = [[3], [3], None, None, [4], [3, 3], [3], [3], [3], [3], [3], [4, 3], [4, 3], [4, 3], [4, 3], [4, 3], [4, 3]]

    def __init__(self):
        self.vb = [ 0.0 for dim0 in range(3) ]
        self.rpy = [ 0.0 for dim0 in range(3) ]
        self.timesteps = 0
        self.time = 0.0
        self.quat = [ 0.0 for dim0 in range(4) ]
        self.R = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(3) ]
        self.omegab = [ 0.0 for dim0 in range(3) ]
        self.omega = [ 0.0 for dim0 in range(3) ]
        self.p = [ 0.0 for dim0 in range(3) ]
        self.v = [ 0.0 for dim0 in range(3) ]
        self.vbd = [ 0.0 for dim0 in range(3) ]
        self.q = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(4) ]
        self.qd = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(4) ]
        self.qdd = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(4) ]
        self.tau = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(4) ]
        self.f_foot = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(4) ]
        self.p_foot = [ [ 0.0 for dim1 in range(3) ] for dim0 in range(4) ]

    def encode(self):
        buf = BytesIO()
        buf.write(simulator_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>3d', *self.vb[:3]))
        buf.write(struct.pack('>3d', *self.rpy[:3]))
        buf.write(struct.pack(">qd", self.timesteps, self.time))
        buf.write(struct.pack('>4d', *self.quat[:4]))
        for i0 in range(3):
            buf.write(struct.pack('>3d', *self.R[i0][:3]))
        buf.write(struct.pack('>3d', *self.omegab[:3]))
        buf.write(struct.pack('>3d', *self.omega[:3]))
        buf.write(struct.pack('>3d', *self.p[:3]))
        buf.write(struct.pack('>3d', *self.v[:3]))
        buf.write(struct.pack('>3d', *self.vbd[:3]))
        for i0 in range(4):
            buf.write(struct.pack('>3d', *self.q[i0][:3]))
        for i0 in range(4):
            buf.write(struct.pack('>3d', *self.qd[i0][:3]))
        for i0 in range(4):
            buf.write(struct.pack('>3d', *self.qdd[i0][:3]))
        for i0 in range(4):
            buf.write(struct.pack('>3d', *self.tau[i0][:3]))
        for i0 in range(4):
            buf.write(struct.pack('>3d', *self.f_foot[i0][:3]))
        for i0 in range(4):
            buf.write(struct.pack('>3d', *self.p_foot[i0][:3]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != simulator_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return simulator_lcmt._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = simulator_lcmt()
        self.vb = struct.unpack('>3d', buf.read(24))
        self.rpy = struct.unpack('>3d', buf.read(24))
        self.timesteps, self.time = struct.unpack(">qd", buf.read(16))
        self.quat = struct.unpack('>4d', buf.read(32))
        self.R = []
        for i0 in range(3):
            self.R.append(struct.unpack('>3d', buf.read(24)))
        self.omegab = struct.unpack('>3d', buf.read(24))
        self.omega = struct.unpack('>3d', buf.read(24))
        self.p = struct.unpack('>3d', buf.read(24))
        self.v = struct.unpack('>3d', buf.read(24))
        self.vbd = struct.unpack('>3d', buf.read(24))
        self.q = []
        for i0 in range(4):
            self.q.append(struct.unpack('>3d', buf.read(24)))
        self.qd = []
        for i0 in range(4):
            self.qd.append(struct.unpack('>3d', buf.read(24)))
        self.qdd = []
        for i0 in range(4):
            self.qdd.append(struct.unpack('>3d', buf.read(24)))
        self.tau = []
        for i0 in range(4):
            self.tau.append(struct.unpack('>3d', buf.read(24)))
        self.f_foot = []
        for i0 in range(4):
            self.f_foot.append(struct.unpack('>3d', buf.read(24)))
        self.p_foot = []
        for i0 in range(4):
            self.p_foot.append(struct.unpack('>3d', buf.read(24)))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if simulator_lcmt in parents: return 0
        tmphash = (0x49c5c4ff138274be) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if simulator_lcmt._packed_fingerprint is None:
            simulator_lcmt._packed_fingerprint = struct.pack(">Q", simulator_lcmt._get_hash_recursive([]))
        return simulator_lcmt._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

