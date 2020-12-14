from ppadb.client import Client as AdbClient
from ppadb.client import Client
from FPS_counter import FPSCounter
from install import get_device
import numpy as np

TAG = 'wE9ryARX5SVzGEER'

fps_counter = FPSCounter()


def mat2quat(mat):
    """ Convert Rotation Matrix to Quaternion.  See rotation.py for notes """
    mat = np.asarray(mat, dtype=np.float64)
    assert mat.shape[-2:] == (3, 3), "Invalid shape matrix {}".format(mat)

    Qxx, Qyx, Qzx = mat[..., 0, 0], mat[..., 0, 1], mat[..., 0, 2]
    Qxy, Qyy, Qzy = mat[..., 1, 0], mat[..., 1, 1], mat[..., 1, 2]
    Qxz, Qyz, Qzz = mat[..., 2, 0], mat[..., 2, 1], mat[..., 2, 2]
    # Fill only lower half of symmetric matrix
    K = np.zeros(mat.shape[:-2] + (4, 4), dtype=np.float64)
    K[..., 0, 0] = Qxx - Qyy - Qzz
    K[..., 1, 0] = Qyx + Qxy
    K[..., 1, 1] = Qyy - Qxx - Qzz
    K[..., 2, 0] = Qzx + Qxz
    K[..., 2, 1] = Qzy + Qyz
    K[..., 2, 2] = Qzz - Qxx - Qyy
    K[..., 3, 0] = Qyz - Qzy
    K[..., 3, 1] = Qzx - Qxz
    K[..., 3, 2] = Qxy - Qyx
    K[..., 3, 3] = Qxx + Qyy + Qzz
    K /= 3.0
    # TODO: vectorize this -- probably could be made faster
    q = np.empty(K.shape[:-2] + (4,))
    it = np.nditer(q[..., 0], flags=['multi_index'])
    while not it.finished:
        # Use Hermitian eigenvectors, values for speed
        vals, vecs = np.linalg.eigh(K[it.multi_index])
        # Select largest eigenvector, reorder to w,x,y,z quaternion
        q[it.multi_index] = vecs[[3, 0, 1, 2], np.argmax(vals)]
        # Prefer quaternion with positive w
        # (q * -1 corresponds to same rotation as q)
        if q[it.multi_index][0] < 0:
            q[it.multi_index] *= -1
        it.iternext()
    return q


def mat2trans(mat):
    return mat[:3, 3]


def parse_arrays(string):
    array_strings = string.split('|')
    arrays = []
    for array_string in array_strings:
        array = np.empty((4,4))
        values = array_string.split(' ')
        c = 0
        r = 0
        count = 0
        for value in values:
            if not value:
                continue
            array[r][c] = float(value)
            c += 1
            if c >= 4:
                c = 0
                r += 1
            count += 1
        if count == 16:
            arrays.append(array)
    return arrays

def get_data(text):
    output = ''
    for line in text.splitlines():
        if TAG in line:
            output += extract_data(line)
    return output

def extract_data(line):
    output = ''
    if TAG in line:
        output += line.split(TAG + ': ')[1]
    return output

def print_poses(data):
    arrays = parse_arrays(data)
    for i, array in enumerate(arrays):
        quat = mat2quat(array[:3, :3])
        trans = mat2trans(array)
        # print(i, ', quat:', quat, 'trans:', trans)

def dump_logcat_by_line(connection):
    file_obj = connection.socket.makefile()
    while True:
        try:
            line = file_obj.readline().strip()
            data = extract_data(line)
            if data:
                # print(data)
                print_poses(data)
                fps_counter.getAndPrintFPS()
        except UnicodeDecodeError:
            pass

    file_obj.close()
    connection.close()

device = get_device()
device.shell("logcat", handler=dump_logcat_by_line)

# Disconnect all devices
client.remote_disconnect()