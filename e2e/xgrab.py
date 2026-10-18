"""Grab a region of the X display (Xvfb) as a numpy array, without external
tools: libX11 via ctypes. Used by the layout stability test."""
import ctypes
import ctypes.util

import numpy as np


class _XImage(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int), ("height", ctypes.c_int), ("xoffset", ctypes.c_int),
        ("format", ctypes.c_int), ("data", ctypes.POINTER(ctypes.c_ubyte)),
        ("byte_order", ctypes.c_int), ("bitmap_unit", ctypes.c_int), ("bitmap_bit_order", ctypes.c_int),
        ("bitmap_pad", ctypes.c_int), ("depth", ctypes.c_int), ("bytes_per_line", ctypes.c_int),
        ("bits_per_pixel", ctypes.c_int),
    ]


class Grabber:
    def __init__(self):
        name = ctypes.util.find_library("X11") or "libX11.so.6"
        self.x = ctypes.CDLL(name)
        self.x.XOpenDisplay.restype = ctypes.c_void_p
        self.x.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x.XGetImage.restype = ctypes.POINTER(_XImage)
        self.x.XGetImage.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
                                     ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_int]
        self.x.XDestroyImage.argtypes = [ctypes.POINTER(_XImage)]
        self.d = self.x.XOpenDisplay(None)
        if not self.d:
            raise RuntimeError("no X display")
        self.root = self.x.XDefaultRootWindow(self.d)

    def gray(self, x, y, w, h):
        """Region as float32 gray image (h, w)."""
        img = self.x.XGetImage(self.d, self.root, x, y, w, h, 0xFFFFFFFF, 2)  # ZPixmap
        if not img:
            raise RuntimeError("XGetImage failed")
        im = img.contents
        n = im.bytes_per_line * im.height
        buf = np.ctypeslib.as_array(im.data, shape=(n,)).copy()
        bpl = im.bytes_per_line
        self.x.XDestroyImage(img)
        px = buf.reshape(h, bpl)[:, :w * 4].reshape(h, w, 4).astype(np.float32)
        return px[:, :, 0] * 0.114 + px[:, :, 1] * 0.587 + px[:, :, 2] * 0.299


def vertical_shift(ref, img, max_shift=40, margin=50):
    """Vertical offset of img against ref from the row profiles (pixels)."""
    a = ref.mean(axis=1)
    b = img.mean(axis=1)
    h = len(a)
    best = (None, 0)
    for s in range(-max_shift, max_shift + 1):
        e = float(np.mean((a[margin:h - margin] - b[margin + s:h - margin + s]) ** 2))
        if best[0] is None or e < best[0]:
            best = (e, s)
    return best[1]


def moved_rows(ref, img, thresh=12.0):
    """Rows whose mean differs from the reference: in a column with static
    labels only a moved layout changes anything."""
    d = np.abs(ref.mean(axis=1) - img.mean(axis=1))
    return int((d > thresh).sum())
