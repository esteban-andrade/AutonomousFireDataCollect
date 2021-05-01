# -*- coding: utf-8 -*-
# This file is based on this gist:
# http://code.activestate.com/recipes/134892/
# So real authors are DannyYoo and company.
import sys

if sys.platform.startswith('linux') or sys.platform == 'darwin':
    # -*- coding: utf-8 -*-
    # Initially taken from:
    # http://code.activestate.com/recipes/134892/
    # Thanks to Danny Yoo
    import sys
    import tty
    import termios

    def readchar():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
elif sys.platform in ('win32', 'cygwin'):
    import msvcrt

    class key:
        # common
        LF = '\x0d'
        CR = '\x0a'
        ENTER = '\x0d'
        BACKSPACE = '\x7f'
        SUPR = ''
        SPACE = '\x20'
        ESC = '\x1b'

        # CTRL
        CTRL_A = '\x01'
        CTRL_B = '\x02'
        CTRL_C = '\x03'
        CTRL_D = '\x04'
        CTRL_E = '\x05'
        CTRL_F = '\x06'
        CTRL_Z = '\x1a'

        # ALT
        ALT_A = '\x1b\x61'

        # CTRL + ALT
        CTRL_ALT_A = '\x1b\x01'

        # cursors
        UP = '\x1b\x5b\x41'
        DOWN = '\x1b\x5b\x42'
        LEFT = '\x1b\x5b\x44'
        RIGHT = '\x1b\x5b\x43'

        CTRL_ALT_SUPR = '\x1b\x5b\x33\x5e'

        # other
        F1 = '\x1b\x4f\x50'
        F2 = '\x1b\x4f\x51'
        F3 = '\x1b\x4f\x52'
        F4 = '\x1b\x4f\x53'
        F5 = '\x1b\x4f\x31\x35\x7e'
        F6 = '\x1b\x4f\x31\x37\x7e'
        F7 = '\x1b\x4f\x31\x38\x7e'
        F8 = '\x1b\x4f\x31\x39\x7e'
        F9 = '\x1b\x4f\x32\x30\x7e'
        F10 = '\x1b\x4f\x32\x31\x7e'
        F11 = '\x1b\x4f\x32\x33\x7e'
        F12 = '\x1b\x4f\x32\x34\x7e'

        PAGE_UP = '\x1b\x5b\x35\x7e'
        PAGE_DOWN = '\x1b\x5b\x36\x7e'
        HOME = '\x1b\x5b\x48'
        END = '\x1b\x5b\x46'

        INSERT = '\x1b\x5b\x32\x7e'
        SUPR = '\x1b\x5b\x33\x7e'

        ESCAPE_SEQUENCES = (
            ESC,
            ESC + '\x5b',
            ESC + '\x5b' + '\x31',
            ESC + '\x5b' + '\x32',
            ESC + '\x5b' + '\x33',
            ESC + '\x5b' + '\x35',
            ESC + '\x5b' + '\x36',
            ESC + '\x5b' + '\x31' + '\x35',
            ESC + '\x5b' + '\x31' + '\x36',
            ESC + '\x5b' + '\x31' + '\x37',
            ESC + '\x5b' + '\x31' + '\x38',
            ESC + '\x5b' + '\x31' + '\x39',
            ESC + '\x5b' + '\x32' + '\x30',
            ESC + '\x5b' + '\x32' + '\x31',
            ESC + '\x5b' + '\x32' + '\x32',
            ESC + '\x5b' + '\x32' + '\x33',
            ESC + '\x5b' + '\x32' + '\x34',
            ESC + '\x4f',
            ESC + ESC,
            ESC + ESC + '\x5b',
            ESC + ESC + '\x5b' + '\x32',
            ESC + ESC + '\x5b' + '\x33',
        )

else:
    raise NotImplemented('The platform %s is not supported yet' % sys.platform)

if sys.platform in ('win32', 'cygwin'):
    #
    # Windows uses scan codes for extended characters. The ordinal returned is
    # 256 * the scan code.  This dictionary translates scan codes to the
    # unicode sequences expected by readkey.
    #
    # for windows scan codes see:
    #   https://msdn.microsoft.com/en-us/library/aa299374
    #      or
    #   http://www.quadibloc.com/comp/scan.htm
    xlate_dict = {
        13: key.ENTER,
        27: key.ESC,
        15104: key.F1,
        15360: key.F2,
        15616: key.F3,
        15872: key.F4,
        16128: key.F5,
        16384: key.F6,
        16640: key.F7,
        16896: key.F8,
        17152: key.F9,
        17408: key.F10,
        22272: key.F11,
        34528: key.F12,

        7680: key.ALT_A,

        # don't have table entries for...
        # CTRL_ALT_A, # Ctrl-Alt-A, etc.
        # CTRL_ALT_SUPR,
        # CTRL-F1

        21216: key.INSERT,
        21472: key.SUPR,    # key.py uses SUPR, not DELETE
        18912: key.PAGE_UP,
        20960: key.PAGE_DOWN,
        18400: key.HOME,
        20448: key.END,

        18656: key.UP,
        20704: key.DOWN,
        19424: key.LEFT,
        19936: key.RIGHT,
    }

    def readkey(getchar_fn = None, blocking: bool = True):
        # Get a single character on Windows. if an extended key is pressed, the
        # Windows scan code is translated into a the unicode sequences readchar
        # expects (see key.py).
        def _getchar():
            ch = msvcrt.getch()
            a = ord(ch)
            if a == 0 or a == 224:
                b = ord(msvcrt.getch())
                x = a + (b * 256)

                try:
                    return xlate_dict[x]
                except KeyError:
                    return None
                return x
            else:
                return ch.decode()

        if getchar_fn:
            raise NotImplementedError

        if blocking:
            while True:
                if msvcrt.kbhit():
                    return _getchar()
        else:
            if msvcrt.kbhit():
                return _getchar()
            else:
                return None

else:
    def readkey(getchar_fn = None, blocking: bool = True):
        if not blocking:
            raise NotImplementedError

        getchar = getchar_fn or readchar
        c1 = getchar()
        if ord(c1) != 0x1b:
            return c1
        c2 = getchar()
        if ord(c2) != 0x5b:
            return c1 + c2
        c3 = getchar()
        if ord(c3) != 0x33:
            return c1 + c2 + c3
        c4 = getchar()
        return c1 + c2 + c3 + c4
