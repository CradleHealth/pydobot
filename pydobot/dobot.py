import serial
import struct
import time
import threading
import warnings

from .enums.suction import Suction
from .message import Message
from .enums import PTPMode
from .enums.CommunicationProtocolIDs import CommunicationProtocolIDs
from .enums.ControlValues import ControlValues


class Dobot:

    def __init__(self, port, verbose=False):
        threading.Thread.__init__(self)

        self._on = True
        self.verbose = verbose
        self.lock = threading.Lock()
        self.ser = serial.Serial(
            port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.2,          # short, slice-based reads
            write_timeout=0.5     # avoid blocking on write
        )
        # streaming receive buffer for robust frame assembly
        self._rxbuf = bytearray()
        is_open = self.ser.isOpen()
        if self.verbose:
            print('pydobot: %s open' % self.ser.name if is_open else 'failed to open serial port')

        self._set_queued_cmd_clear()
        self._set_queued_cmd_start_exec()
        self._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
        self._set_ptp_coordinate_params(velocity=200, acceleration=200)
        self._set_ptp_jump_params(10, 200)
        self._set_ptp_common_params(velocity=100, acceleration=100)
        self._get_pose()

    """
        Gets the current command index
    """
    def _get_queued_cmd_current_index(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
        msg.ctrl = ControlValues.ZERO  # immediate GET
        response = self._send_command(msg)
        idx = struct.unpack_from('<Q', response.params, 0)[0]
        return idx

    """
        Gets the real-time pose of the Dobot
    """
    def _get_pose(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_POSE
        response = self._send_command(msg)
        self.x = struct.unpack_from('f', response.params, 0)[0]
        self.y = struct.unpack_from('f', response.params, 4)[0]
        self.z = struct.unpack_from('f', response.params, 8)[0]
        self.r = struct.unpack_from('f', response.params, 12)[0]
        self.j1 = struct.unpack_from('f', response.params, 16)[0]
        self.j2 = struct.unpack_from('f', response.params, 20)[0]
        self.j3 = struct.unpack_from('f', response.params, 24)[0]
        self.j4 = struct.unpack_from('f', response.params, 28)[0]

        if self.verbose:
            print("pydobot: x:%03.1f \
                            y:%03.1f \
                            z:%03.1f \
                            r:%03.1f \
                            j1:%03.1f \
                            j2:%03.1f \
                            j3:%03.1f \
                            j4:%03.1f" %
                  (self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4))
        return response

    def _read_message(self, overall_timeout=2.0):
        """Read exactly one framed message using AA AA header and checksums.
        Returns a Message or None on timeout."""
        deadline = time.time() + overall_timeout
        while time.time() < deadline:
            # Read at least one byte (serial timeout governs per-call wait)
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                self._rxbuf.extend(chunk)
            # Try to find a complete frame in the buffer
            while True:
                buf = self._rxbuf
                # need at least header + len + lenchk + tail chk
                if len(buf) < 5:
                    break
                # find sync 0xAA 0xAA
                try:
                    start = buf.index(0xAA)
                    # ensure second 0xAA follows
                    if start + 1 >= len(buf):
                        break
                    if buf[start + 1] != 0xAA:
                        # drop the first AA and continue searching
                        del buf[:start + 1]
                        continue
                except ValueError:
                    # no 0xAA found; drop all
                    self._rxbuf.clear()
                    break
                # ensure we have len and len checksum
                if start + 4 > len(buf):
                    break
                length = buf[start + 2]
                len_chk = buf[start + 3]
                if ((length + len_chk) & 0xFF) != 0:
                    # bad len checksum; drop first byte and resync
                    del buf[:start + 1]
                    continue
                total = 2 + 2 + length + 1  # hdr(2) + len+lenchk(2) + body(length) + bodychk(1)
                if start + total > len(buf):
                    # need more bytes
                    break
                frame = bytes(buf[start:start + total])
                # validate body checksum (two's complement)
                body_sum = sum(frame[2:2 + 2 + length]) & 0xFF
                body_chk = frame[-1]
                if ((body_sum + body_chk) & 0xFF) != 0:
                    # bad body checksum; drop first header byte and resync
                    del buf[:start + 1]
                    continue
                # we have a good frame; remove it from buffer and parse
                del buf[:start + total]
                try:
                    msg = Message(frame)
                except Exception:
                    # if parsing fails, continue searching
                    continue
                if self.verbose:
                    print('pydobot: <<', msg)
                return msg
            # no complete frame yet; loop until deadline
        return None

    def _send_command(self, msg, wait=False):
        self.lock.acquire()
        self._send_message(msg)
        # allow a reasonable window for the immediate ACK frame
        response = self._read_message(overall_timeout=3.0)
        self.lock.release()

        if not wait:
            return response

        expected_idx = struct.unpack_from('<Q', response.params, 0)[0]

        # Wait for execution to complete. Some firmware reports the executing index (done when current > expected),
        # others report the last executed index (done when current == expected). Add a hard cap to avoid hangs.
        deadline = time.time() + 30.0
        seen_eq = 0
        while time.time() < deadline:
            current_idx = self._get_queued_cmd_current_index()
            if current_idx is None:
                time.sleep(0.05)
                continue
            if self.verbose:
                print(f'pydobot: exec wait current={current_idx} expected={expected_idx}')
            if current_idx > expected_idx:
                if self.verbose:
                    print('pydobot: command %d executed (current advanced to %d)' % (expected_idx, current_idx))
                break
            if current_idx == expected_idx:
                seen_eq += 1
                if seen_eq >= 2:
                    if self.verbose:
                        print('pydobot: command %d executed (current==expected)' % expected_idx)
                    break
            else:
                seen_eq = 0
            time.sleep(0.05)
        else:
            raise TimeoutError('Command queued but not observed as executed within deadline (expected %d)' % expected_idx)

        return response

    def _send_message(self, msg):
        if self.verbose:
            print('pydobot: >>', msg)
        self.ser.write(msg.bytes())
        try:
            self.ser.flush()
        except Exception:
            pass

    """
        Executes the CP Command
    """
    def _set_cp_cmd(self, x, y, z):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_CP_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.append(0x00)
        return self._send_command(msg)

    """
        Sets the status of the gripper
    """
    def _set_end_effector_gripper(self, enable=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    """
        Sets the status of the suction cup (OLD)
    """
    def _set_end_effector_suction_cup(self, enable=False):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)


    def _set_end_effector_suction_cup_new(self, suction_state: Suction):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        if suction_state == Suction.SUCK:
            msg.params.extend(bytearray([0x01]))
            msg.params.extend(bytearray([0x00]))
        elif suction_state == Suction.RELEASE:
            msg.params.extend(bytearray([0x01]))
            msg.params.extend(bytearray([0x01]))
        elif suction_state == Suction.OFF:
            msg.params.extend(bytearray([0x00]))
            msg.params.extend(bytearray([0x00]))
        else:
            print("Unknown Gripper State")
            print("0: Open")
            print("1: Close")
            print("2: Dissable")
        return self._send_command(msg)

    """
        Sets the velocity ratio and the acceleration ratio in PTP mode
    """
    def _set_ptp_joint_params(self, v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', v_x)))
        msg.params.extend(bytearray(struct.pack('f', v_y)))
        msg.params.extend(bytearray(struct.pack('f', v_z)))
        msg.params.extend(bytearray(struct.pack('f', v_r)))
        msg.params.extend(bytearray(struct.pack('f', a_x)))
        msg.params.extend(bytearray(struct.pack('f', a_y)))
        msg.params.extend(bytearray(struct.pack('f', a_z)))
        msg.params.extend(bytearray(struct.pack('f', a_r)))
        return self._send_command(msg)

    """
        Sets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """
    def _set_ptp_coordinate_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
       Sets the lifting height and the maximum lifting height in JUMP mode
    """
    def _set_ptp_jump_params(self, jump, limit):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', jump)))
        msg.params.extend(bytearray(struct.pack('f', limit)))
        return self._send_command(msg)


    """
        Sets the velocity ratio, acceleration ratio in PTP mode
    """
    def _set_ptp_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
        Executes PTP command
    """
    def _set_ptp_cmd(self, x, y, z, r, mode, wait):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_PTP_CMD
        msg.ctrl = ControlValues.THREE
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode.value]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._send_command(msg, wait)

    """
        Clears command queue
    """
    def _set_queued_cmd_clear(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_CLEAR
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    """
        Start command
    """
    def _set_queued_cmd_start_exec(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_START_EXEC
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    """
        Wait command
    """
    def _set_wait_cmd(self, ms):
        msg = Message()
        msg.id = 110
        msg.ctrl = 0x03
        msg.params = bytearray(struct.pack('I', ms))
        return self._send_command(msg)

    """
        Stop command
    """
    def _set_queued_cmd_stop_exec(self):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_QUEUED_CMD_STOP_EXEC
        msg.ctrl = ControlValues.ONE
        return self._send_command(msg)

    def _get_eio_level(self, address):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_EIO
        msg.ctrl = ControlValues.ZERO
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        return self._send_command(msg)

    def _set_eio_level(self, address, level):
        msg = Message()
        msg.id = CommunicationProtocolIDs.SET_GET_EIO
        msg.ctrl = ControlValues.ONE
        msg.params = bytearray([])
        msg.params.extend(bytearray([address]))
        msg.params.extend(bytearray([level]))
        return self._send_command(msg)

    def get_eio(self, addr):
        return self._get_eio_level(addr)

    def set_eio(self, addr, val):
        return self._set_eio_level(addr, val)

    def close(self):
        self._on = False
        self.lock.acquire()
        self.ser.close()
        if self.verbose:
            print('pydobot: %s closed' % self.ser.name)
        self.lock.release()

    def go(self, x, y, z, r=0.):
        warnings.warn('go() is deprecated, use move_to() instead')
        self.move_to(x, y, z, r)

    def move_to(self, x, y, z, r, wait=False, mode=PTPMode.MOVL_XYZ):
        self._set_ptp_cmd(x, y, z, r, mode=mode, wait=wait)

    def suck(self, suction_state: Suction):
        self._set_end_effector_suction_cup_new(suction_state)

    def grip(self, enable):
        self._set_end_effector_gripper(enable)

    def speed(self, velocity=100., acceleration=100.):
        self._set_ptp_common_params(velocity, acceleration)
        self._set_ptp_coordinate_params(velocity, acceleration)

    def wait(self, ms):
        self._set_wait_cmd(ms)

    def pose(self):
        response = self._get_pose()
        x = struct.unpack_from('f', response.params, 0)[0]
        y = struct.unpack_from('f', response.params, 4)[0]
        z = struct.unpack_from('f', response.params, 8)[0]
        r = struct.unpack_from('f', response.params, 12)[0]
        j1 = struct.unpack_from('f', response.params, 16)[0]
        j2 = struct.unpack_from('f', response.params, 20)[0]
        j3 = struct.unpack_from('f', response.params, 24)[0]
        j4 = struct.unpack_from('f', response.params, 28)[0]
        return x, y, z, r, j1, j2, j3, j4
