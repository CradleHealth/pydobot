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

    _SYNC = b'\xAA\xAA'

    def _read_exactly(self, n, deadline):
        """Read exactly n bytes from serial or return None on timeout."""
        buf = bytearray()
        while len(buf) < n:
            if time.monotonic() > deadline:
                return None
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                # keep looping until deadline
                continue
            buf += chunk
        return bytes(buf)

    def _read_frame_raw(self, overall_timeout=1.0):
        """
        Read one Dobot frame and return the raw bytes (including header and checksum),
        or None on timeout/invalid frame.
        Frame format:
          [0xAA][0xAA][LEN][ID][CTRL][PAYLOAD ...][CHK]
        CHK = (sum of all bytes except checksum) & 0xFF
        """
        deadline = time.monotonic() + overall_timeout

        # 1) Sync on 0xAA 0xAA
        sync = b""
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            sync += b
            if len(sync) > 2:
                sync = sync[-2:]
            if sync == self._SYNC:
                break
        else:
            if self.verbose:
                print('pydobot: !! timeout waiting for frame header (0xAA 0xAA)')
            return None  # header not found

        # 2) Read LEN
        len_b = self._read_exactly(1, deadline)
        if len_b is None:
            return None
        length = len_b[0]
        if length < 2:
            if self.verbose:
                print(f'pydobot: !! invalid length {length} (must be >= 2 for ID+CTRL)')
            return None
        # 3) Read BODY (ID + CTRL + PAYLOAD)
        body = self._read_exactly(length, deadline)
        if body is None or len(body) != length:
            if self.verbose:
                print('pydobot: !! timeout/short read on body')
            return None
        cmd_id = body[0]
        ctrl   = body[1]
        payload = body[2:] if length > 2 else b''
        # 4) Read checksum
        chk_b = self._read_exactly(1, deadline)
        if chk_b is None:
            if self.verbose:
                print('pydobot: !! timeout reading checksum')
            return None
        checksum = chk_b[0]
        # 5) Verify checksum. Support additive and LRC (two's complement) styles, with/without LEN.
        sum_body      = (sum(body)) & 0xFF
        sum_len_body  = (length + sum(body)) & 0xFF
        valid = (
            checksum == sum_body or
            checksum == sum_len_body or
            ((sum_body + checksum) & 0xFF) == 0x00 or
            ((sum_len_body + checksum) & 0xFF) == 0x00
        )
        if not valid:
            if self.verbose:
                full = self._SYNC + bytes([length]) + body + bytes([checksum])
                def hx(b): return ' '.join(f'{x:02X}' for x in b)
                calc_lrc_body     = ((-sum_body) & 0xFF)
                calc_lrc_lenbody  = ((-sum_len_body) & 0xFF)
                print('pydobot: !! checksum mismatch')
                print('         raw:', hx(full))
                print(f'         len={length} id=0x{cmd_id:02X} ctrl=0x{ctrl:02X} '
                      f'sum_body=0x{sum_body:02X} sum_len_body=0x{sum_len_body:02X} '
                      f'lrc_body=0x{calc_lrc_body:02X} lrc_lenbody=0x{calc_lrc_lenbody:02X} got=0x{checksum:02X}')
            return None
        return self._SYNC + bytes([length]) + body + bytes([checksum])

    def __init__(self, port, verbose=False):
        threading.Thread.__init__(self)

        self._on = True
        self.verbose = verbose
        self.lock = threading.Lock()
        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS,
                                 timeout=0.2,
                                 write_timeout=0.5)
        is_open = self.ser.isOpen()
        if self.verbose:
            print('pydobot: %s open' % self.ser.name if is_open else 'failed to open serial port')

        # Drain any stale bytes and give the controller a moment to settle
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass
        time.sleep(0.2)

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
        # Use standard command path; lock is not held during the execution wait loop
        msg = Message()
        msg.id = CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
        msg.ctrl = ControlValues.ZERO  # immediate GET, not queued
        resp = self._send_command(msg)  # wait=False by default
        if resp is None or resp.id != CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX:
            return None
        return struct.unpack_from('<I', resp.params, 0)[0]

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

    def _read_message(self, overall_timeout=3.0):
        deadline = time.monotonic() + overall_timeout
        while time.monotonic() < deadline:
            # Small per-iteration timeout keeps things responsive and tolerates interleaving
            slice_timeout = max(0.05, min(0.5, deadline - time.monotonic()))
            raw = self._read_frame_raw(overall_timeout=slice_timeout)
            if raw is None:
                continue
            msg = Message(raw)
            if self.verbose:
                print('pydobot: <<', msg)
            return msg
        return None

    def _safe_get_current_index(self, overall_timeout=1.5):
        """Try to read the current queued index with its own deadline; return int or None on timeout."""
        deadline = time.monotonic() + overall_timeout
        try:
            msg = Message()
            msg.id = CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX
            # This request is small and should respond quickly
            self._send_message(msg)
            while time.monotonic() < deadline:
                raw = self._read_message(overall_timeout=0.2)
                if raw is None:
                    continue
                # Expect the reply for current index; if it's a different frame, keep looping
                if raw.id == CommunicationProtocolIDs.GET_QUEUED_CMD_CURRENT_INDEX:
                    try:
                        return struct.unpack_from('<I', raw.params, 0)[0]
                    except Exception:
                        return None
                # Otherwise ignore unrelated frames during resync
            return None
        except Exception:
            return None

    def _send_command(self, msg, wait=False):
        self.lock.acquire()
        before_idx = None
        if wait:
            try:
                before_idx = self._get_queued_cmd_current_index()
            except Exception:
                before_idx = None
        self._send_message(msg)

        # Heavier-load scenarios (JUMP_XYZ, long moves, etc.) can delay replies.
        # Allow more time when the caller asked to wait.
        reply_timeout = 4.0 if not wait else 15.0
        response = self._read_message(overall_timeout=reply_timeout)

        self.lock.release()

        if response is None:
            # Fallback: controller sometimes withholds the immediate ACK during heavy JUMP_XYZ moves.
            # If we had a 'before' index, poll until the index advances, treating that as acceptance.
            if wait and before_idx is not None:
                start_wait = time.monotonic()
                # Allow up to the same reply_timeout window to observe index advancement
                while time.monotonic() - start_wait < reply_timeout:
                    try:
                        cur = self._get_queued_cmd_current_index()
                        if cur is not None and cur > before_idx:
                            # Synthesize a minimal Message-like object so downstream logic stays unchanged
                            fake = Message()
                            fake.id = msg.id
                            fake.ctrl = msg.ctrl
                            fake.params = bytearray(struct.pack('<I', cur))
                            response = fake
                            break
                    except Exception:
                        pass
                    time.sleep(0.1)
            # If still no response after fallback, one last probe to help with diagnostics/resync
            if response is None:
                _ = self._safe_get_current_index(overall_timeout=1.0)
                raise TimeoutError("No valid response from Dobot (timeout/short frame)")

        if not wait:
            return response

        expected_idx = struct.unpack_from('<I', response.params, 0)[0]
        if self.verbose:
            print('pydobot: waiting for command', expected_idx)

        # Wait for execution with tiny debounce and clear logging
        exec_deadline = time.monotonic() + 30.0  # hard cap to avoid indefinite hangs
        seen_eq = 0  # consecutive reads equal to expected (for 'last executed' semantics)
        while time.monotonic() < exec_deadline:
            current_idx = self._get_queued_cmd_current_index()
            if current_idx is None:
                if self.verbose:
                    print('pydobot: exec wait — no index reply this cycle')
                time.sleep(0.05)
                continue
            if self.verbose:
                print(f'pydobot: exec wait current={current_idx} expected={expected_idx}')
            # Case 1: firmware reports 'currently executing' -> completion when index advances past expected
            if current_idx > expected_idx:
                if self.verbose:
                    print('pydobot: command %d executed (current advanced to %d)' % (expected_idx, current_idx))
                break
            # Case 2: firmware reports 'last executed' -> completion when index equals expected (debounced)
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
            raise TimeoutError('Command queued but not observed as executed within deadline (expected %d, last %s)' %
                               (expected_idx, str(current_idx)))

        return response

    def _send_message(self, msg):
        if self.verbose:
            print('pydobot: >>', msg)
        self.ser.write(msg.bytes())
        try:
            self.ser.flush()  # ensure bytes leave the OS buffer promptly
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
