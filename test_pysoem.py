#!/usr/bin/env python3
import rospy
import pysoem
import threading
import time
import ctypes
from enum import Enum, IntEnum
from typing import NamedTuple, Any, List

IF_NAME = 'enx68e1dc123fd9'
TARGET_VELOCITY = 200_000  # [pulse/s]
ACCEL_TIME = 1.0  # [s]
CYCLE_TIME_NS = 1_000_000  # [ns]
GEAR_RATIO = 50
PULSE_PER_REV = 10_000 * GEAR_RATIO  # [pulse/rev]

COMMAND_FILTER_TIME = 10  # [ms]
SVE_RATE = 1000  # [0.1%]
SVE_POSITION_GAIN = 10
SVE_VELOCITY_GAIN = 20

POSITION_ERROR_LIMIT_DEG = 10.0  # [deg]
OVERLOAD_TIME_SEC = 1.0  # [s]
QUICK_STOP_TORQUE = 20.0  # [%]
QUICK_STOP_DECEL = 1_000  # [pulse/s^2]

CTYPE_SIZES = {
    ctypes.c_int8: 1,
    ctypes.c_uint8: 1,
    ctypes.c_int16: 2,
    ctypes.c_uint16: 2,
    ctypes.c_int32: 4,
    ctypes.c_uint32: 4,
}

CType = Any


def from_ctype(data, ctype):
    return ctype.from_buffer_copy(data).value


def parse_inputs(data: bytes, ctypes: List[CType]):
    start_index = 0
    values = []
    for ctype in ctypes:
        end_index = start_index + CTYPE_SIZES[ctype]
        value = from_ctype(data[start_index:end_index], ctype)
        start_index = end_index
        values.append(value)
    return values


def pulse_to_deg(pulse):
    return (360.0 * pulse / PULSE_PER_REV + 180.0) % 360.0 - 180.0


class OperationMode(IntEnum):
    NONE = 0
    PP = 1
    PV = 3
    HM = 6
    CSP = 8
    CSV = 9


class SM2_SyncType(IntEnum):
    FREE_RUN = 0
    EVENT_SYNC = 1
    DC = 2


class SM3_SyncType(IntEnum):
    FREE_RUN = 0
    DC = 2
    EVENT_SYNC = 0x22


class ControlWord(Enum):
    SHUTDOWN = 0b0110
    DISABLE_VOLTAGE = 0b0000
    QUICK_STOP = 0b1011
    SWITCH_ON_AND_DISABLE_OPERATION = 0b0111
    SWITCH_ON_AND_ENABLE_OPERATION = 0b1111


class CurrentControlMode(IntEnum):
    CCM_INPUT = 0
    CST = 1
    SVE = 2


class QuickStopOption(IntEnum):
    CURRENT_OFF = 0
    PROFILE_DECEL_DISABLED = 1
    QUICKSTOP_DECEL_DISABLED = 2
    IMMEDIATE_STOP_DISABLED = 3
    PROFILE_DECEL_ACTIVE = 5
    QUICKSTOP_DECEL_ACTIVE = 6
    IMMEDIATE_STOP_ACTIVE = 7


class Field(NamedTuple):
    index: int
    subindex: int
    ctype: CType


def format_status_word(status: int) -> str:
    return f'{status:016b}'


class EcMaster:
    def __init__(self, ifname: str):
        self._master = pysoem.Master()
        self._master.open(ifname)
        n_slaves = self._master.config_init()

        if n_slaves == 0:
            self._master.close()
            raise EcMasterError('No slave found')

        for i, slave in enumerate(self._master.slaves):
            print(f'Found Slave <{slave.name}>')
            if slave.name == 'AZD2A-KED':
                self.set_control_word(i, ControlWord.SHUTDOWN)
                self.set_sm2_sync_type(i, SM2_SyncType.DC)
                self.set_sm3_sync_type(i, SM3_SyncType.DC)
                slave.config_func = self.AZD2A_KED_setup
                slave.is_lost = False
                print('config_func is set')

        size_iomap = self._master.config_map()
        print(f'IO map size: {size_iomap}')

        if self._master.state_check(pysoem.SAFEOP_STATE, timeout=50_000) != pysoem.SAFEOP_STATE:
            print(f'state: {self._master.read_state()}')
            self._master.close()
            raise EcMasterError("Not all slaves reached SAFEOP state")

        self._in_op = False
        self._do_check_state = False
        self._n_slaves = n_slaves
        self._ifname = ifname
        self._pd_thread_stop_event = threading.Event()
        self._ch_thread_stop_event = threading.Event()
        self._actual_wkc = 0

    def __del__(self):
        self._master.close()

    def sdo_write(self, slave_index: int, field: Field, data):
        slave = self._master.slaves[slave_index]
        slave.sdo_write(index=field.index, subindex=field.subindex,
                        data=bytes(field.ctype(data)))

    def AZD2A_KED_setup(self, slave_index: int):
        """Config function that will be called when transitioning
        from PreOP state to SafeOPstate."""

        SM2_PDO_SIZE = Field(index=0x1C12, subindex=0, ctype=ctypes.c_uint8)
        SM3_PDO_SIZE = Field(index=0x1C13, subindex=0, ctype=ctypes.c_uint8)
        PDO_MAP_AXIS1_RX_1_SIZE = Field(index=0x1600, subindex=0, ctype=ctypes.c_uint8)
        PDO_MAP_AXIS1_RX_1_ASSIGN1 = Field(index=0x1600, subindex=1, ctype=ctypes.c_uint32)
        PDO_MAP_AXIS1_TX_1_SIZE = Field(index=0x1A00, subindex=0, ctype=ctypes.c_uint8)
        PDO_MAP_AXIS1_TX_1_ASSIGN1 = Field(index=0x1A00, subindex=1, ctype=ctypes.c_uint32)
        PDO_MAP_AXIS1_TX_1_ASSIGN2 = Field(index=0x1A00, subindex=2, ctype=ctypes.c_uint32)
        PDO_MAP_AXIS1_TX_1_ASSIGN3 = Field(index=0x1A00, subindex=3, ctype=ctypes.c_uint32)
        PDO_MAP_AXIS1_TX_1_ASSIGN4 = Field(index=0x1A00, subindex=4, ctype=ctypes.c_uint32)
        PDO_MAP_AXIS1_TX_1_ASSIGN5 = Field(index=0x1A00, subindex=5, ctype=ctypes.c_uint32)
        PDO_MAP_AXIS1_TX_1_ASSIGN6 = Field(index=0x1A00, subindex=6, ctype=ctypes.c_uint32)
        COMMAND_FILTER_TIME_AXIS1 = Field(index=0x412A, subindex=1, ctype=ctypes.c_int16)
        CURRENT_CONTROL_MODE_AXIS1 = Field(index=0x412D, subindex=1, ctype=ctypes.c_uint8)
        SVE_RATE_AXIS1 = Field(index=0x412E, subindex=1, ctype=ctypes.c_int16)
        SVE_POSITION_GAIN_AXIS1 = Field(index=0x412F, subindex=1, ctype=ctypes.c_int16)
        SVE_VELOCITY_GAIN_AXIS1 = Field(index=0x4130, subindex=1, ctype=ctypes.c_int16)

        TARGET_POSITION_FIELD = 0x607A_00_20
        # TARGET_VELOCITY_FIELD = 0x60FF_00_20
        STATUS_WORD_FIELD = 0x6041_00_10
        FEEDBACK_POSITION_FIELD = 0x6064_00_20
        FEEDBACK_VELOCITY_FIELD = 0x606C_00_20
        POSITION_ERROR_FIELD = 0x60F4_00_20
        TORQUE_MONITOR_FIELD = 0x406B_01_10
        CURRENT_ALARM_FIELD = 0x4040_01_10

        # ---- PDO map settings --------
        self.sdo_write(slave_index, SM2_PDO_SIZE, 0)
        self.sdo_write(slave_index, SM3_PDO_SIZE, 0)

        self.sdo_write(slave_index, PDO_MAP_AXIS1_RX_1_SIZE, 0)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_RX_1_ASSIGN1, TARGET_POSITION_FIELD)
        # self.sdo_write(slave_index, PDO_MAP_AXIS1_RX_1_ASSIGN1, TARGET_VELOCITY_FIELD)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_RX_1_SIZE, 1)

        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_SIZE, 0)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_ASSIGN1, FEEDBACK_POSITION_FIELD)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_ASSIGN2, FEEDBACK_VELOCITY_FIELD)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_ASSIGN3, POSITION_ERROR_FIELD)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_ASSIGN4, TORQUE_MONITOR_FIELD)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_ASSIGN5, CURRENT_ALARM_FIELD)
        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_ASSIGN6, STATUS_WORD_FIELD)
        self.input_types = [
            ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int16, ctypes.c_uint16,
            ctypes.c_uint16
        ]
        self.sdo_write(slave_index, PDO_MAP_AXIS1_TX_1_SIZE, len(self.input_types))

        self.sdo_write(slave_index, SM2_PDO_SIZE, 1)
        self.sdo_write(slave_index, SM3_PDO_SIZE, 1)
        # ------------------------------

        self.sdo_write(slave_index, COMMAND_FILTER_TIME_AXIS1, COMMAND_FILTER_TIME)
        self.sdo_write(slave_index, CURRENT_CONTROL_MODE_AXIS1, CurrentControlMode.SVE)
        self.sdo_write(slave_index, SVE_RATE_AXIS1, SVE_RATE)
        self.sdo_write(slave_index, SVE_POSITION_GAIN_AXIS1, SVE_POSITION_GAIN)
        self.sdo_write(slave_index, SVE_VELOCITY_GAIN_AXIS1, SVE_VELOCITY_GAIN)

        slave = self._master.slaves[slave_index]
        slave.dc_sync(act=True, sync0_cycle_time=CYCLE_TIME_NS)

    def _processdata_thread(self):
        """Background thread that sends and receives the process-data frame
        in a 10ms interval."""
        while not self._pd_thread_stop_event.is_set():
            self._master.send_processdata()
            self._actual_wkc = self._master.receive_processdata(timeout=100_000)
            if not self._actual_wkc == self._master.expected_wkc:
                print("Incorrect wkc. "
                      f"actual: {self._actual_wkc}, expected: {self._master.expected_wkc}")
            time.sleep(0.01)

    def _pdo_update_loop(self):
        """The actual application code used to toggle the digital output in an endless loop.
        Called when all slaves reached OP state.
        Updates the rx PDO every second.
        """
        self._in_op = True
        target_velocity = TARGET_VELOCITY
        target_acceleration = target_velocity / ACCEL_TIME
        target_position = self.get_feedback_position(0)  # [pulse]
        cycle_time = CYCLE_TIME_NS * 1e-9  # [s]
        rate = rospy.Rate(1.0 / cycle_time)
        pulse = 0
        accel = 0.0
        sign = 1
        i = 0
        i_accel = int(ACCEL_TIME / cycle_time)
        quick_stopped = False

        try:
            while not rospy.is_shutdown():
                i += 1
                if i % (i_accel * 4) == 0:
                    sign = 1
                    accel = 0
                    target_acceleration = target_velocity / ACCEL_TIME
                elif i % (i_accel * 4) == (i_accel):
                    sign = 0
                    accel = 0
                    target_acceleration = 0
                # elif i % (i_accel * 4) == (i_accel * 2):
                #     sign = -1
                #     accel = 0
                #     target_acceleration = target_velocity / ACCEL_TIME / 1
                elif i % (i_accel * 4) == (i_accel * 3):
                    sign = 0
                    accel = 0
                    target_acceleration = 0

                accel += target_acceleration * cycle_time ** 2
                if accel >= 1.0:
                    delta = int(accel)
                    accel -= int(accel)
                else:
                    delta = 0

                pulse = max(min(pulse + sign * delta, int(target_velocity * cycle_time)), 0)
                target_position += pulse
                self._master.slaves[0].output = bytes(ctypes.c_int32(target_position))
                time_now = rospy.Time.now().to_sec() * 1000
                feedback_position = from_ctype(self._master.slaves[0].input, ctypes.c_int32)
                feedback_position, feedback_velocity, error, torque, alarm, status = \
                    parse_inputs(self._master.slaves[0].input, self.input_types)
                error_deg = pulse_to_deg(error)
                print(f"[{i:5d}] t={time_now%1000:5.1f}ms, "
                      f"pulse={pulse:3d}, "
                      f"v={int(feedback_velocity / PULSE_PER_REV * 60):3d}rpm, "
                      f"err={error_deg:+6.2f}deg, "
                      f"T={torque / 10:+5.1f}%, "
                      f"alarm=0x{alarm:02x}, status={status:016b}")
                if not quick_stopped and abs(torque / 10) > QUICK_STOP_TORQUE:
                    master.set_control_word(0, ControlWord.QUICK_STOP)
                    print("Quick Stop!!!!")
                    quick_stopped = True
                rate.sleep()

        except KeyboardInterrupt:
            # ctrl-C abort handling
            print("stopped")

    def run(self):
        self._master.state = pysoem.OP_STATE

        check_thread = threading.Thread(target=self._check_thread)
        check_thread.start()
        proc_thread = threading.Thread(target=self._processdata_thread)
        proc_thread.start()

        # send one valid process data to make outputs in slaves happy
        self._master.send_processdata()
        self._master.receive_processdata(timeout=2000)
        # request OP state for all slaves

        self._master.write_state()

        all_slaves_reached_op_state = False
        for i in range(40):
            self._master.state_check(pysoem.OP_STATE, timeout=50_000)
            if self._master.state == pysoem.OP_STATE:
                all_slaves_reached_op_state = True
                break

        if all_slaves_reached_op_state:
            self._pdo_update_loop()

        # termination process
        self._pd_thread_stop_event.set()
        self._ch_thread_stop_event.set()
        proc_thread.join()
        check_thread.join()
        self._master.state = pysoem.INIT_STATE
        self._master.write_state()
        self._master.close()

        if not all_slaves_reached_op_state:
            raise EcMasterError("Not all slaves reached OP state")

    @staticmethod
    def _check_slave(slave, pos):
        if slave.state == (pysoem.SAFEOP_STATE + pysoem.STATE_ERROR):
            print(f"ERROR : slave {pos} is in SAFE_OP + ERROR, attempting ack.")
            slave.state = pysoem.SAFEOP_STATE + pysoem.STATE_ACK
            slave.write_state()
        elif slave.state == pysoem.SAFEOP_STATE:
            print(f"WARNING : slave {pos} is in SAFE_OP, try change to OPERATIONAL.")
            slave.state = pysoem.OP_STATE
            slave.write_state()
        elif slave.state > pysoem.NONE_STATE:
            if slave.reconfig():
                slave.is_lost = False
                print(f"MESSAGE : slave {pos} reconfigured")
        elif not slave.is_lost:
            slave.state_check(pysoem.OP_STATE)
            if slave.state == pysoem.NONE_STATE:
                slave.is_lost = True
                print(f"ERROR : slave {pos} lost")
        if slave.is_lost:
            if slave.state == pysoem.NONE_STATE:
                if slave.recover():
                    slave.is_lost = False
                    print(f"MESSAGE : slave {pos} recovered")
            else:
                slave.is_lost = False
                print(f"MESSAGE : slave {pos} found")

    def _check_thread(self):
        while not self._ch_thread_stop_event.is_set():
            wkc_ok = self._actual_wkc < self._master.expected_wkc
            if self._in_op and (wkc_ok or self._do_check_state):
                self._do_check_state = False
                self._master.read_state()
                for i, slave in enumerate(self._master.slaves):
                    if slave.state != pysoem.OP_STATE:
                        self._do_check_state = True
                        EcMaster._check_slave(slave, i)
                if not self._do_check_state:
                    print("OK : all slaves resumed OPERATIONAL.")
            time.sleep(0.01)

    def get_status_word(self, slave_index: int) -> int:
        slave = self._master.slaves[slave_index]
        return from_ctype(slave.sdo_read(index=0x6041, subindex=0), ctypes.c_uint16)

    def get_control_word(self, slave_index: int) -> int:
        slave = self._master.slaves[slave_index]
        return from_ctype(slave.sdo_read(index=0x6040, subindex=0), ctypes.c_uint16)

    def set_control_word(self, slave_index: int, data: ControlWord):
        slave = self._master.slaves[slave_index]
        slave.sdo_write(index=0x6040, subindex=0,
                        data=bytes(ctypes.c_uint16(data.value)))

    def get_operation_mode(self, slave_index: int) -> OperationMode:
        slave = self._master.slaves[slave_index]
        return OperationMode(
            from_ctype(slave.sdo_read(index=0x6060, subindex=0), ctypes.c_int8))

    def set_operation_mode(self, slave_index: int, data: OperationMode):
        slave = self._master.slaves[slave_index]
        slave.sdo_write(index=0x6060, subindex=0,
                        data=bytes(ctypes.c_int8(data.value)))

    def get_target_position(self, slave_index: int) -> int:
        slave = self._master.slaves[slave_index]
        return from_ctype(slave.sdo_read(index=0x6062, subindex=0), ctypes.c_int32)

    def set_target_position(self, slave_index: int, data: int):
        slave = self._master.slaves[slave_index]
        slave.sdo_write(index=0x607a, subindex=0,
                        data=bytes(ctypes.c_int32(data)))

    def get_feedback_position(self, slave_index: int) -> int:
        slave = self._master.slaves[slave_index]
        return from_ctype(slave.sdo_read(index=0x6064, subindex=0), ctypes.c_int32)

    def get_sm2_sync_type(self, slave_index: int) -> SM2_SyncType:
        slave = self._master.slaves[slave_index]
        return SM2_SyncType(
            from_ctype(slave.sdo_read(index=0x1c32, subindex=1), ctypes.c_uint16))

    def set_sm2_sync_type(self, slave_index: int, data: SM2_SyncType):
        slave = self._master.slaves[slave_index]
        slave.sdo_write(index=0x1c32, subindex=1,
                        data=bytes(ctypes.c_uint16(data.value)))

    def get_sm3_sync_type(self, slave_index: int) -> SM3_SyncType:
        slave = self._master.slaves[slave_index]
        return SM3_SyncType(
            from_ctype(slave.sdo_read(index=0x1c33, subindex=1), ctypes.c_uint16))

    def set_sm3_sync_type(self, slave_index: int, data: SM3_SyncType):
        slave = self._master.slaves[slave_index]
        slave.sdo_write(index=0x1c33, subindex=1,
                        data=bytes(ctypes.c_uint16(data.value)))

    def reset_alarm(self, slave_index: int):
        slave = self._master.slaves[slave_index]
        slave.sdo_write(index=0x40C0, subindex=1, data=bytes(ctypes.c_uint8(0)))
        time.sleep(0.5)
        slave.sdo_write(index=0x40C0, subindex=1, data=bytes(ctypes.c_uint8(1)))

    def set_brake_without_excitation(self, slave_index: int, enabled: False):
        slave = self._master.slaves[slave_index]
        if enabled:
            slave.sdo_write(index=0x413D, subindex=1, data=bytes(ctypes.c_uint8(0)))
        else:
            slave.sdo_write(index=0x413D, subindex=1, data=bytes(ctypes.c_uint8(1)))

    def set_alarm_conditions(self, slave_index: int,
                             position_error_limit_deg: float = 10.0,
                             overload_time_sec: float = 5.0,
                             quickstop_option=QuickStopOption.QUICKSTOP_DECEL_DISABLED,
                             quickstop_decel: int = 1_000_000,
                             ):
        slave = self._master.slaves[slave_index]
        position_error_limit = int(position_error_limit_deg * GEAR_RATIO / 360.0 * 100)  # [0.01 rev]
        overload_time = int(overload_time_sec * 10)  # [0.1 sec]
        print(f"position_error_limit={position_error_limit}")
        print(f"overload_time={overload_time}")
        print(f"quicksto_option={quickstop_option.name}")
        print(f"quicksto_decel={quickstop_decel}")
        slave.sdo_write(index=0x6065, subindex=0, data=bytes(ctypes.c_uint32(position_error_limit)))
        slave.sdo_write(index=0x4180, subindex=1, data=bytes(ctypes.c_int16(overload_time)))
        slave.sdo_write(index=0x605A, subindex=0, data=bytes(ctypes.c_int16(quickstop_option)))
        slave.sdo_write(index=0x6085, subindex=0, data=bytes(ctypes.c_uint32(quickstop_decel)))
        print(from_ctype(slave.sdo_read(index=0x6084, subindex=0), ctypes.c_uint32))


class EcMasterError(Exception):
    def __init__(self, message):
        super().__init__(message)


if __name__ == '__main__':
    rospy.init_node('test_pysoem')
    master = EcMaster(IF_NAME)
    slave_index = 0
    master.reset_alarm(slave_index)
    master.set_brake_without_excitation(slave_index, False)
    master.set_operation_mode(slave_index, OperationMode.CSP)
    master.set_alarm_conditions(slave_index, POSITION_ERROR_LIMIT_DEG, OVERLOAD_TIME_SEC,
                                QuickStopOption.QUICKSTOP_DECEL_DISABLED, QUICK_STOP_DECEL)
    master.set_control_word(slave_index, ControlWord.SWITCH_ON_AND_ENABLE_OPERATION)
    master.run()

# vim: foldmethod=indent
