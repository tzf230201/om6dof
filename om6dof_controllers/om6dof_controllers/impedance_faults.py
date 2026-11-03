"""Human-readable latched faults; never grants motion or clears a fault."""
import math

FAULTS = {1: 'invalid joint feedback', 2: 'bus health missing/stale/faulted',
          4: 'gravity model failed', 8: 'invalid target received',
          16: 'control-loop period outside limit', 32: 'joint position/speed limit exceeded',
          64: 'gravity exceeds torque capacity', 128: 'nonfinite effort',
          256: 'bus fault occurred since activation'}


def limit_description(values, joints):
    """Decode the first limit trip; reject unknown or malformed telemetry."""
    if (len(values) != 7 or not all(math.isfinite(x) for x in values)
            or values[0] != 1 or values[1] not in (1, 2)
            or values[6] not in (0, 1)
            or values[2] != int(values[2]) or not 0 <= values[2] < len(joints)
            or values[4] >= values[5]):
        return ''
    joint = joints[int(values[2])]
    if values[1] == 2:
        return (f'{joint} speed {values[3]:+.4f} rad/s exceeded '
                f'[{values[4]:+.4f}, {values[5]:+.4f}] rad/s')
    return (f'{joint} position {math.degrees(values[3]):+.2f} deg outside '
            f'[{math.degrees(values[4]):+.2f}, {math.degrees(values[5]):+.2f}] deg')


def fault_message(status, flags, detail=''):
    if flags < 0 or flags != int(flags):
        return 'Invalid controller fault telemetry; support and inspect the stack.'
    flags = int(flags)
    if flags:
        causes = [text for bit, text in FAULTS.items() if flags & bit]
        if flags & ~sum(FAULTS):
            causes.append('unknown fault bits')
        if flags & 32 and detail:
            causes.append('First limit trip: ' + detail)
        return (f'Latched fault {flags}: ' + '; '.join(causes) +
                '. Target torque disabled; no automatic reset. '
                'Support the arm and use the supervised recovery/Stop controls.')
    if status == 4:
        return 'Gravity ramp in progress; keep supporting and wait before Run target.'
    return 'Invalid controller status; target disabled.'
