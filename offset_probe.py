import logging
from . import probe
from . import gcode_macro

HINT_TIMEOUT = """
If the probe did not move far enough to trigger, consider
reducing the Z axis minimum position so the probe can travel further.
The probe will not move further than the minimum Z position.
"""


class OffsetProbe:
    def __init__(self, config, mcu_probe):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe = mcu_probe

        self.probe_x = config.getfloat('probe_x', 0.)
        self.probe_y = config.getfloat('probe_y', 0.)

        # The starting Z height when probing tools - all tools must be accurate to within this value
        self.tool_probe_z = config.getfloat('reprobe_height', 1., above=0.)

        # Plunge speed
        self.speed = config.getfloat('speed', 5.0, above=0.)
        # Speed to move Z away from the probe after trigger
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.lift_distance = config.getfloat('lift_distance', 0.5, above=0.)
        # X/Y move speeds
        self.move_speed = config.getfloat('move_speed', 100.0, above=0.)

        # Gcode-based moves, this moves the toolhead relative to the tool offsets
        self.gcode_move = self.printer.load_object(config, "gcode_move")

        # Infer Z position to move to during a probe
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position = zconfig.getfloat('position_min', 0.,
                                               note_valid=False)
        else:
            pconfig = config.getsection('printer')
            self.z_position = pconfig.getfloat('minimum_z_position', 0.,
                                               note_valid=False)

        # Register custom gcode commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('OFFSET_PROBE', self.cmd_OFFSET_PROBE,
                                    desc=self.cmd_OFFSET_PROBE_help)

        # Load the start/end/toolchange gcode macros
        gcode_macro = self.printer.lookup_object('gcode_macro')
        self.start_gcode = gcode_macro.load_template(config, 'start_gcode', '')
        self.end_gcode = gcode_macro.load_template(config, 'end_gcode', '')
        self.toolchange_gcode_0 = gcode_macro.load_template(config, 'toolchange_gcode_0', '')
        self.toolchange_gcode_1 = gcode_macro.load_template(config, 'toolchange_gcode_1', '')

    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float('LIFT_SPEED', self.lift_speed, above=0.)
        return self.lift_speed


    def _ensure_homed(self):
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before offset probing")

    # Probe the bed at the current position using the given speed and toolhead
    def _probe(self, speed, toolhead):
        self._ensure_homed()

        toolhead = self.printer.lookup_object(toolhead)
        phoming = self.printer.lookup_object('homing')

        pos = toolhead.get_position()
        pos[2] = self.z_position

        mcu_probe = self.mcu_probe

        try:
            epos = phoming.probing_move(mcu_probe, pos, speed)
        except self.printer.command_error as e:
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
            raise self.printer.command_error(reason)
        return epos[2]

    # Probe, then retract and re-probe at 1/2 speed
    def _accurate_probe(self, speed, toolhead):
        toolhead = self.printer.lookup_object(toolhead)
        start_z = self._probe(speed, toolhead)
        reprobe_speed = round(speed / 2)

        self._lift_between_probes(2)
        accurate_z = self._probe(reprobe_speed, toolhead)
        return accurate_z

    def _get_gcode_position(self, x=None, y=None, z=None):
        offsets = self.gcode_move.base_position

        if x is not None:
            x += offsets[0]
        if y is not None:
            y += offsets[1]
        if z is not None:
            z += offsets[2]
        return x, y, z

    def _lift_between_probes(self, toolhead, dist=None):
        if dist is None:
            dist = self.lift_distance
        toolhead = self.printer.lookup_object(toolhead)
        liftpos = toolhead.get_position()
        liftpos[2] += dist
        toolhead.manual_move(liftpos, self.lift_speed)

    cmd_OFFSET_PROBE_help = "Calculate the offsets for all defined tools"

    def cmd_OFFSET_PROBE(self, gcmd):
        # Run the start gcode which hopefully undocks any tools
        self.start_gcode.run_gcode_from_command()

        # Move to the probe point, first offsetting the target by the probe offset amount
        toolhead = self.printer.lookup_object('toolhead')

        # For each extruder, run T<extruder num> and then probe
        offsets = []

        # Execute the toolchange script for extruder
        toolchange_script = self.toolchange_gcode_0
        toolchange_script.run_gcode_from_command()
        # Generate new offset coordinates based on the absolute probe pos, and the re-probe height
        curr_pos = toolhead.get_position()
        coord = self.probe_x, y=self.probe_y, z=curr_pos[2] + curr_pos[3:]

        # Move to the absolute probe point
        toolhead.manual_move(coord, self.move_speed)
        # Probe and get the new offset
        base_z = self._accurate_probe(self.speed, toolhead)
        gcmd.respond_info('Tool %d: %.6f' % (0, base_z))
        self._lift_between_probes(toolhead)

        toolhead = self.printer.lookup_object('toolhead1')

        # Execute the toolchange script for extruder1
        toolchange_script = self.toolchange_gcode_1
        toolchange_script.run_gcode_from_command()
        # Generate new offset coordinates based on the absolute probe pos, and the re-probe height
        curr_pos = toolhead.get_position()
        coord = [i for i in self._get_gcode_position(x=self.probe_x, y=self.probe_y, z=curr_pos[2])] + curr_pos[3:]

        # Move to the absolute probe point
        toolhead.manual_move(coord, self.move_speed)
        # Probe and get the new offset
        z = self._accurate_probe(self.speed, toolhead)
        offsets.append(z - base_z)
        gcmd.respond_info('Tool %d: %.6f' % (0, z))
        self._lift_between_probes(toolhead)

        self.end_gcode.run_gcode_from_command()

        offset_str = []
        for i in range(len(offsets)):
            offset_str.append('T%d=%.6f' % (i, offsets[i]))
        gcmd.respond_info('Offsets: %s' % (', '.join(offset_str)))

        return offsets


def load_config(config):
    # Piggyback on the probe.ProbeEndstopWrapper which gives us z-endstop-registration
    return OffsetProbe(config, probe.ProbeEndstopWrapper(config))
