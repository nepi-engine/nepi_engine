#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# Game controller support for NEPI interfaces. A controller driver publishes a
# sensor_msgs/Joy topic; an interface that offers Controller Control holds one
# ControllerLink, which follows the operator's selected Joy topic and drives the
# device only through callables the interface injects. Everything that decides
# what a stick or button means lives here, so the interface only wires it in.
# Not to be confused with nepi_controls, which supports ControlsIF.


import os
import copy
import math
import time
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_controls

from sensor_msgs.msg import Joy


logger = nepi_sdk.sdk_logger


#########################
### Xbox Controller Layout

# Indices as the Linux xpad driver reports them through the joystick API
# (/dev/input/js*), which is also the order ROS joy_node publishes them in.
# Values are after normalize_axis(): sticks and D-pad read +1 left and +1 up.
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_LT = 2         # Triggers rest at +1 and read -1 fully pressed
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_RT = 5
AXIS_DPAD_X = 6
AXIS_DPAD_Y = 7
AXIS_COUNT = 8

BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_START = 7
BUTTON_GUIDE = 8
BUTTON_LEFT_STICK = 9
BUTTON_RIGHT_STICK = 10
BUTTON_COUNT = 11

DEAD_ZONE = 0.15
STALE_TIMEOUT_S = 0.5
LOOP_RATE_HZ = 10
JOY_PUBLISH_RATE_HZ = 20

AXIS_RAW_MAX = 32767.0
JOY_MSG_TYPE = 'sensor_msgs/Joy'
OPTIONS_REFRESH_S = 1.0


#########################
### Settings

SETTING_PREFIX = 'controller_'
TOPIC_SETTING = 'controller_topic'
CONTROL_SETTING = 'controller_control'
NONE_TOPIC = 'None'


#########################
### Decisions and Actions

ACTION_MOVE = 'move'
ACTION_STOP = 'stop'
ACTION_HOME = 'home'
ACTION_NONE = 'none'

# Pan/tilt button actions, on a rising edge
PAN_TILT_STOP_BUTTON = BUTTON_B
PAN_TILT_HOME_BUTTON = BUTTON_Y

# Lead of the goto fallback target ahead of the current position at full stick
GOTO_STEP_DEG = 5.0
# Jog speed ratios are sent in these steps, so stick noise does not resend a jog
JOG_SPEED_STEP = 0.1


#########################
### Axis Functions

def apply_dead_zone(value, dead_zone = DEAD_ZONE):
  """Clamp a normalized axis value to -1..1 and zero it inside the dead zone.

  Args:
      value (float): Normalized axis value.
      dead_zone (float, optional): Magnitude at or below which the value reads
          0.0. Defaults to DEAD_ZONE.

  Returns:
      float: The clamped value, exactly 0.0 inside the dead zone or for a value
          that is not a finite number.
  """
  try:
    value = float(value)
  except Exception:
    return 0.0
  if math.isnan(value) or math.isinf(value):
    return 0.0
  value = max(-1.0, min(1.0, value))
  if abs(value) <= dead_zone:
    return 0.0
  return value


def normalize_axis(raw_value, dead_zone = DEAD_ZONE):
  """Normalize a raw Linux joystick axis value the way ROS joy_node does.

  Divides by 32767 and negates, so stick left and stick up read +1, then
  returns exactly 0.0 inside the dead zone. Unlike joy_node, the range outside
  the dead zone is not rescaled.

  Args:
      raw_value (int): Raw js_event axis value, -32767..32767.
      dead_zone (float, optional): Dead zone as a fraction of full scale.
          Defaults to DEAD_ZONE.

  Returns:
      float: The normalized value in -1..1.
  """
  try:
    value = -float(raw_value) / AXIS_RAW_MAX
  except Exception:
    return 0.0
  return apply_dead_zone(value, dead_zone)


#########################
### Topic Functions

def get_controller_topics(topics_list = None, types_list = None):
  """Return the available controller topics: every sensor_msgs/Joy topic, plus None.

  Args:
      topics_list (list, optional): Topic names to search instead of asking the
          ROS master. Passed through to nepi_sdk.find_topics_by_msg.
      types_list (list, optional): Message types matching topics_list.

  Returns:
      list: 'None' followed by the Joy topics, sorted.
  """
  topics = nepi_sdk.find_topics_by_msg(JOY_MSG_TYPE, topics_list = topics_list, types_list = types_list)
  return [NONE_TOPIC] + sorted(set(topics))


def get_topic_options(selected_topic, topics_list = None, types_list = None):
  """Return the controller_topic options: the controller topics plus the selection.

  The current selection stays listed even when its publisher is gone (a
  controller unplugged, or a restored topic not up yet), because a Selection
  value must be one of its options.

  Args:
      selected_topic (str): The currently selected topic.
      topics_list (list, optional): See get_controller_topics.
      types_list (list, optional): See get_controller_topics.

  Returns:
      list: The option strings.
  """
  options = get_controller_topics(topics_list = topics_list, types_list = types_list)
  selected_topic = str(selected_topic)
  if selected_topic not in options:
    options.append(selected_topic)
  return options


def cleanTopic(topic):
  topic = str(topic).strip()
  if topic in ['', NONE_TOPIC, 'none']:
    return NONE_TOPIC
  if topic.startswith('/') == False:
    return None
  return topic


def toBool(value):
  if isinstance(value, str):
    return value.strip().lower() in ['true', '1', 'on']
  return value == True


#########################
### Settings Functions

def is_controller_setting(setting_name):
  """Return True for a setting this module owns (name starts with controller_).

  Args:
      setting_name (str): The setting name.

  Returns:
      bool: True if the setting is a controller setting.
  """
  return str(setting_name).startswith(SETTING_PREFIX)


def get_settings_init_dict(topic_options = None):
  """Return the init dict for the controller_topic and controller_control settings.

  controller_control carries 'param': False. nepi_controls.get_params_dict()
  skips such a setting, so SettingsIF never saves it and no saved config can
  hold it: Controller Control commands motion, so it must come up off on every
  start. controller_topic is saved like any setting, since selecting a topic
  moves nothing.

  Args:
      topic_options (list, optional): Options for controller_topic. Defaults
          to ['None'].

  Returns:
      dict: A nepi_controls init dict.
  """
  if topic_options is None:
    topic_options = [NONE_TOPIC]
  topic_options = [str(option) for option in topic_options]
  if NONE_TOPIC not in topic_options:
    topic_options = [NONE_TOPIC] + topic_options
  init_dict = dict()
  init_dict[TOPIC_SETTING] = {
      'type': 'Selection',
      'options': topic_options,
      'default': NONE_TOPIC,
      'display_name': 'Controller Topic',
      'description': 'Game controller (sensor_msgs/Joy) topic that Controller Control follows'
  }
  init_dict[CONTROL_SETTING] = {
      'type': 'Toggle',
      'default': False,
      'display_name': 'Controller Control',
      'description': 'Left stick moves pan and tilt, B stops, Y goes home. Always off after a restart',
      'param': False
  }
  return init_dict


def refresh_settings_dict(settings_dict, topic, enabled, topic_options):
  """Write the live topic, its options and the enable state into a settings dict.

  Args:
      settings_dict (dict): A controls dict holding the controller settings.
      topic (str): The selected topic.
      enabled (bool): Whether Controller Control is on.
      topic_options (list): The controller_topic options.

  Returns:
      dict: The updated controls dict.
  """
  topic = str(topic)
  topic_options = [str(option) for option in topic_options]
  if topic not in topic_options:
    topic_options.append(topic)
  # Options go first: set_value validates a Selection against the options
  # already in the dict.
  settings_dict = nepi_controls.set_options(settings_dict, TOPIC_SETTING, topic_options)
  settings_dict = nepi_controls.set_value(settings_dict, TOPIC_SETTING, topic)
  settings_dict = nepi_controls.set_value(settings_dict, CONTROL_SETTING, enabled == True)
  return settings_dict


def create_settings_dict(topic = NONE_TOPIC, enabled = False, topic_options = None):
  """Build the controller settings as a nepi_controls controls dict.

  Args:
      topic (str, optional): The selected topic. Defaults to 'None'.
      enabled (bool, optional): Whether Controller Control is on. Defaults to False.
      topic_options (list, optional): The controller_topic options.

  Returns:
      dict: A controls dict with the two controller settings.
  """
  if topic_options is None:
    topic_options = [NONE_TOPIC]
  settings_dict = nepi_controls.create_controls_dict(get_settings_init_dict(topic_options))
  return refresh_settings_dict(settings_dict, topic, enabled, topic_options)


def merge_settings(settings_dict, link):
  """Return a device's settings dict with the controller settings added.

  The device's own dict is copied, never changed. A device with no settings
  (None) gets the controller settings alone.

  Args:
      settings_dict (dict or None): The device's controls dict.
      link (ControllerLink): The link whose live state the settings report.

  Returns:
      dict: The merged controls dict.
  """
  controller_dict = create_settings_dict(link.get_topic(), link.is_enabled(), link.get_topic_options())
  if isinstance(settings_dict, dict) == False:
    return controller_dict
  merged_dict = copy.deepcopy(settings_dict)
  for name in controller_dict.keys():
    if name not in nepi_controls.CONTROLS_META_KEYS:
      merged_dict[name] = controller_dict[name]
  return merged_dict


def update_controller_setting(link, setting_name, setting_value):
  """Apply a controller_ setting update to a ControllerLink.

  controller_control turns ON only through an explicit operator update. A True
  is applied only when it changes the link's live state after link.start().
  The restore SettingsIF runs while the interface is being built comes before
  start(), so a True it delivers is ignored. Every later restore (config init,
  reset, factory reset) re-applies the live value, because the setting is never
  saved ('param': False), so it cannot switch Controller Control on either.
  A False is always applied.

  Args:
      link (ControllerLink): The interface's link.
      setting_name (str): controller_topic or controller_control.
      setting_value: The new value.

  Returns:
      list: [success (bool), message (str)].
  """
  if setting_name == TOPIC_SETTING:
    return link.set_topic(setting_value)
  if setting_name == CONTROL_SETTING:
    enable = toBool(setting_value)
    if enable == False:
      return link.set_enabled(False)
    if link.is_enabled() == True:
      return [True, "Controller Control is already on"]
    if link.is_started() == False:
      return [False, "Ignored a restored Controller Control value. Controller Control turns on only when the operator switches it on"]
    return link.set_enabled(True)
  return [False, "Setting " + str(setting_name) + " is not a controller setting"]


#########################
### Pan/Tilt Functions

def getIndex(values, index, fallback = 0):
  try:
    return values[index]
  except Exception:
    return fallback


def get_rising_edges(buttons, prev_buttons):
  """Return the indexes of buttons pressed now and not pressed before.

  Args:
      buttons (list): Current button states (0/1 or bool).
      prev_buttons (list): Previous button states. Missing entries read released.

  Returns:
      list: Indexes of the buttons with a rising edge.
  """
  rising = []
  for index in range(len(buttons)):
    if getIndex(buttons, index) and not getIndex(prev_buttons, index):
      rising.append(index)
  return rising


def map_pan_tilt(axes, buttons, prev_buttons, dead_zone = DEAD_ZONE):
  """Map a controller's left stick and buttons to pan/tilt rates and actions.

  Signs match the RUI pan/tilt arrows: the left arrow jogs pan +1 and the up
  arrow jogs tilt -1, so stick left gives pan_rate +1 and stick up gives
  tilt_rate -1. B gives a stop and Y a home, each on a rising edge only; when
  both rise together, stop wins.

  Args:
      axes (list): Joy axes, normalized (stick left and up read +1).
      buttons (list): Joy buttons.
      prev_buttons (list): Buttons from the previous cycle.
      dead_zone (float, optional): Applied again, for Joy publishers with a
          smaller dead zone. Defaults to DEAD_ZONE.

  Returns:
      list: [pan_rate (float, -1..1), tilt_rate (float, -1..1), actions (list)].
  """
  pan_rate = apply_dead_zone(getIndex(axes, AXIS_LEFT_X, 0.0), dead_zone)
  tilt_rate = 0.0 - apply_dead_zone(getIndex(axes, AXIS_LEFT_Y, 0.0), dead_zone)
  actions = []
  rising = get_rising_edges(buttons, prev_buttons)
  if PAN_TILT_STOP_BUTTON in rising:
    actions.append(ACTION_STOP)
  elif PAN_TILT_HOME_BUTTON in rising:
    actions.append(ACTION_HOME)
  return [pan_rate, tilt_rate, actions]


def get_goto_target(current_deg, rate, step_deg, min_deg, max_deg):
  """Return the next goto target for an axis with no jog path, clamped to its limits.

  Args:
      current_deg (float): Current axis position.
      rate (float): Stick rate, -1..1; positive moves toward max_deg.
      step_deg (float): How far ahead of the current position the target leads
          at full stick.
      min_deg (float): Lower soft limit.
      max_deg (float): Upper soft limit.

  Returns:
      float: The target in degrees, within [min_deg, max_deg].
  """
  low_deg = min(min_deg, max_deg)
  high_deg = max(min_deg, max_deg)
  rate = max(-1.0, min(1.0, float(rate)))
  target_deg = float(current_deg) + rate * float(step_deg)
  return max(low_deg, min(high_deg, target_deg))


class PanTiltJogPlanner(object):
  """Turns pan/tilt rates into the jog commands the RUI arrow buttons send.

  A jog runs until it is stopped, so a command is sent only when an axis'
  direction or (stepped) speed changes. Stopping one axis stops both on every
  PTX driver, so whenever an axis goes idle the plan is one stop followed by a
  fresh jog for any axis still moving. Call reset() after anything else moves
  or stops the device, so the next plan resends its jogs; a ControllerLink
  given this planner does that itself on every stop and home it sends.

  Args:
      use_speed (bool, optional): True when the jog path takes a speed ratio.
          False sends every jog at speed ratio 1.0, so only a direction change
          resends it. Defaults to True.
      speed_step (float, optional): Speed ratio step. Defaults to JOG_SPEED_STEP.
  """

  AXES = ['pan', 'tilt']

  def __init__(self, use_speed = True, speed_step = JOG_SPEED_STEP):
    self.use_speed = use_speed
    self.speed_step = speed_step
    self.last_cmds = {'pan': None, 'tilt': None}

  def reset(self):
    """Forget the jogs sent, so the next plan resends them."""
    self.last_cmds = {'pan': None, 'tilt': None}

  def get_axis_command(self, rate):
    """Return [direction, speed_ratio] for a rate, or None for a centered axis.

    Args:
        rate (float): Axis rate, -1..1.

    Returns:
        list or None: Direction +1/-1 and a speed ratio stepped to speed_step.
    """
    rate = apply_dead_zone(rate, 0.0)
    if rate == 0.0:
      return None
    direction = 1 if rate > 0 else -1
    if self.use_speed == False:
      return [direction, 1.0]
    speed_ratio = round(abs(rate) / self.speed_step) * self.speed_step
    speed_ratio = round(max(self.speed_step, min(1.0, speed_ratio)), 3)
    return [direction, speed_ratio]

  def plan(self, pan_rate, tilt_rate):
    """Return the jog commands needed to follow these rates.

    Args:
        pan_rate (float): Pan rate, -1..1.
        tilt_rate (float): Tilt rate, -1..1.

    Returns:
        list: [axis, direction, speed_ratio] commands, in order. A direction of
            0 is a stop for that axis ('All' when both went idle).
    """
    cmds = {'pan': self.get_axis_command(pan_rate), 'tilt': self.get_axis_command(tilt_rate)}
    idle_axes = [axis for axis in self.AXES if self.last_cmds[axis] is not None and cmds[axis] is None]
    plan = []
    if len(idle_axes) > 0:
      stop_axis = idle_axes[0] if len(idle_axes) == 1 else 'All'
      plan.append([stop_axis, 0, 0.0])
      self.reset()
    for axis in self.AXES:
      if cmds[axis] is not None and cmds[axis] != self.last_cmds[axis]:
        plan.append([axis, cmds[axis][0], cmds[axis][1]])
        self.last_cmds[axis] = cmds[axis]
    return plan


#########################
### Decision Class

class ControllerDecider(object):
  """Decides each cycle whether a controller moves the device, stops it once, or does nothing.

  Rules, in priority order:
    - A topic change sends one stop if the last command was a move.
    - Controller Control off, or a stale controller (no message younger than
      stale_timeout_s), sends one stop if the last command was a move, then
      nothing.
    - B (stop action) sends a stop.
    - Y (home action) sends home only when has_home is True.
    - A deflected stick sends a move with its rates, every cycle.
    - Sticks returning to center send one stop; centered sticks send nothing,
      so Controller Control never fights other commands.
  """

  def __init__(self, clock = None, stale_timeout_s = STALE_TIMEOUT_S):
    self.clock = clock if clock is not None else time.monotonic
    self.stale_timeout_s = stale_timeout_s
    self.moving = False

  def is_stale(self, last_msg_time, now = None):
    """Return True when no controller message is younger than the stale timeout.

    Args:
        last_msg_time (float or None): Arrival time of the latest message, on
            this decider's clock. None means no message yet.
        now (float, optional): Current time. Defaults to the clock.

    Returns:
        bool: True if stale.
    """
    if last_msg_time is None:
      return True
    if now is None:
      now = self.clock()
    return (now - last_msg_time) > self.stale_timeout_s

  def decide(self, now, enabled, last_msg_time, pan_rate, tilt_rate,
             actions = None, has_home = False, topic_changed = False):
    """Return this cycle's decision.

    Args:
        now (float or None): Current time; None reads the clock.
        enabled (bool): Controller Control state.
        last_msg_time (float or None): Latest message arrival time.
        pan_rate (float): Mapped pan rate.
        tilt_rate (float): Mapped tilt rate.
        actions (list, optional): Button actions from the mapping.
        has_home (bool, optional): Whether the device can go home.
        topic_changed (bool, optional): The topic selection changed since the
            last cycle.

    Returns:
        list: [action, pan_rate, tilt_rate], action one of ACTION_MOVE,
            ACTION_STOP, ACTION_HOME, ACTION_NONE.
    """
    if now is None:
      now = self.clock()
    if actions is None:
      actions = []
    stop_owed = self.moving
    if topic_changed == True and stop_owed:
      self.moving = False
      return [ACTION_STOP, 0.0, 0.0]
    if enabled != True or self.is_stale(last_msg_time, now):
      self.moving = False
      if stop_owed:
        return [ACTION_STOP, 0.0, 0.0]
      return [ACTION_NONE, 0.0, 0.0]
    if ACTION_STOP in actions:
      self.moving = False
      return [ACTION_STOP, 0.0, 0.0]
    if ACTION_HOME in actions and has_home == True:
      self.moving = False
      return [ACTION_HOME, 0.0, 0.0]
    if pan_rate != 0.0 or tilt_rate != 0.0:
      self.moving = True
      return [ACTION_MOVE, pan_rate, tilt_rate]
    self.moving = False
    if stop_owed:
      return [ACTION_STOP, 0.0, 0.0]
    return [ACTION_NONE, 0.0, 0.0]


#########################
### Controller Link Class

class ControllerLink(object):
  """Runs Controller Control for one interface.

  Subscribes to the selected sensor_msgs/Joy topic and unsubscribes when the
  selection changes, caches the latest axes and buttons with their monotonic
  arrival time, and runs its own loop at loop_rate_hz. Each cycle maps the
  controller (map_pan_tilt by default) and lets a ControllerDecider choose a
  move, one stop, home, or nothing. It acts only through the injected
  callables, so it knows nothing about the device behind them.

  Build it before the interface's SettingsIF, which restores the settings, and
  call start() once the interface is up: Controller Control can be switched on
  only after start().

  Args:
      move (callable): move(rate_a, rate_b), called every cycle while a stick
          is deflected.
      stop (callable): stop(), called once per stop decision.
      home (callable, optional): home(), called on the home action.
      has_home (callable, optional): has_home() -> bool.
      options_changed (callable, optional): Called after the controller_topic
          options change, so the interface can republish its settings.
      mapping (callable, optional): mapping(axes, buttons, prev_buttons) ->
          [rate_a, rate_b, actions]. Defaults to map_pan_tilt.
      jog_planner (object, optional): A planner the move callable uses, such
          as PanTiltJogPlanner. The link calls its reset() before every stop
          and home, and keeps it as self.jog_planner.
      clock (callable, optional): Monotonic clock. Defaults to time.monotonic.
      loop_rate_hz (float, optional): Loop rate. Defaults to LOOP_RATE_HZ.
      stale_timeout_s (float, optional): Defaults to STALE_TIMEOUT_S.
  """

  def __init__(self, move, stop, home = None, has_home = None, options_changed = None,
               mapping = None, jog_planner = None, clock = None, loop_rate_hz = LOOP_RATE_HZ,
               stale_timeout_s = STALE_TIMEOUT_S):
    self.moveFunction = move
    self.stopFunction = stop
    self.homeFunction = home
    self.hasHomeFunction = has_home
    self.optionsChangedFunction = options_changed
    self.jog_planner = jog_planner
    self.mapping = mapping if mapping is not None else map_pan_tilt
    self.clock = clock if clock is not None else time.monotonic
    self.loop_rate_hz = loop_rate_hz
    self.decider = ControllerDecider(clock = self.clock, stale_timeout_s = stale_timeout_s)

    self.lock = threading.Lock()
    self.started = False
    self.enabled = False
    self.topic = NONE_TOPIC
    self.topic_options = [NONE_TOPIC]
    self.topic_changed = False
    self.sub = None
    self.clearCache()

  ###############################
  # Public Methods

  def get_topic(self):
    """Return the selected controller topic ('None' when none is selected)."""
    with self.lock:
      return self.topic

  def get_topic_options(self):
    """Return a copy of the controller_topic options."""
    with self.lock:
      return list(self.topic_options)

  def is_enabled(self):
    """Return True while Controller Control is on."""
    with self.lock:
      return self.enabled

  def is_started(self):
    """Return True once start() has run."""
    with self.lock:
      return self.started

  def set_enabled(self, enabled):
    """Switch Controller Control on or off.

    Switching off makes the next cycle send one stop if the controller was
    moving the device. Settings updates go through update_controller_setting(),
    which refuses a restored True; this method does not check.

    Args:
        enabled (bool): The new state.

    Returns:
        list: [success (bool), message (str)].
    """
    enabled = (enabled == True)
    with self.lock:
      changed = (enabled != self.enabled)
      self.enabled = enabled
    state_str = 'on' if enabled else 'off'
    if changed:
      logger.log_info("Controller Control switched " + state_str)
    return [True, "Controller Control is " + state_str]

  def set_topic(self, topic):
    """Select the controller topic, unsubscribing from the previous one.

    A topic that is not published yet is accepted, so a restored selection
    starts working when its controller appears. A change makes the next cycle
    send one stop if the controller was moving the device.

    Args:
        topic (str): A Joy topic name, or 'None'.

    Returns:
        list: [success (bool), message (str)].
    """
    clean_topic = cleanTopic(topic)
    if clean_topic is None:
      return [False, "Controller topic " + str(topic) + " is not a topic name. Select one from the list"]
    with self.lock:
      if clean_topic == self.topic:
        return [True, "Controller topic is already " + clean_topic]
      old_sub = self.sub
      self.sub = None
      self.topic = clean_topic
      self.topic_changed = True
      self.clearCache()
      if clean_topic not in self.topic_options:
        self.topic_options.append(clean_topic)
    self.unregisterSub(old_sub)
    if clean_topic != NONE_TOPIC:
      self.subscribe(clean_topic)
    logger.log_info("Controller topic set to " + clean_topic)
    return [True, "Controller topic set to " + clean_topic]

  def refresh_topic_options(self, topics_list = None, types_list = None):
    """Re-read the available Joy topics into the controller_topic options.

    Args:
        topics_list (list, optional): See get_controller_topics.
        types_list (list, optional): See get_controller_topics.

    Returns:
        bool: True if the options changed.
    """
    with self.lock:
      selected_topic = self.topic
    options = get_topic_options(selected_topic, topics_list = topics_list, types_list = types_list)
    with self.lock:
      if self.topic not in options:
        options.append(self.topic)
      changed = (options != self.topic_options)
      self.topic_options = options
    return changed

  def run_cycle(self, now = None):
    """Run one control cycle: map, decide and act.

    Called by the link's own timer; also usable directly with a test clock.

    Args:
        now (float, optional): Current time on the link's clock.

    Returns:
        list: The decision, [action, rate_a, rate_b].
    """
    if now is None:
      now = self.clock()
    with self.lock:
      enabled = self.enabled
      axes = list(self.axes)
      pressed = sorted(self.pressed_buttons)
      self.pressed_buttons = set()
      last_msg_time = self.last_msg_time
      topic_changed = self.topic_changed
      self.topic_changed = False
    # Rising edges are found per message in _joyCb, so a tap or a release and
    # re-press shorter than one cycle is not lost. The mapping gets this cycle's
    # new presses against a released previous state.
    buttons = [0] * (pressed[-1] + 1 if len(pressed) > 0 else 0)
    for index in pressed:
      buttons[index] = 1
    [rate_a, rate_b, actions] = self.mapping(axes, buttons, [])
    has_home = False
    if ACTION_HOME in actions:
      has_home = (self.callFunction(self.hasHomeFunction) == True)
    decision = self.decider.decide(now, enabled, last_msg_time, rate_a, rate_b,
                                   actions = actions, has_home = has_home, topic_changed = topic_changed)
    action = decision[0]
    if action in [ACTION_STOP, ACTION_HOME] and self.jog_planner is not None:
      self.callFunction(self.jog_planner.reset)
    if action == ACTION_MOVE:
      self.callFunction(self.moveFunction, decision[1], decision[2])
    elif action == ACTION_STOP:
      self.callFunction(self.stopFunction)
    elif action == ACTION_HOME:
      self.callFunction(self.homeFunction)
    return decision

  def start(self):
    """Start the control loop and the topic options refresh. Calls after the first do nothing."""
    with self.lock:
      if self.started == True:
        return
      self.started = True
    nepi_sdk.start_timer_process(1.0 / self.loop_rate_hz, self._loopCb)
    nepi_sdk.start_timer_process(OPTIONS_REFRESH_S, self._optionsCb)

  ###############################
  # Private Methods

  def clearCache(self):
    self.axes = []
    self.buttons = []
    self.pressed_buttons = set()
    self.last_msg_time = None

  def subscribe(self, topic):
    sub = nepi_sdk.create_subscriber(topic, Joy, self._joyCb, queue_size = 1, callback_args = (topic,))
    if sub is None:
      return
    with self.lock:
      keep = (self.topic == topic and self.sub is None)
      if keep:
        self.sub = sub
    if keep == False:
      self.unregisterSub(sub)

  def unregisterSub(self, sub):
    if sub is not None:
      try:
        sub.unregister()
      except Exception as e:
        logger.log_warn("Failed to unregister controller subscriber: " + str(e), throttle_s = 5.0)

  def callFunction(self, function, *args):
    if function is None:
      return None
    try:
      return function(*args)
    except Exception as e:
      logger.log_warn("Controller callback failed: " + str(e), throttle_s = 5.0)
      return None

  def _joyCb(self, msg, args):
    # Drop a message still in flight from a topic that was just deselected
    topic = getIndex(args, 0, None)
    now = self.clock()
    with self.lock:
      if topic != self.topic:
        return
      buttons = list(msg.buttons)
      # Collect presses until the next cycle consumes them
      for index in get_rising_edges(buttons, self.buttons):
        self.pressed_buttons.add(index)
      self.axes = list(msg.axes)
      self.buttons = buttons
      self.last_msg_time = now

  def _loopCb(self, timer):
    # A periodic timer thread dies on an uncaught exception, which would leave
    # a moving device without its stale stop.
    try:
      self.run_cycle()
    except Exception as e:
      logger.log_warn("Controller loop cycle failed: " + str(e), throttle_s = 5.0)

  def _optionsCb(self, timer):
    try:
      changed = self.refresh_topic_options()
      with self.lock:
        topic = self.topic
        resubscribe = (topic != NONE_TOPIC and self.sub is None)
      if resubscribe:
        self.subscribe(topic)
      if changed:
        self.callFunction(self.optionsChangedFunction)
    except Exception as e:
      logger.log_warn("Controller topic refresh failed: " + str(e), throttle_s = 5.0)
