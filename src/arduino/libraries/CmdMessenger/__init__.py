#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, warnings

MAXCALLBACKS = 50
DEFAULT_TIMEOUT = 5000

class CmdMessenger(object):
    """
    Python implementation of CmdMessenger for Arduino/AVR/Java/C#.
    Create an instance of this object passing an open file-like
    object, and optionally the field separator, the command separator
    and the escape character. Defaults are respectively ",", ";", "/".
    Backslash doesn't seem to work on Arduino as an escape character.

    **Note:** the file-like object should be open, and its read method
    should not block, or the parser will also block. This can be
    accomplished with the Python serial module by setting the timeout
    to 0 (zero). If your object is not that smart, you can write a
    wrapper and pass it as :py:data:`~readmeth`. The wrapper should
    accept one argument, and return the number of bytes specified in
    that argument. The file-like object should implement read (unless
    you're going to pass a wrapper), write, flush and close.
    """

    print_newline = False
    """
    If True, a newline (Windows/Arduino style, \\r\\n) will be written
    after each sent command. Does not change the way commands are
    received.
    """

    _default_callback = lambda *a, **kw: None
    _callbacks = {}
    _file_buffer = ""
    _commands = []
    _max_callbacks = MAXCALLBACKS
    _timeout = DEFAULT_TIMEOUT

    def __init__(self, flobject, fld_separator=",", cmd_separator=";", esc_character="/", readmeth=None, cmdNames=None):
        self.commandNames = cmdNames
        self._file = flobject
        self._fld_sep = fld_separator
        self._cmd_sep = cmd_separator
        self._esc_char = esc_character

        if readmeth is None:
            self._read = self._file.read
        else:
            self._read = readmeth

        if self._file.closed:
            raise IOError("The file-like object is closed.")

    def attach(self, func, msgid=None):
        """
        Attach :py:data:`func` as a callback for the command with ID
        :py:data:`msgid`. :py:data:`msgid` must be an integer between
        0 and :py:attr:`~CmdMessenger._max_callbacks`.

        If :py:data:`msgid` is None or not specified, the callable
        will be set as a default callback for unrecognized commands.

        Callbacks must accept three arguments: the command ID, a list of
        arguments and the command string. Arguments will be sent as
        strings. You can use :py:meth:`~CmdMessenger.typify_args` to
        convert them to the appropriate types.

        There can be only one callback for each message ID.
        """

        if msgid and msgid >= 0 and msgid < self._max_callbacks:
            self._callbacks[msgid] = func
        else:
            self._default_callback = func

    def detach(self, msgid):
        """
        Detach callback from :py:data:`msgid`. You don't need to run
        this to attach a new callback. Just attach it.
        """
        self._callbacks.pop(msgid)

    def feed_in_data(self, size=10000):
        """
        Read the :py:data:`size` bytes from the file-like object and
        process them. If :py:data:`msgid` is not specified, it will
        default to 10000 (~10KB). If you specified a read wrapper,
        it will be used for reading.
        """
        self._file_buffer += self._read(size)
        #print "BUFFER", self._file_buffer
        self._process_buffer()
        #print "COMMANDS", self._commands
        self._exec_commands()

    def feed_in_string(self, string):
        """
        Add :py:data:`string` to the message buffer and process it.
        """
        self._file_buffer += string
        self._process_buffer()
        self._exec_commands()

    def _process_buffer(self):
        if self._cmd_sep not in self._file_buffer:
            return

        cmds = self._file_buffer.split(self._cmd_sep)
        for c in cmds:
            # Leave the last one alone
            if cmds.index(c) != len(cmds) - 1:
                c = c.strip()

            # Unsplit escaped separators
            while c.endswith(self._esc_char):
                i = cmds.index(c)
                cmds.pop(i)
                fc = cmds.pop(i)
                cmds.insert(i, c + self._cmd_sep + fc)

        self._file_buffer = ""
        # If the string ends with a separator, last item is empty
        if cmds[-1] != "":
            self._file_buffer = cmds.pop(-1)

        self._commands += cmds

        while "" in self._commands:
            self._commands.remove("")

    def _exec_commands(self, run_max=None):
        count = 0

        while len(self._commands) > 0:
            if run_max is not None and count >= run_max:
                break
            try:
                self.exec_command(self._commands.pop(0))
            except ValueError as e:
                warnings.warn("Unable to parse invalid command. Make sure you're not sending debug output. ValueError: " + e.message, RuntimeWarning)
            count += 1

    def exec_command(self, command):
        """
        Process the :py:data:`command` and run its callback. Note
        that this is used internally, and its only documented in case
        you want to run a custom command in the local host manually.
        """
        args = self.read_args(command, (int,))
        if not args:
            return
        msgid = args.pop(0)
        if msgid in self._callbacks:
            self._callbacks[msgid](msgid, args, command)
        else:
            self._default_callback(msgid, args, command)

    def wait_for_ack(self, ackid, errid=None, msgid=None, timeout=60):
        """
        Block until an acknoweledgement or error message has been
        received, or until :py:data:`timeout` has expired. The error
        message ID is optional. Note that this will download all the
        messages from the file-like object or read wrapper, but won't
        run any callback by itself. Use :py:meth:`~CmdMessenger.feed_in_data`
        to run them.
        The acknoweledgement message will be removed from the command list.

        If :py:data:`msgid` is also specified, the function will only return
        if the acknoweledgement/error ID is followed by it, unless it times out.

        The method will return None if timed out, or the a list of the
        arguments received, with the message ID as the first item.
        """
        init_t = time.time()
        while time.time() - init_t < timeout:
            self._file_buffer += self._read(10000)
            self._process_buffer()
            for i in self._commands:
                args = self.read_args(i, (int,))
                if args[0] in (ackid, errid):
                    if msgid and len(args) >= 2 and int(args[1]) == msgid:
                        self._commands.remove(i)
                        return args
                    elif msgid is None:
                        self._commands.remove(i)
                        return args
            time.sleep(0.001)
        return None

    def read_args(self, command, types=None):
        """
        Split a :py:data:`command` into its single arguments. If :py:data:`types`
        is provided and is a list of primitives/callables, it will convert
        its respective argument to that type.
        """
        cmds = self._file_buffer.split(self._cmd_sep)
        cmd = command.split(self._fld_sep)
        #print "COMMAND", cmd

        for a in cmd:
            # Unsplit escaped separators. This code is a disaster. TODO: Can I remove all this escaping nonsense?
            while a.endswith(self._esc_char):
                    i = cmds.index(a)
                    cmds.pop(i)
                    fa = cmds.pop(i)
                    cmds.insert(i, a + self._fld_sep + fa)

        for a in cmd:
            a = a.strip()

        if len(cmd) == 0:
            return
        elif len(cmd) == 1 and cmd[0] == "":
            return

        if types:
            cmd = self.typify_args(cmd, types)

        return cmd

    def typify_args(self, arglist, types):
        """
        Accepts a list of strings and a list of primitives/callables.
        Every type/callable will be called with the respective item of
        the argument list. If a type is None, the respective item will be
        left alone. The argument list should be longer or as big as the
        types list.

        Ex. typify_args(["10", "7.9", "inf", "hello"], [int, float, float, None])
        --> [10, 7.9, inf, "hello"]
        """
        for i in xrange(0, len(types)):
            if not types[i]:
                continue
            try:
                arglist[i] = types[i](arglist[i])
            except ValueError:
                arglist[i] = arglist[i]  # Leave as a raw bytestring.
        return arglist

    def escape(self, string):
        """
        Return a new string with the field separator character escaped.
        """
        return string.replace(self._cmd_sep, self._esc_char + self._cmd_sep) \
                     .replace(self._fld_sep, self._esc_char + self._fld_sep)

    def unescape(self, string):
        """
        Return a new string with the escaped field separators unescaped.
        """
        return string.replace(self._esc_char + self._cmd_sep, self._cmd_sep) \
                     .replace(self._esc_char + self._fld_sep, self._fld_sep)

    def send_cmd(self, msgid, *args, **kwargs):
        """
        Send command with ID :py:data:`msgid` and positional arguments
        as arguments to the file-like object. If the keyword argument
        flush is True (default), the file-like object will also be flushed.
        """
        if self.commandNames is not None:
            msgType = self.commandNames[msgid]
        else:
            msgType = msgid
        self._file.write(str(msgid))
        for a in args:
            self._file.write(self._fld_sep)
            if type(a) == bool:
                a = int(a)
            self._file.write(str(a))
        self._file.write(self._cmd_sep)

        if self.print_newline:
            self._file.write("\r\n")

        if "flush" in kwargs and kwargs["flush"]:
            self._file.flush()

    def close(self):
        """
        Close the file-like object.
        """
        return self._file.close()
