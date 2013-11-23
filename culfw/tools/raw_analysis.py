#!/usr/bin/env python
# coding: utf-8
# Author: Hagen Fritsch

def AssembleCommand(sync, bytes, bits, pause, repeat, (high0, low0, high1, low1), finalhigh, data):
  """Assembles a CUL command."""
  return'G%02x%x%d%x%x%02x%02x%02x%02x%02x%s' % (sync, bytes, bits, pause, repeat, high0, low0,
                                                 high1, low1, finalhigh, data)

def UnexpectedData(data, message):
  """Reports an error after receiving a non-conform packet."""
  if type(data) == str:
    data = ord(data)
  print 'Error: UnexpectedData %02x (%c): %s' % (data, data, message)

def ReadTiming(tty, two_bytes=False):
  """Reads one or two timing bytes."""
  data = ord(tty.read(1))
  if two_bytes:
    data |= ord(tty.read(1)) << 8
  return data

def ReadPacket(tty):
  """
  Reads one packet.
  The first byte is the quality indicator (a..p or if an error occurred A..P).
  The next byte specifies what data has been collected:
  b[7..4]: must be 0
  b[3]: high time present (after a falling flank)
  b[2]: high time  size
  b[1]: low time present (after a rising flank)
  b[0]: low time size
  data-type size: 0 == 8bit, 1 == 16bit. LSByte comes first.
  """
  quality = tty.read(1)
  if quality == '': # End of file
    raise KeyboardInterrupt
  if quality == '.': # Magic end-of-packet indicator.
    return None
  has_error = False
  if quality in 'abcdefghijklmnop':
    quality = ord(quality) - ord('a')
  elif quality in 'ABCDEFGHIJKLMNOP':
    print 'Got error packet', quality
    quality = ord(quality) - ord('A')
    has_error = True
  else:
    UnexpectedData(quality, 'Expected Quality Indicator (a..z / A..Z)')
    return
  
  status = ord(tty.read(1))
  has_hightime = status & 8
  has_lowtime = status & 2
  high_size = status & 4
  low_size = status & 1
  if (status & 0xf0) or (not has_hightime and high_size) or (not has_lowtime and low_size):
    UnexpectedData(status, 'Expected Status Indicator')

  high_time = ReadTiming(tty, high_size) if has_hightime else None
  low_time = ReadTiming(tty, low_size) if has_lowtime else None
  return AttributeDict(quality=quality, has_error=has_error, high_time=high_time, low_time=low_time)

def FormatPacket(data, median_times, repeats=6, finalhigh=0, pause=0):
  """Computes several data representations"""
  additional_bits = len(data) % 8
  byte_count = len(data) / 8
  data += '0' * ((8 - additional_bits) % 8)
  hexdata = ('0'*20 + hex(int(data, 2))[2:])[-(byte_count + (additional_bits+7) / 8)*2:]
  command = AssembleCommand(0, byte_count, additional_bits, pause,
                            min(repeats, 15), median_times, finalhigh, hexdata)
  return AttributeDict(cmd=command, hex=hexdata, bytes=byte_count, additional_bits=additional_bits)

class AttributeDict(dict): 
  __getattr__ = dict.__getitem__
  __setattr__ = dict.__setitem__

class Parser(object):
  args = None
  time_pos = 0
  binary = ''
  hasError = False
  
  def __init__(self):
    self.data = []
    self.timings = ([],[],[],[])
    self.results = {}
    self.packet_num = 0
    self.pause = 0
    self.reset()

  def reset(self):
    """Reset saved high- and lowtime after a bit was processed."""
    self.hightime = 0
    self.lowtime = 0
    
  def endpacket(self, pause_time=0):
    """Saves information about the received packet."""
    self.packet_num += 1

    if pause_time == 0:
      pause_time = self.pause
      self.pause = 0

    if not self.hightime:
      print 'Warning: Packet end without falling flank.'
    if self.lowtime:
      print 'Warning: Packet end with unparsed lowtime.'

    # Mark the end for the graph.
    self.data.append((self.time_pos - 0.05, -1))

    if self.args.dump_packets and self.binary:
      print 'Packet %d bits%s: %s' % (len(self.binary), ' with error' if self.hasError else '',
                                      self.binary)

    if self.binary and (not self.hasError or self.args.allow_errors):
      # Increase the counter for the received binary data and save the packet
      # number.
      data = self.results.setdefault(self.binary,
                  AttributeDict(count=0, packets=[], finalhigh=0, pause=[]))
      data.count += 1
      data.packets.append(self.packet_num)
      data.finalhigh = self.hightime
      if pause_time:
        data.pause.append(pause_time)

    # Reset the settings to be ready for the next packet.
    self.binary = ''
    self.reset()
    self.hasError = False

  def read(self, tty):
    """Reads one packet from the tty and properly parses and stores it."""
    packet = ReadPacket(tty)
    #print 'packet:', packet, p.args.pause
    if not packet:
      if p.args.pause is None:
        self.endpacket()
      return

    if packet.has_error:
      if self.args.verbose:
        print 'Warning: A flank could not be reported in time.'
      self.hasError = True

    if packet.low_time and self.args.pause and (packet.low_time << 4) >= self.args.pause:
      self.endpacket(packet.low_time)
      return
    
    report_threshold = self.args.max_time
    if packet.high_time > report_threshold or packet.low_time > report_threshold:
      if (packet.low_time > report_threshold and not packet.high_time and
          not self.hightime and not self.binary):
        # A single lowtime was received at the beginning of a packet.
        # Assume it's the pause after the previous one.
        self.pause = packet.low_time
        return
      else:
        print 'Got a long bit:', packet.high_time, packet.low_time

    if packet.high_time:
      self.hightime = packet.high_time
    if packet.low_time:
      self.lowtime = packet.low_time
    rtime, ftime = self.hightime, self.lowtime
    
    # If both flanks are present and a reasonable quality of signal is there
    # process the bit.
    if packet.quality >= self.args.quality:
      if rtime and ftime:
        # TODO: Implement smarter methods for determining the bit value
        # (e.g. this considers short-long vs long-short, but what about short-short vs long-long)
        is_one = int(rtime > ftime)
        if self.args.bits:
          # bitvalue, hightime, lowtime, total
          print '%d\t%d\t%d\t%d' % (is_one, rtime, ftime, rtime + ftime)
        # Store the timings
        self.timings[is_one*2].append(rtime)
        self.timings[is_one*2+1].append(ftime)
        self.binary += str(is_one)
    else:
      self.hasError = True
      if self.args.verbose:
        print 'Dropping low quality bit.', packet.quality, rtime, ftime

    # Store timing information for plotting and reset.
    if ftime:
      if rtime:
        self.data.append((self.time_pos, 0))
        self.data.append((self.time_pos + 0.05, packet.quality))
      else:
        self.hasError = True
        if self.args.verbose:
          print 'Unexpected lowtime without hightime:', ftime
      self.data.append((self.time_pos + rtime, packet.quality))
      self.data.append((self.time_pos + rtime + 0.05, 0))
      self.time_pos += rtime + ftime
      self.reset()

  def analyze(self):
    """Analyze the received packets assuming they've been sent using the same protocol."""
    try:
      median_times = [sorted(i)[len(i) / 2] for i in self.timings]
    except IndexError:
      print 'Not enough packets were received or another error ocurred.'
      print '(Timings incomplete: %s)' % repr(self.timings)
      return
    print 'Global Timings:', median_times, '(hi0, lo0, hi1, lo1)'

    try:
      target, _ = max(self.results.iteritems(), key=lambda (k, v): v.count)
    except ValueError:
      print 'Not enough packets were received or another error ocurred.'
      print self.results
      return

    have_pause = False
    print 'Data Candidates'

    for binstr, candidate in sorted(self.results.iteritems(), key=lambda (k,v): v.packets[0]):
      pause = sorted(candidate.pause)[len(candidate.pause)/2] if candidate.pause else 0
      if pause != 0:
        pause = (pause<<4) / 1000.
        have_pause = True
        pause_str = 'pause:%1.2fms ' % pause
      else:
        pause = 7
        pause_str = ''
      packet = FormatPacket(binstr, median_times, repeats=candidate.count,
                            finalhigh=candidate.finalhigh, pause=round(pause))
      if binstr == target:
        command = packet.cmd

      # Filter rarely received ones (likely errors).
      if candidate.count >= self.args.packet_count_threshold:
        print '  % 2d: %s %s (%d+%d)\t%s[packets:%s]' % (
            candidate.count, packet.cmd[:17], packet.cmd[17:], packet.bytes, packet.additional_bits,
            pause_str, ','.join(map(str, candidate.packets)))

    print 'Likely command to repeat:', command
    if not have_pause:
      print 'Notice, that the pause time between packets could not be detected.'
      print 'To measure it, you can enable LONG_PULSE in your firmware.'

  def plot(self):
    from matplotlib import pyplot as plt

    data = zip(*self.data)
    data[0] = [i * 16 for i in data[0]] # Adjust the time-scale first.
    plt.plot(*data)
    plt.show()

if __name__ == "__main__":
  import sys, argparse, textwrap

  arg_p = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description='Analyze culfw raw dumps made in X98 mode.',
    epilog=textwrap.dedent('''\
    Suggested usage:
      echo X98 > /dev/ttyACM0
      cat /dev/ttyACM0 > logfile
      %(name)s -d --pause 3000 logfile
      
    But you can also run the script directly on your tty:
    (press Ctrl+C to stop recording and start the analysis)
      echo X98 > /dev/ttyACM0
      %(name)s -d --pause 3000 /dev/ttyACM0
    ''') % dict(name=sys.argv[0]))
  arg_p.add_argument('logfile', help='The log file name to analyze')
  arg_p.add_argument('-a', '--no-analysis', action='store_true', help='don\'t run the analysis')
  arg_p.add_argument('-p', '--plot', action='store_true', help='plot the timing data')
  arg_p.add_argument('-d', '--dump-packets', action='store_true',
                      help='dump each packet directly after receiving it')
  arg_p.add_argument('-b', '--bits', action='store_true', help='dump each received bit')
  arg_p.add_argument('-v', '--verbose', action='store_true',
                      help='output verbose warnings in error conditions')
  arg_p.add_argument('-e', '--allow-errors', action='store_true',
                      help='also analyze packets in which an error was detected')
  arg_p.add_argument('-q', '--quality', default=8, type=int, metavar='N',
                      help='minimum signal strength required to log a packet (0..15)')
  arg_p.add_argument('-c', '--packet-count-threshold', default=2, type=int, metavar='N',
                      help='minimum number of packets needed to show a packet in analysis')
  arg_p.add_argument('--max-time', default=0xff, type=int, metavar='N',
                      help='maximum expected lowtime / hightime for bits (in μs/16)')
  arg_p.add_argument('--pause', type=int, metavar='N',
                      help='''maximum lowtime (in μs) after which a packet end will be
                      assumed. If set, packet-end markers ('.' / 0x2E) will not cause
                      a packet end trigger. Use this if you have LONG_PULSE enabled in
                      your firmware, or you expect only short pause times. A good default
                      for this mode is 3000μs.''')
  Parser.args = arg_p.parse_args()

  p = Parser()
  tty = open(p.args.logfile, "r")
  try:
    while 1:
      p.read(tty)
  except KeyboardInterrupt:
    pass
  finally:
    if not p.args.no_analysis:
      p.analyze()
    if p.args.plot:
      p.plot()
