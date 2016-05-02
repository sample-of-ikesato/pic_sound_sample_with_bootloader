#!/usr/bin/env ruby
# -*- coding: utf-8 -*-

$LOAD_PATH.push File.dirname($0)

require "rubygems"
require "serialport"
require 'optparse'

def usage()
  puts "Usage: #{File.basename($0)} [OPTION] USB-DEVICE WAVFILE"
  puts ""
  puts "Options:"
  puts "  -v, --verbose   output verbose format"
  puts "  -d, --debug     debug mode"
  puts "  -h, --help      display this help and exit"
  exit 1
end

debug=false
verbose=false
OptionParser.new {|opt|
  opt.on("--debug") {
    debug=true
  }
  opt.on("--verbose") {
    verbose=true
  }
  opt.on("--help") {
    usage
  }
  opt.parse!(ARGV)
}

usage if ARGV.length < 1
PORT=ARGV.shift
sp = SerialPort.new(PORT, 9600, 8, 1, 0) # 8bit, stopbit 1, parity none


def make_data(rsize, sequence)
  ret = nil
  if sequence + rsize >= 256
    ret = (sequence ... 256).to_a
    ret += (0 ... (sequence + rsize - 256)).to_a
    sequence += rsize - 256
  else
    ret = (sequence ... (sequence+rsize)).to_a
    sequence += rsize
  end
  [ret.pack("C*"), sequence]
end


start = last = Time.now
sequence = 0;
sp.write([1,0].pack("C*"))
dfp = File.open("debug.dat", "wb")
while true
  buf = sp.read(2)
  if buf == 0
    puts "Device detatched!"
    break
  end
  if buf.size < 2
    puts "Error: buffer needs 2 bytes."
    break
  end
  cmd, size = buf.unpack("C*")
  if size > 0
    data = sp.read(size).unpack("C*")
    if data.size < size
      puts "Errror: data needs #{size} bytes"
    end
  end
  if cmd == 2
    rsize = data[0]
    data, sequence = make_data(rsize, sequence)
    sp.write([3, data.size].pack("C*") + data)
    if Time.now - last > 0.1
      last = Time.now
      diff = last - start
      printf("\r%02d:%02d.%03d #{sprintf("%3d", sequence)} #{rsize}", diff/60, diff%60, (diff * 1000)%1000)
    end
  elsif cmd == 9
    dfp.write(data.pack("C*")) if size > 0
    dfp.flush
  else
    puts "unrecognize command #{cmd} #{size} #{data}"
  end
end
puts ""
diff = Time.now - start
printf("%02d:%02d.%03d #{sprintf("%3d", sequence)} #{rsize}\n", diff/60, diff%60, (diff * 1000)%1000)
