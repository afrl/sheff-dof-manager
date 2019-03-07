#!/usr/bin/python
""" Script to automate building DofManager module. """
# Note: This script calls the `tee` command, which is available in PowerShell and Linux, but not 
# in Windows cmd. Alternatively, you can pass the log option which will only print to a log and 
# alert to an error, and doesn't use tee.

from __future__ import print_function

import sys, os     
import shutil
import getopt        # Get and parse command-line options
import glob          # File globbing
import subprocess    # Call external commands
import numpy as np   # Used to save CMake config values

try:
  import colorama    # Print to terminal in color

  def print_color(msg, color):
    if   (color == "red"): 
      print(colorama.Fore.RED + msg + colorama.Style.RESET_ALL)
    elif (color == "green"): 
      print(colorama.Fore.GREEN + msg + colorama.Style.RESET_ALL)
    elif (color == "yellow"): 
      print(colorama.Fore.YELLOW + msg + colorama.Style.RESET_ALL)
    elif (color == "blue"): 
      print(colorama.Fore.BLUE + msg + colorama.Style.RESET_ALL)
    elif (color == "magenta"): 
      print(colorama.Fore.MAGENTA + msg + colorama.Style.RESET_ALL)
    elif (color == "cyan"): 
      print(colorama.Fore.CYAN + msg + colorama.Style.RESET_ALL)

except ImportError:
  def print_color(msg, color):
    print(msg)



def usage():
  print ("""
DofManager Makefile. 

Usage: python make.py [OPTION]... [VAR=VALUE]...

Available targets are:

 --config       Configure with cmake: uses default values on initial configure and previously chosen values 
                  otherwise. User can override with config options below.
 --build        Build and install (default target)
 --test         Run all unit tests
 --doc          Generate all doc
 --clear-cache  Clear the CMake cache 
 --clean        Delete build and install dir 

Configuration:
  -h, --help           display this help and exit
  --release            build in release mode (debug is default) 
  --dir                for building with different settings, may want multiple build/install dirs. 
                         Select a different build name, e.g., --dir=gnu, then build dir = build_gnu 
                         and install dir = install_dir_gnu
  --use-mpi=ON         use MPI
  --make-tests=ON      turn on unit testing
  --make-dev=ON        turn on development project (a rapid protyping project intended for
                       devs to use while creating new tests)
  --generator=VS15     CMake generator to use. Default is `VS15` (Visual Studio 15 2017 Win64)
                       for windows and `make` (Unix Makefiles) for linux 
  --print              for debugging only: print the config/build command that will be executed

""")


class options:
  """ Class to hold command-line options, initialized to default values. """
  def __init__(self):
    """ Initialize default and/or saved options"""
    self.log = False   # This is a hidden option. In a module, the developer will typically want to
                       # see the output of config/build (default). But we pass this option when calling 
                       # from the top-level make to hide output unless there is an error. 
    # Default CMake config options. Except for the generator (which is unlikely to change), the CMake 
    # options are saved to a dictionary so they can be reloaded. 
    self.generator = "Visual Studio 15 2017 Win64" 
    if (os.name == "posix"):
      self.generator = "Unix Makefiles"

    self.config = {"build-type" : "Debug",
                   "use-mpi" : "ON", 
                   "make-dev" : "ON",
                   "make-tests" : "ON"}
    
    # Other options
    self.targets = []
    self.top_dir = os.getcwd()
    self.build_dir = os.path.join(self.top_dir, "build")
    self.install_dir = os.path.join(self.top_dir, "install_dir")
    self.print_cmd = False 
    
    # Load previous config options if available
    self.load() 
    
  def save(self):
    """ Save CMake config options """
    np.save(self.build_dir+'/config', self.config) # Actually saves as config.npy

  def load(self):
    """ Load CMake config options """
    if (os.path.exists(self.build_dir + "/config.npy")):
      self.config = np.load(self.build_dir + "/config.npy").item()
 
  def print(self):
    """ Load CMake config options """
    print("\nConfig Options:")
    for key in sorted(self.config):
      print("  ", '{:<15}'.format(key), self.config[key])
    print("")



def process_args():
  """ Read in and parse command-line options and arguments. """

  # First, parse the options
  try:
    opts, args = getopt.gnu_getopt(sys.argv[1:], "h", ["help", "log", "release", "debug", "dir=",
    "use-mpi=", "make-tests=", "make-dev=", "generator=", "print", "config", "build", "test", "doc", 
    "clear-cache", "clean"])

  except getopt.GetoptError:
    print ("\n===Error: invalid command-line options or arguments")
    usage ()
    sys.exit(1)

  # Set the defaults/load previous config before checking options
  user_opts = options()

  for o, a in opts:
    if o in ("-h", "--help"):
      usage()
      sys.exit()
    if o in ("--log"):
      user_opts.log = True
    if o in ("--dir"):
      user_opts.build_dir = os.path.join(user_opts.top_dir, "build_" + a)
      user_opts.install_dir = os.path.join(user_opts.top_dir, "install_dir_" + a)
    if o in ("--generator"):
      if a in ("codeblocks"):
        user_opts.generator = "CodeBlocks - Unix Makefiles"
    if o in ("--print"):
      user_opts.print_cmd = True

    # Override default/previous CMake config options 
    if o in ("--release", "--debug"):
      user_opts.config["build-type"] = o[2:].capitalize()
    if o in ("--use-mpi", "--make-tests", "--make-dev"):
      if   a in ("OFF", "0"):
        user_opts.config[o[2:]] = "OFF"
      elif a in ("ON", "1"):
        user_opts.config[o[2:]] = "ON"

    # Targets 
    if o in ("--config", "--build", "--test", "--doc", "--clear-cache", "--clean"):
      user_opts.targets.append(o[2:])
      
  # Make build the default target if none given
  if (not user_opts.targets):
    user_opts.targets.append("build") 

  return user_opts

def mod_for_os(cmd):
  """Windows requires some modifications via the subprocess call since PowerShell is
  required.  The subprocess will call cmd.exe, so we need to tell cmd to call powershell.
  This function will take a cmd string and modify it for the particular operating system.
  """
  if (os.name == 'nt'):
    cmd = cmd.replace('"', r'\"')
    cmd = cmd.replace('tee', 'Tee-Object')
    cmd = 'powershell "' + cmd + '"'
  # Both config and build targets use the tee command so we can display to the screen 
  # and save to a log simultaneously. The log is needed so that the top-level make
  # knows when config/build has happened. If build was successful, log should contain 
  # the line "Install the project...". Note that, at least on Linux, full color output 
  # only works if you don't combine with tee. Ideally, we'd also like to send stderr to 
  # tee, but this removes all color. Also, with tee we don't get the correct exit status 
  # of config/build. In Linux, we can use `set -o pipefail` to catch the config/build 
  # exit status instead of tee's. Alternatively, pass --log to ship to log file without
  # using tee. 
  elif (os.name == "posix"):
    cmd = "set -o pipefail; " + cmd
  return cmd

def main():
  """ Main driver to execute desired target(s) """
  # Note: this script is not smart if multiple targets are chosen. It will execute them
  # in the same order, no matter the order provided. 

  # Windows requires calling color first before any prints for commandline colors to work.
  if (os.name == 'nt'):
    os.system('color')
  opts = process_args()

  if ("clear-cache" in opts.targets): 
    print_color("Clearing CMake cache", "yellow")
    if (os.path.exists(opts.build_dir + "/CMakeCache.txt")):
      os.remove(opts.build_dir + "/CMakeCache.txt")

    # Also need to clear the saved CMake config options
    if (os.path.exists(opts.build_dir + "/config.npy")):
      os.remove(opts.build_dir + "/config.npy")

  if ("clean" in opts.targets):
    print_color("Cleaning...", "yellow")
    if (os.path.exists(opts.build_dir)):
      shutil.rmtree(opts.build_dir)

    if (os.path.exists(opts.install_dir)):
      shutil.rmtree(opts.install_dir)

  config_flag = 0
  if ("config" in opts.targets): 

    if (not os.path.exists(opts.build_dir)):
      os.mkdir(opts.build_dir)

    if (not os.path.exists(opts.install_dir)):
      os.mkdir(opts.install_dir)

    # First, save options to be reloaded on next config
    opts.save() 

    os.chdir(opts.build_dir)

    log = " | tee cmake.log"
    if (opts.log):
      log = " > cmake.log 2>&1"

    cmake_flags = (" -DCMAKE_BUILD_TYPE=" + opts.config["build-type"] + 
                   " -DDOFMANAGER_BuildDev:BOOL=" + opts.config["make-dev"] +
                   " -DDOFMANAGER_BuildTests:BOOL=" + opts.config["make-tests"])

    print_color("Configuring------------DofManager", "blue")
    cmd = mod_for_os('cmake -G"' + opts.generator + '" ' + 
        '-DCMAKE_INSTALL_PREFIX:PATH="' + opts.install_dir + '" ' + 
        cmake_flags + " .. " + log)

    if (opts.print_cmd): 
      print_color(cmd, "cyan")
      
    config_flag = subprocess.call(cmd, shell=True)

    if (config_flag != 0):
      if (opts.log):
        with open ("cmake.log", 'r') as fin:
          print (fin.read()) 
      print_color("===ERROR during configuration", "red")

  # Print current config options
  if (opts.print_cmd): 
    opts.print() 
  
  build_flag = 0
  if ("build" in opts.targets) and (config_flag == 0):

    # First check if we've configured
    if (not os.path.exists(opts.build_dir + "/CMakeCache.txt")):
      print_color("===ERROR: No cmake config. Please run the --config target first", "red") 
    else: 
      os.chdir(opts.build_dir)

      log = " | tee build.log"
      if (opts.log):
        log = " > build.log 2>&1"

      print_color("Building---------------DofManager", "blue")
      cmd = mod_for_os('cmake --build . --target install' + log)

      if (opts.print_cmd):
        print_color(cmd, "cyan")

      build_flag = subprocess.call(cmd, shell = True)

      if (build_flag != 0):
        if (opts.log):
          with open ("build.log", 'r') as fin:
            print (fin.read()) 
        print_color("===ERROR during build", "red")
      
  if ("test" in opts.targets) and (build_flag == 0): 
  
      # I'm not doing any checking for now...  
      os.chdir(opts.build_dir)

      log = " | tee test.log"
      if (opts.log):
        log = " > test.log 2>&1"

      print_color("Testing----------------DofManager", "blue")
      cmd = mod_for_os('ctest -C ' + opts.config["build-type"] + log)

      if (opts.print_cmd):
        print_color(cmd, "cyan")

      test_flag = subprocess.call(cmd, shell = True)

      if (test_flag != 0):
        if (opts.log):
          with open ("test.log", 'r') as fin:
            print (fin.read()) 
        print_color("===ERROR during unit testing", "red")


  if ("doc" in opts.targets):
    print ("TODO") 

# Execute main on initial import or script call:
if __name__=="__main__":
    main()

