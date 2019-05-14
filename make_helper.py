#!/usr/bin/python
""" Script to automate building BSAM++, including all submodules. Includes additional targets to 
behave more like a makefile. """
# Note: Lowest GNU compiler successfully tested is 8.2.0. Minimal requirement
# is that the compiler has C++14 standard enabled. So theoretically GNU 5.0 should work.
# Note that <experimental/filesystem> requires g++ 5.3 or higher and -lstdc++fs flag, but
# we have a workaround implemented for now, and you can choose to use this or the compiler's
# version of filesystem.

from __future__ import print_function

import sys
import os
import shutil
import glob          # File globbing
import subprocess    # Call external commands
import numpy as np   # Used to save CMake config values
import re            # Regex
import datetime      # Current timestamp
import argparse      # For parsing command line arguments
from enum import Enum

try:
    import colorama    # Print to terminal in color

    def print_color(msg, color="white"):
        if (color == "red"):
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
        else:
            print(colorama.Fore.WHITE + msg + colorama.Style.RESET_ALL)

except ImportError:
    def print_color(msg, color):
        print(msg)


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

def get_write_encoding():
    if os.name == 'nt':
        return 'utf-16'
    else:
        return 'utf-8'

class ModuleType(Enum):
    TopLevel = 1,
    FirstParty = 2,
    SecondParty = 3,
    ThirdParty = 4

class ModuleDirectories:
    def __init__(self, module_name, module_type, module_dir, dir_suffix):
        self.name = module_name
        self.module_type = module_type
        self.module_dir = module_dir
        if module_type == ModuleType.ThirdParty:
            self.build_dir = os.path.normpath(os.path.join(self.module_dir, '../', module_name + '_build' + dir_suffix)).replace('\\', '/')
            self.install_dir = os.path.normpath(os.path.join(self.module_dir, '../', module_name + '_install_dir' + dir_suffix)).replace('\\', '/')
        else:
            self.build_dir = os.path.join(self.module_dir, 'build' + dir_suffix).replace('\\', '/')
            self.install_dir = os.path.join(self.module_dir, 'install_dir' + dir_suffix).replace('\\', '/')

    def __str__(self):
        string = 'Module ' + self.name + ' directories:\n   '
        attrs = vars(self)
        string += '\n   '.join("%s: %s" % item for item in attrs.items())
        return string

class MakeOptions:
    """ Class to hold command-line options, initialized to default values. """

    def __init__(self, module_name, module_type):
        """ Initialize default and/or saved options"""

        self.root_module_dir = os.getcwd()
        self.root_module_type = module_type

        self.config = False
        self.build = False
        self.run_tests = False
        self.doc = False
        self.clear_cache = False
        self.clean = False

        # The CMake options are saved to a dictionary so they can be reloaded.
        self.cmake_opts = {
            "generator": "Visual Studio 15 2017 Win64",
            "build-type": "Debug",
            "make-dev": "OFF",
            "make-tests": "OFF" }
        if (os.name == "posix"):
            self.cmake_opts['generator'] = "Unix Makefiles"
        self.special_cmake_opts = {}

        # Build options
        self.verbose = False
        self.n_jobs = "20"
        self.print_cmd = False
        self.dir_suffix = ""

        # Options specific to the top-level project
        if module_type == ModuleType.TopLevel:
            self.copy_make = False

            # Module selection options
            self.include_third_party = False
            self.include_second_party = False
            self.include_first_party = False
            self.include_top = False
            # Directory options
            self.modules_dir = os.path.join(self.root_module_dir, "modules")

        # Load previous config options if available
        self.load()

    def get_selected_module_dirs(self, module_info):
        module_dirs = []
        if self.root_module_type != ModuleType.TopLevel:
            module_dirs.append(ModuleDirectories(module_info.root_module_name,
                module_info.root_module_type, module_info.root_module_dir, self.dir_suffix))
        else:
            if self.include_top:
                module_dirs.append(ModuleDirectories(module_info.root_module_name,
                    module_info.root_module_type, module_info.root_module_dir, self.dir_suffix))
            if self.include_first_party:
                for module_name, module_dir in module_info.first_party_submodules.items():
                    module_dirs.append(ModuleDirectories(module_name,
                        ModuleType.FirstParty, module_dir, self.dir_suffix))
            if self.include_second_party:
                for module_name, module_dir in module_info.second_party_submodules.items():
                    module_dirs.append(ModuleDirectories(module_name,
                        ModuleType.SecondParty, module_dir, self.dir_suffix))
            if self.include_third_party:
                for module_name, module_dir in module_info.third_party_submodules.items():
                    module_dirs.append(ModuleDirectories(module_name,
                        ModuleType.ThirdParty, module_dir, self.dir_suffix))
        return module_dirs

    def save(self):
        """ Save CMake config options """
        # Actually saves as config.npy
        np.save(os.path.join(self.root_module_dir, "config"), self.config)
        
    def load(self):
        """ Load CMake config options """
        if (os.path.exists(os.path.join(self.root_module_dir, "config.npy"))):
            self.config = np.load(os.path.join(self.root_module_dir, "config.npy")).item()

    def print(self):
        """ Print CMake config options """
        print("\nCMake configuration options:")
        for key in sorted(self.cmake_opts):
            print("  ", '{:<15}'.format(key), self.cmake_opts[key])
        print("")
        print("\nSpecialized CMake configuration options:")
        for key in sorted(self.special_cmake_opts):
            print("  ", '{:<15}'.format(key), self.special_cmake_opts[key])
        print("")
    
    def __str__(self):
        string = 'Options:\n   '
        attrs = vars(self)
        string += '\n   '.join("%s: %s" % item for item in attrs.items())
        return string

class ModuleInfo:
    def __init__(self, root_module_dir):
        self.root_module_dir = os.path.abspath(root_module_dir)
        self.root_module_name, self.root_module_type = \
            self.determine_module_name_and_type(root_module_dir)
        self.first_party_submodules = {}
        self.second_party_submodules = {}
        self.third_party_submodules = {}
        self.find_available_submodules(root_module_dir)

    def __str__(self):
        string = 'Module Info:' + \
            '\n   Root module: ' + self.root_module_name + ' in ' + self.root_module_dir + \
            '\n   First party submodules: ' + str(self.first_party_submodules) + \
            '\n   Second party submodules: ' + str(self.second_party_submodules) + \
            '\n   Third party submodules: ' + str(self.third_party_submodules)
        return string

    def determine_module_name_and_type(self, module_dir):
        """Given the root directory for a module, this method determines
        what kind of module it is based on the directory paths."""
        if os.path.exists(os.path.join(module_dir, 'modules')):
            return ('bsamxx', ModuleType.TopLevel)
        else:
            curr_dir_name = os.path.basename(os.path.normpath(module_dir))
            second_up_dir_name = os.path.basename(os.path.normpath(os.path.join(module_dir, '../')))
            if 'third_party' in second_up_dir_name:
                return (curr_dir_name, ModuleType.ThirdParty)
            elif 'second_party' in second_up_dir_name:
                return (curr_dir_name, ModuleType.SecondParty)
            else:
                return (curr_dir_name, ModuleType.FirstParty)

    def find_available_submodules(self, root_dir):
        """Given the root directory from where make.py is run from, this
        will return a dictionary of all available modules with the keys
        being the module names and the values being the module types."""
        root_dir = os.path.abspath(root_dir)
        if os.path.exists(os.path.join(root_dir, 'modules')):
            # Then we are in the top-level repo
            for directory in os.listdir(os.path.join(root_dir, 'modules')):
                if 'third_party' in directory:
                    for subdir in os.listdir(os.path.join(root_dir, 'modules', directory)):
                        if 'build' not in subdir and 'install' not in subdir \
                            and os.path.isdir(os.path.join(root_dir, 'modules', directory, subdir)):
                            self.third_party_submodules[subdir] = os.path.join(root_dir, 'modules', directory, subdir)
                elif 'second_party' in directory:
                    for subdir in os.listdir(os.path.join(root_dir, 'modules', directory)):
                        self.second_party_submodules[subdir] = os.path.join(root_dir, 'modules', directory, subdir)
                else:
                    self.first_party_submodules[directory] = os.path.join(root_dir, 'modules', directory)
                    
class CMakeOptionType(Enum):
    UserDefinedBool = 1,
    AlwaysTrueBool = 2,
    AlwaysFalseBool = 3

class SpecializedCMakeOption:
    def __init__(self, option_type=CMakeOptionType.UserDefinedBool, \
        config_var='', help_str='', modules_and_var_map={}):

        self.type = option_type
        self.cmd_line_flag = '--' + config_var
        self.args_attr = config_var.replace('-', '_')
        self.config_var = config_var
        self.help_str = help_str
        self.modules_and_var_map = modules_and_var_map

def parse_options(specialized_options = []):
    # Windows requires calling color first before any prints for commandline colors to work.
    if (os.name == 'nt'):
      os.system('color')

    # First figure out what type of module this is being run from
    module_info = ModuleInfo(os.getcwd())

    if module_info.root_module_type == ModuleType.TopLevel:
        description = 'A make utility for configuring, building, and testing BSAM++ \
            and submodules of the underlying FEA framework.'
    else:
        description = 'A utility similar to make for configuring, building, and testing the ' \
            + module_info.root_module_name + ' submodule.'
    parser = argparse.ArgumentParser(description=description)
    # Commands
    command_group = parser.add_argument_group('Commands')
    command_group.add_argument('--config', action='store_true',
                               help='Configures CMake with the provided configuration options')
    command_group.add_argument(
        '--build', action='store_true', help='Build and install')
    command_group.add_argument(
        '--test', action='store_true', help='Run unit tests (requires --make-tests to build tests)')
    command_group.add_argument(
        '--doc', action='store_true', help='Generate the doxygen documentation')
    command_group.add_argument(
        '--clear-cache', action='store_true', help='Clear the CMake cache files')
    command_group.add_argument(
        '--clean', action='store_true', help='Delete build and install directories')
    if module_info.root_module_type == ModuleType.TopLevel:
        command_group.add_argument(
            '--copy-make', action='store_true', help='Copy the make_helper.py from the top-level module down to submodules')
    # Configuration options
    config_group = parser.add_argument_group('Configuration options')
    config_group.add_argument('--generator', default='', action='store',
                              choices=('VS2015', 'VS2017', 'VS2019',
                                       'make', 'codeblocks'),
                              help='Specify the CMake generator to use.  Default is `VS2015` (Visual Studio 15 2017 Win64) for windows and `make` (Unix Makefiles) for linux')
    config_group.add_argument(
        '--release', action='store_true', help='Build in release mode (debug is default)')
    config_group.add_argument(
        '--make-tests', action='store_true', help='Build the submodule unit-tests')
    config_group.add_argument('-j', type=int, dest='n_jobs', default=20,
                              help='Specify the number of parallel jobs to run for cmake/ctest (default 20)')
    # Add special options
    for special_opt in specialized_options:
        if special_opt.type == CMakeOptionType.UserDefinedBool:
            config_group.add_argument(special_opt.cmd_line_flag, action='store_true', help=special_opt.help_str)
    # Top-level specifc options
    if module_info.root_module_type == ModuleType.TopLevel:
        select_group = parser.add_argument_group(
            'Module selection (if none are specified, then all first-party and top are selected)')
        select_group.add_argument(
            '--top', action='store_true', help='Include the top-level project (BSAM++ exe)')
        select_group.add_argument(
            '--submodules', action='store_true', help='Include all first, second, and third party submodules (does not include top-level project)')
        select_group.add_argument(
            '--first-party', action='store_true', help='Include all first-party submodules')
        select_group.add_argument(
            '--second-party', action='store_true', help='Include all second-party submodules')
        select_group.add_argument(
            '--third-party', action='store_true', help='Include all third-party submodules')
    # Misc. options
    misc_group = parser.add_argument_group('Misc. options')
    misc_group.add_argument('--verbose', '-v', action='store_true',
                            help='Display more info during target execution')
    misc_group.add_argument('--print', action='store_true',
                            help='Print the config/build command that will be executed (for debugging)')
    misc_group.add_argument('--dir-suffix', default='', help='for building with different settings, may want multiple build/install dirs. \
                          Specify a build/install suffix, e.g., --dir=gnu, then build dir = build_gnu \
                          and install dir = install_dir_gnu. This option must come BEFORE any other \
                          config options')

    args = parser.parse_args()

    # From the parsed arguments, create a MakeOptions object with
    # the appropriate properties
    opts = MakeOptions(module_info.root_module_name, module_info.root_module_type)
    # Commands
    if args.config:
        opts.config = True
    if args.build:
        opts.build = True
    if args.test:
        opts.run_tests = True
    if args.clear_cache:
        opts.clear_cache = True
    if args.clean:
        opts.clean = True
    # Configuration options
    if args.generator != '':
        if args.generator == 'VS2015':
            opts.cmake_opts['generator'] = 'Visual Studio 14 2015 Win64'
        elif args.generator == 'VS2017':
            opts.cmake_opts['generator'] = 'Visual Studio 15 2017 Win64'
        elif args.generator == 'VS2019':
            opts.cmake_opts['generator'] = 'Visual Studio 16 2019'
        elif args.generator == 'make':
            opts.cmake_opts['generator'] = 'Unix Makefiles'
        elif args.generator == 'codeblocks':
            opts.cmake_opts['generator'] = 'CodeBlocks - Unix Makefiles'
    if args.release:
        opts.cmake_opts['build-type'] = 'Release'
    if args.make_tests:
        opts.cmake_opts['make-tests'] = 'ON'
    opts.n_jobs = args.n_jobs
    # Specialied configuraiton options
    for special_opt in specialized_options:
        if special_opt.type == CMakeOptionType.UserDefinedBool:
            if getattr(args, special_opt.args_attr):
                opts.special_cmake_opts[special_opt.config_var] = 'ON'
            else:
                opts.special_cmake_opts[special_opt.config_var] = 'OFF'
    # Top-level specifc options
    if module_info.root_module_type == ModuleType.TopLevel:
        if args.copy_make:
            opts.copy_make = True
        if args.top:
            opts.include_top = True
        if args.submodules:
            opts.include_first_party = True
            opts.include_second_party = True
            opts.include_third_party = True
        if args.first_party:
            opts.include_first_party = True
        if args.second_party:
            opts.include_second_party = True
        if args.third_party:
            opts.include_third_party = True
        if not args.top and not args.submodules and not args.first_party \
            and not args.second_party and not args.third_party:
            opts.include_top = True
            opts.include_first_party = True
    # Misc. options
    if args.verbose:
        opts.verbose = True
    if args.dir_suffix != '':
        opts.dir_suffix = '_' + args.dir_suffix

    if args.print:
        opts.print_cmd = True

        print(opts)
        print('')
        opts.print()
        print(module_info)

    return opts, module_info

def clear_cache(selected_modules_dirs):
    """Clear CMake cache and saved config opts"""
    print_color("Clearing CMake cache", "yellow")
    for module_dir in selected_modules_dirs:
        build_cmake_path = os.path.join(module_dir.build_dir, "CMakeCache.txt")
        if (os.path.exists(build_cmake_path)):
            os.remove(build_cmake_path)
        # Also need to clear the saved CMake config options
        config_cmake_path = os.path.join(module_dir.build_dir, "config.npy")
        if (os.path.exists(config_cmake_path)):
            os.remove(config_cmake_path)

def clean(selected_modules_dirs):
    """Deletes the build and install directories"""
    print_color("Cleaning...", "yellow")
    for module_dir in selected_modules_dirs:
        if (os.path.exists(module_dir.build_dir)):
            shutil.rmtree(module_dir.build_dir)
        if (os.path.exists(module_dir.install_dir)):
            shutil.rmtree(module_dir.install_dir)

def config(opts, selected_modules_dirs, specialized_options = []):
    for module_dir in selected_modules_dirs:
        # Make sure the build and install directories exist
        if (not os.path.exists(module_dir.build_dir)):
            os.mkdir(module_dir.build_dir)
        if (not os.path.exists(module_dir.install_dir)):
            os.mkdir(module_dir.install_dir)
        
        os.chdir(module_dir.build_dir)

        log = " | tee cmake.log"
        write_log = module_dir.module_dir != opts.root_module_dir
        if write_log:
            log = " > cmake.log 2>&1"

        module_name_caps = module_dir.name.upper().replace('_', '')

        cmake_flags = " -DCMAKE_BUILD_TYPE=" + opts.cmake_opts["build-type"] + " -j " + str(opts.n_jobs)

        if module_dir.module_type != ModuleType.ThirdParty:
            cmake_flags += ' -D' + module_name_caps + '_BuildDev:BOOL=' + opts.cmake_opts["make-dev"] \
                    + ' -D' + module_name_caps + '_BuildTests:BOOL=' + opts.cmake_opts["make-tests"]

        for special_opt in specialized_options:
            if module_dir.name in special_opt.modules_and_var_map.keys():
                if special_opt.type == CMakeOptionType.UserDefinedBool:
                    cmake_flags += ' -D' + special_opt.modules_and_var_map[module_dir.name] + ':BOOL=' + opts.special_cmake_opts[special_opt.config_var]
                elif special_opt.type == CMakeOptionType.AlwaysTrueBool:
                    cmake_flags += ' -D' + special_opt.modules_and_var_map[module_dir.name] + ':BOOL=ON'
                elif special_opt.type == CMakeOptionType.AlwaysFalseBool:
                    cmake_flags += ' -D' + special_opt.modules_and_var_map[module_dir.name] + ':BOOL=OFF'


        print_color("Configuring------------" + module_dir.name, "blue")
        cmd = mod_for_os('cmake -G"' + opts.cmake_opts['generator'] + '" ' + 
            '-DCMAKE_INSTALL_PREFIX:PATH="' + module_dir.install_dir + '" ' + 
            cmake_flags + ' "' + module_dir.module_dir + '" ' + log)

        if opts.print_cmd:
            print_color(cmd, "cyan")

        config_flag = subprocess.call(cmd, shell=True)

        if config_flag != 0:
            if write_log:
                with open("cmake.log", 'r', encoding=get_write_encoding()) as fin:
                    print(fin.read()) 
            print_color("===ERROR during configuration", "red")

def build(opts, selected_modules_dirs):
    for module_dir in selected_modules_dirs:
        os.chdir(module_dir.build_dir)

        log = " | tee build.log"
        write_log = module_dir.module_dir != opts.root_module_dir
        if write_log:
            log = " > build.log 2>&1"

        print_color("Building---------------" + module_dir.name, "blue")
        cmd = mod_for_os('cmake --build . --target install -j ' + str(opts.n_jobs) + ' ' + log)

        if (opts.print_cmd):
            print_color(cmd, "cyan")

        build_flag = subprocess.call(cmd, shell = True)

        if build_flag != 0:
            if write_log:
                with open("build.log", 'r', encoding=get_write_encoding()) as fin:
                    print(fin.read()) 
            print_color("===ERROR during build", "red")

def test(opts, selected_modules_dirs):
    for module_dir in selected_modules_dirs:
        os.chdir(module_dir.build_dir)

        print_color("Testing----------------" + module_dir.name, "blue")
        cmd = mod_for_os('ctest -C ' + opts.cmake_opts["build-type"] + ' -j ' + str(opts.n_jobs))

        test_flag = subprocess.call(cmd, shell = True)

        if (test_flag != 0):
            print_color("===ERROR during unit testing", "red")

def copy_make(opts, selected_modules_dirs):
    if opts.root_module_type == ModuleType.TopLevel:
        for module_dir in selected_modules_dirs:
            if module_dir.module_type != ModuleType.TopLevel:
                print_color('Copying make_helper.py into ' + module_dir.name, "cyan")
                shutil.copy(src=os.path.join(opts.root_module_dir, 'make_helper.py'), \
                    dst=os.path.join(module_dir.module_dir, 'make_helper.py'))

def make(specialized_opts, build_order=[]):
    """The kernel of the make process.  Call this with any specialized options that may
    be needed by the particular module(s).  If build_order is provided, then the modules
    will be sorted by that order for all operations."""

    opts, module_info = parse_options(specialized_opts)
    selected_modules = opts.get_selected_module_dirs(module_info)
    if len(build_order) != 0:
        selected_modules.sort(key=lambda x: build_order.index(x.name))

    if opts.clear_cache:
        clear_cache(selected_modules)
    if opts.clean:
        clean(selected_modules)
    if opts.config:
        config(opts, selected_modules, specialized_opts)
    if opts.build:
        build(opts, selected_modules)
    if opts.run_tests:
        test(opts, selected_modules)
    if opts.copy_make:
        copy_make(opts, selected_modules)

# Example usage:
"""
import make_helper
from make_helper import SpecializedCMakeOption

specialized_opts = [
    SpecializedCMakeOption('make-paraview-reader', 'Build the ParaView reader (only applies to fea_io)', { 'fea_io': 'FEAIO_BuildParaViewReader' }),
    SpecializedCMakeOption('use-mpi', 'Build with MPI (requires MPI environment variables for CMake to find it)', { 'fea_io': 'FEAIO_USE_MPI' }),
    SpecializedCMakeOption('use-fs', 'Turn on if compiler has <experimental/filesystem>', { 'fea_io': 'FEAIO_HaveFilesystem', 'utility': 'UTILITY_HaveFilesystem' })
    ]

make_helper.make(specialized_opts)
"""