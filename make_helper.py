#! /usr/bin/python3

""" Script to automate building the executable, including all submodules. Includes additional
targets to behave more like a makefile. """
# Note: Lowest GNU compiler successfully tested is 8.2.0. Minimal requirement
# is that the compiler has C++14 standard enabled. So theoretically GNU 5.0 should work.
# Note that <experimental/filesystem> requires g++ 5.3 or higher and -lstdc++fs flag, but
# we have a workaround implemented for now, and you can choose to use this or the compiler's
# version of filesystem. 

from __future__ import print_function

import argparse      # For parsing command line arguments
import datetime      # Current timestamp
import glob          # File globbing
import numpy as np   # Used to save CMake config values
import os
import re            # Regex
import shutil
import subprocess    # Call external commands
import sys

from enum import Enum

#####################################################################################

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

#####################################################################################
# Only prints message if "verbose" is turned on.

def print_if_verbose( msg, opts ):
    if( opts.verbose ):
        print( msg )

#####################################################################################

def mod_for_os(cmd, shell_executable):
    """Windows requires some modifications via the subprocess call since PowerShell is
    required.  The subprocess will call cmd.exe, so we need to tell cmd to call powershell.
    This function will take a cmd string and modify it for the particular operating system.
    """
    if (os.name == 'nt'):
        if shell_executable is None or 'cmd' in shell_executable: # defaults to cmd
            cmd = cmd.replace('\\', '/')
            cmd = cmd.replace('"', r'\"')
            cmd = cmd.replace('tee', 'Tee-Object')
            cmd = 'powershell "' + cmd + '"'
        elif 'powershell' in shell_executable:
            cmd = cmd.replace('\\', '/')
            cmd = cmd.replace('"', r'\"')
            cmd = cmd.replace('tee', 'Tee-Object')
        else:
            raise Exception('Unrecognized shell for Windows: ' + shell_executable)
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

#####################################################################################

def get_write_encoding():
    if os.name == 'nt':
        return 'utf-16'
    else:
        return 'utf-8'

#####################################################################################

def find_generator():
    if os.name == 'nt':
        if os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC") \
            or os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Tools\MSVC") \
            or os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\VC\Tools\MSVC") \
            or os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Test Professional\VC\Tools\MSVC"):
            return "VS2019"
        elif os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Tools\MSVC") \
            or os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2017\Professional\VC\Tools\MSVC") \
            or os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\VC\Tools\MSVC") \
            or os.path.exists(r"C:\Program Files (x86)\Microsoft Visual Studio\2017\Test Professional\VC\Tools\MSVC"):
            return "VS2017"
        else:
            print_color('No valid Visual Studio installation was found automatically.', 'red')
            return ''
    else:
        return 'make'    

#####################################################################################

class ModuleType( Enum ):
    TopLevel    = 1,
    FirstParty  = 2,
    SecondParty = 3,
    ThirdParty  = 4

#
# End class ModuleType
#####################################################################################

class ModuleDirectories:
    def __init__(self, module_name, module_type, module_dir, dir_suffix):
        self.name = module_name
        self.module_type = module_type
        self.module_dir = module_dir
        if module_type == ModuleType.ThirdParty:
            self.build_dir = os.path.normpath(os.path.join(self.module_dir, '..', module_name + '_build' + dir_suffix)).replace('\\', '/')
            self.install_dir = os.path.normpath(os.path.join(self.module_dir, '..', module_name + '_install_dir' + dir_suffix)).replace('\\', '/')
        else:
            self.build_dir = os.path.join(self.module_dir, 'build' + dir_suffix).replace('\\', '/')
            self.install_dir = os.path.join(self.module_dir, 'install_dir' + dir_suffix).replace('\\', '/')

    def __str__(self):
        string = 'Module: ' + self.name + '\n   '
        attrs = sorted( vars(self).items() )
        string += '\n   '.join( "%s: %s" % item for item in attrs )
        return string

#
# End class ModuleDirectories
#####################################################################################

class MakeOptions:
    """ Class to hold command-line options, initialized to default values. """

    def __init__( self, module_name, module_type ):
        """ Initialize default options"""

        self.root_module_dir = os.getcwd()
        self.root_module_type = module_type
        self.root_build_dir = "" # Need path to store config files

        self.config = False
        self.build = False
        self.run_tests = False
        self.list_tests = False
        self.doc = False
        self.clear_cache = False
        self.clean = False
        self.clean_toolchain = False
        self.copy_make = False
        self.open_vs = False
        self.open_vscode = False
        # Store cmake cmd, since on some linux installs it is cmake3
        self.cmake_cmd = "cmake"
        self.ctest_cmd = "ctest"

        # The CMake options are saved to a dictionary so they can be reloaded.
        self.cmake_opts = {
            "generator": "Visual Studio 15 2017",
            "architecture": "x64",
            "toolset" : "",
            "build-type": "Debug",
            "make-tests": "OFF" }
        # Note: this default should never get used due to find_generator()
        # set defaults there.
        if (os.name == "posix"):
            self.cmake_opts['generator'] = "Unix Makefiles"
            self.cmake_opts['architecture'] = ""
        self.special_cmake_opts = {}

        # Build options
        self.verbose = False
        self.n_jobs = "1"
        self.print_cmd = False
        self.dir_suffix = ""
        self.test_num = None
        self.test_name = None
        if os.name == 'nt':
            self.shell_executable = None
        else:
            # We actually assume some utilities unique to bash in this build process
            # However, if the user changes their default shell (or they are using WSL),
            # then we need to override the shell used for subprocesses with bash.
            # Note: the user can also override this with a cmd line option
            self.shell_executable = '/bin/bash'

        # Options specific to the top-level project
        if module_type == ModuleType.TopLevel:
            # Module selection options
            self.include_third_party = False
            self.include_second_party = False
            self.include_first_party = False
            self.include_top = False
            self.include_specific_modules = []
            # Directory options
            self.modules_dir = os.path.join(self.root_module_dir, "modules")


    def get_selected_module_dirs( self, module_info ):
        module_dirs = []
        if self.root_module_type != ModuleType.TopLevel:
            module_dirs.append( ModuleDirectories( module_info.root_module_name,
                                                   module_info.root_module_type, module_info.root_module_dir, self.dir_suffix ) )
        else:
            if self.include_top:
                module_dirs.append(ModuleDirectories(module_info.root_module_name,
                    module_info.root_module_type, module_info.root_module_dir, self.dir_suffix))
            for module_name, module_dir in module_info.first_party_submodules.items():
                if self.include_first_party or module_name in self.include_specific_modules:
                    module_dirs.append(ModuleDirectories(module_name,
                        ModuleType.FirstParty, module_dir, self.dir_suffix))
            for module_name, module_dir in module_info.second_party_submodules.items():
                if self.include_second_party or module_name in self.include_specific_modules:
                    module_dirs.append(ModuleDirectories(module_name,
                        ModuleType.SecondParty, module_dir, self.dir_suffix))
            for module_name, module_dir in module_info.third_party_submodules.items():
                if self.include_third_party or module_name in self.include_specific_modules:
                    module_dirs.append(ModuleDirectories(module_name,
                        ModuleType.ThirdParty, module_dir, self.dir_suffix))
        return module_dirs

    ################
    # Note: currently saving and loading special_cmake_opts, but not that once an 
    # opt is set, you can't unset it without removing the config (via clear-cache or clean)
    def save( self ):
        """ Save CMake config options """
        # Actually saves as config.npz
        np.savez( os.path.join(self.root_build_dir, "config"), cmake_opts = self.cmake_opts, special_cmake_opts = self.special_cmake_opts )
        
    ################

    def load( self ):
        """ Load CMake config options """
        if( os.path.exists(os.path.join(self.root_build_dir, "config.npz")) ):
            data = np.load(os.path.join(self.root_build_dir, "config.npz"), allow_pickle=True)

            copts  = data['cmake_opts'].item()
            scopts = data['special_cmake_opts'].item()

            self.cmake_opts.update( copts )
            self.special_cmake_opts.update( scopts )

    ################

    def print( self ):
        """ Print CMake config options """

        print( "MakeOptions:\n" )
        print( "  root_module_dir: ",  self.root_module_dir )
        print( "  root_module_type: ", self.root_module_type )
        print( "  root_build_dir: ",   self.root_build_dir )

        print( "  config: ",           self.config )
        print( "  build: ",            self.build )
        print( "  run_tests: ",        self.run_tests )
        print( "  list_tests: ",       self.list_tests )
        print( "  doc: ",              self.doc )
        print( "  clear_cache: ",      self.clear_cache )
        print( "  clean: ",            self.clean )
        print( "  copy_make: ",        self.copy_make )
        
        print( "  verbose: ",          self.verbose )
        print( "  n_jobs: ",           self.n_jobs )
        print( "  print_cmd: ",        self.print_cmd )
        print( "  dir_suffix: ",       self.dir_suffix )
        print( "  test_num: ",         self.test_num )
        print( "  test_name: ",        self.test_name )

        if self.root_module_type == ModuleType.TopLevel:
            print( "  TopLevelModule fields:\n" )
            print( "    include_third_party: ",  self.include_third_party )
            print( "    include_second_party: ", self.include_second_party )
            print( "    include_first_party: ",  self.include_first_party )
            print( "    include_top: ",          self.include_top )
            print( "    modules_dir: ",          self.modules_dir )

        print( "\n  CMake configuration options:" )
        for key in sorted( self.cmake_opts ):
            print("   ", '{:<15}'.format(key), self.cmake_opts[key])
        print( "" )
        print( "\nSpecialized CMake configuration options:" )
        for key in sorted( self.special_cmake_opts ):
            print( "   ", '{:<15}'.format(key), self.special_cmake_opts[key] )
        print( "" )
    
    ################

    def __str__( self ):
        string = 'Options:\n   '
        attrs = vars( self )
        string += '\n   '.join("%s: %s" % item for item in attrs.items())
        return string

#
# End class MakeOptions
#####################################################################################

class ModuleInfo:
    def __init__( self, root_module_dir, root_repo_name = '' ):
        self.root_module_dir = os.path.abspath(root_module_dir)
        self.root_module_name, self.root_module_type = \
            self.determine_module_name_and_type( root_module_dir, root_repo_name )
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

    def determine_module_name_and_type(self, module_dir, root_repo_name = ''):
        """Given the root directory for a module, this method determines
        what kind of module it is based on the directory paths."""

        if os.path.exists(os.path.join(module_dir, 'modules')):
            return (root_repo_name, ModuleType.TopLevel)
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

            # Use dependencies python module to check that the module is indeed a possible module
            import dependencies
            available_modules = dependencies.simple_dependencies.keys()

            for directory in os.listdir(os.path.join(root_dir, 'modules')):
                if 'third_party' in directory:
                    for subdir in os.listdir(os.path.join(root_dir, 'modules', directory)):
                        if 'build' not in subdir and 'install' not in subdir \
                            and os.path.isdir(os.path.join(root_dir, 'modules', directory, subdir)):
                            # Exclude directories that are not an actual module
                            if subdir in available_modules:
                                self.third_party_submodules[subdir] = os.path.join(root_dir, 'modules', directory, subdir)
                elif 'second_party' in directory:
                    for subdir in os.listdir(os.path.join(root_dir, 'modules', directory)):
                        # Exclude directories that are not an actual module
                        if subdir in available_modules:
                            self.second_party_submodules[subdir] = os.path.join(root_dir, 'modules', directory, subdir)
                else:
                    # Exclude directories that are not an actual module
                    if directory in available_modules:
                        self.first_party_submodules[directory] = os.path.join(root_dir, 'modules', directory)
                    
#
# End class ModuleInfo
#####################################################################################

class SpecializedOptionType( Enum ):
    CMakeBool = 1,
    CMakeString = 2,
    CMakeWarningFlag = 3,
    EnvVarBool = 4,
    EnvVarString = 5

#
# End class SpecializedOptionType
#####################################################################################

class SpecializedCMakeOption:
    def __init__( self, \
                  option_type, \
                  config_var='', \
                  value=None, \
                  help_str='', \
                  modules_to_vars_map={} ):

        assert(option_type == SpecializedOptionType.CMakeBool or
               option_type == SpecializedOptionType.CMakeString or
               option_type == SpecializedOptionType.CMakeWarningFlag)

        self.type = option_type
        self.cmd_line_flag = '--' + config_var
        self.args_attr = config_var.replace('-', '_')
        self.config_var = config_var
        self.help_str = help_str
        self.modules_to_vars_map = modules_to_vars_map
        self.value = value

#
# End class SpecializedCMakeOption
#####################################################################################

class SpecializedEnvVarOption:
    def __init__(self,
                 option_type, \
                 option_name='',
                 environment_var='',
                 value=None,
                 help_str=''):

        assert(option_type == SpecializedOptionType.EnvVarBool or
               option_type == SpecializedOptionType.EnvVarString)

        self.type = option_type
        self.cmd_line_flag = '--' + option_name
        self.args_attr = option_name.replace('-', '_')
        self.env_var = environment_var
        self.value = value
        self.help_str = help_str

#
# End class SpecializedEnvVarOption
#####################################################################################

def parse_options( specialized_options = [], root_repo_name = '' ):

    # Windows requires calling color first before any prints for commandline colors to work.
    if (os.name == 'nt'):
        os.system('color')

    # First figure out what type of module this is being run from
    module_info = ModuleInfo( os.getcwd(), root_repo_name )

    if module_info.root_module_type == ModuleType.TopLevel:
        description = 'A make utility for configuring, building, and testing the executable \
            and submodules of the underlying FEA framework.'
    else:
        description = 'A utility similar to make for configuring, building, and testing the ' \
            + module_info.root_module_name + ' submodule.'
    parser = argparse.ArgumentParser(description=description)
    
    # Commands
    command_group = parser.add_argument_group('Commands')
    command_group.add_argument(
        '--config', action='store_true', help='Configures CMake with the provided (or saved) configuration options')
    command_group.add_argument(
        '--build', action='store_true', help='Build and install')
    command_group.add_argument(
        '--test', action='store_true', help='Run unit tests (requires --make-tests to build tests)')
    command_group.add_argument(
        '--list', action='store_true', help='Print list of tests to select for build or test') 
    command_group.add_argument(
        '--doc', action='store_true', help='Generate the doxygen documentation')
    command_group.add_argument(
        '--clear-cache', action='store_true', help='Clear the CMake cache files and saved config options')
    command_group.add_argument(
        '--clean', action='store_true', help='Delete build and install directories')
    if module_info.root_module_type == ModuleType.TopLevel:
        command_group.add_argument(
            '--copy-make', action='store_true', help='Copy the make_helper.py from the top-level module down to submodules')
    command_group.add_argument(
        '--open-vscode', action='store_true', help='Open module folder with VSCode.')
    if os.name == 'nt': # Option only for Windows
        command_group.add_argument(
            '--open-vs', action='store_true', help='Open Visual Studio solution after configuration.')
    
    # Configuration options
    config_group = parser.add_argument_group('Configuration options')
    config_group.add_argument('--generator', default='', action='store',
                              choices=('VS2015', 'VS2017', 'VS2019',
                                       'make', 'codeblocks'),
                              help='Specify the CMake generator to use.  Default is `VS2017` (Visual Studio 15 2017) for windows with x64 architecture and `make` (Unix Makefiles) for linux')
    config_group.add_argument('--toolset', default='', action='store',
                              choices=('Intel17','Intel18'),
                              help='Specify Intel toolset for use with Visual Studio. Default is the native toolset (MSVC)')
    config_group.add_argument(
        '--mode', type=str, choices=('Debug', 'Release', 'RelWithDebInfo', 'MinSizeRel'),
        default='debug', help='Set the mode to build (debug is default)')
    config_group.add_argument(
        '--make-tests', action='store_true', help='Build the submodule unit-tests')
    config_group.add_argument(
        '--nomake-tests', action='store_true')
    config_group.add_argument(
        '-j', type=int, dest='n_jobs', default=1,
                              help='Specify the number of parallel jobs to run for cmake/ctest (default 1)')
    
    ##################
    # Add special options
    for special_opt in specialized_options:
        if special_opt.type == SpecializedOptionType.CMakeBool and special_opt.value is None:
            config_group.add_argument( special_opt.cmd_line_flag, action='store_true', help=special_opt.help_str )
        elif special_opt.type == SpecializedOptionType.CMakeString and special_opt.value is None:
            config_group.add_argument( special_opt.cmd_line_flag, type=str, action='store_true', help=special_opt.help_str )

    ##################
    # Top-level specifc options
    if module_info.root_module_type == ModuleType.TopLevel:
        select_group = parser.add_argument_group(
            'Module selection (if none are specified, then top is selected)')
        select_group.add_argument(
            '--top', action='store_true', help='Include the top-level project (exe)')
        select_group.add_argument(
            '--submodules', action='store_true', help='Include all first, second, and third party submodules (does not include top-level project)')
        select_group.add_argument(
            '--first-party', action='store_true', help='Include all first-party submodules')
        select_group.add_argument(
            '--second-party', action='store_true', help='Include all second-party submodules')
        select_group.add_argument(
            '--third-party', action='store_true', help='Include all third-party submodules')
        select_group.add_argument(
            '--all', action='store_true', help='Include all submodules (1/2/3-party) and top')

        # Use dependencies python module to add an optoin for each individual dependency
        import dependencies
        for m in dependencies.simple_dependencies.keys():
            select_group.add_argument(
                '--' + m.replace('_', '-'), action='store_true', help='Specifically include the ' + str(m) + ' module')

    ##################
    # Misc. options
    misc_group = parser.add_argument_group('Misc. options')
    misc_group.add_argument('--verbose', '-v', action='store_true',
                            help='Display more info during target execution')
    misc_group.add_argument('--print', action='store_true',
                            help='Print the config/build command that will be executed (for debugging)')
    misc_group.add_argument('--dir-suffix', default='',
                            help='for building with different settings, may want multiple build/install dirs. \
                            Specify a build/install suffix, e.g., --dir=gnu, then build dir = build_gnu \
                            and install dir = install_dir_gnu. This option must come BEFORE any other \
                            config options')
    misc_group.add_argument('-n', type=int, dest='test_num', default=0, help='Select a single test by number (see --list) \
                            to build or test')
    misc_group.add_argument('--name', dest='test_name', default='', help='Select a single test by name to build or test. \
                            String value is case-sensitive and overridden by -n if both used. For build target, do not \
                            include test type (Unit|Regression) or module name. For test target, all tests with the given \
                            substring will be executed.')
    misc_group.add_argument('--shell-executable', default='',
                            help='Overrides the default shell for child processes.')                     

    ##################
    # Add special options for environment variables for misc group
    for special_opt in specialized_options:
        if special_opt.type == SpecializedOptionType.EnvVarBool and special_opt.value is None:
            misc_group.add_argument(
                special_opt.cmd_line_flag, action='store_true', help=special_opt.help_str)
        elif special_opt.type == SpecializedOptionType.EnvVarString:
            misc_group.add_argument(
                special_opt.cmd_line_flag, type=str, default='', help=special_opt.help_str)

    ####################################
    # If no args provided, show help...
    if( len( sys.argv ) == 1 ):
        parser.print_help()
        sys.exit( 1 )
    ####################################

    args = parser.parse_args()

    # From the parsed arguments, create a MakeOptions object with
    # the appropriate properties
    opts = MakeOptions( module_info.root_module_name, module_info.root_module_type )

    # Commands
    if args.config:
        opts.config = True
    if args.build:
        opts.build = True
    if args.test:
        opts.run_tests = True
    if args.list:
        opts.list_tests = True
    if args.clear_cache:
        opts.clear_cache = True
    if args.clean:
        opts.clean = True
    if args.open_vs:
        opts.open_vs = True
    if args.open_vscode:
        opts.open_vscode = True

    # Directory suffix option: This MUST come before the other options as dir_suffix affects 
    # build location and therefore saved config options (since we may want to save different 
    # config options for different build directories). Note that we only need to know the root 
    # build dir, as local calls to config will load their own local root build dir config opts. 
    # We then load any stored options before overriding with current options.
    if args.dir_suffix != '':
        opts.dir_suffix = '_' + args.dir_suffix
    # Define root build dir:
    if module_info.root_module_type == ModuleType.ThirdParty:
        opts.root_build_dir = os.path.normpath(os.path.join(module_info.root_module_dir, '..', \
          module_info.root_module_name + '_build' + opts.dir_suffix)).replace('\\', '/')
    else:
        opts.root_build_dir = os.path.join(module_info.root_module_dir, 'build' + opts.dir_suffix).replace('\\', '/')
    # Load previous config options if available
    opts.load()

    # Configuration options
    if args.generator == '':
        args.generator = find_generator()
    if args.generator == 'VS2015':
        opts.cmake_opts['generator'] = 'Visual Studio 14 2015'
    elif args.generator == 'VS2017':
        opts.cmake_opts['generator'] = 'Visual Studio 15 2017'
    elif args.generator == 'VS2019':
        opts.cmake_opts['generator'] = 'Visual Studio 16 2019'
    elif args.generator == 'make':
        opts.cmake_opts['generator'] = 'Unix Makefiles'
    elif args.generator == 'codeblocks':
        opts.cmake_opts['generator'] = 'CodeBlocks - Unix Makefiles'
    if args.toolset != '':
        if 'Visual Studio' in opts.cmake_opts['generator']:
            if args.toolset == 'Intel18':
                opts.cmake_opts['toolset'] = '-T"Intel C++ Compiler 18.0",host=x64'
    opts.cmake_opts['build-type'] = args.mode
    if args.make_tests:
        opts.cmake_opts['make-tests'] = 'ON'
    if args.nomake_tests:
        opts.cmake_opts['make-tests'] = 'OFF'

    opts.n_jobs = args.n_jobs

    # Specialied configuration options
    for special_opt in specialized_options:
        if special_opt.type == SpecializedOptionType.CMakeBool:
            if special_opt.value is None:
                if getattr(args, special_opt.args_attr):
                    opts.special_cmake_opts[special_opt.config_var] = 'ON'
                elif not opts.special_cmake_opts.get(special_opt.config_var):
                    # If not already defined by loaded config file, then default to OFF
                    opts.special_cmake_opts[special_opt.config_var] = 'OFF'
            else:
                assert(special_opt.value == 'ON' or special_opt.value == 'OFF')
                opts.special_cmake_opts[special_opt.config_var] = special_opt.value
        elif special_opt.type == SpecializedOptionType.CMakeString:
            if special_opt.value is None:
                opts.special_cmake_opts[special_opt.config_var] = getattr(args, special_opt.args_attr)
            else:
                opts.special_cmake_opts[special_opt.config_var] = special_opt.value
        elif special_opt.type == SpecializedOptionType.EnvVarBool:
            if special_opt.value is None:
                if getattr(args, special_opt.args_attr):
                    os.environ[special_opt.env_var] = 'ON'
            else:
                assert(special_opt.value == 'ON' or special_opt.value == 'OFF')
                os.environ[special_opt.env_var] = special_opt.value
        elif special_opt.type == SpecializedOptionType.EnvVarString:
            if special_opt.value is None:
                os.environ[special_opt.env_var] = getattr(args, special_opt.args_attr)
            else:
                os.environ[special_opt.env_var] = special_opt.value

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
        if args.all:
            opts.include_top = True
            opts.include_first_party = True
            opts.include_second_party = True
            opts.include_third_party = True
        if args.all and args.clean:
            opts.clean_toolchain = True
        if args.first_party:
            opts.include_first_party = True
        if args.second_party:
            opts.include_second_party = True
        if args.third_party:
            opts.include_third_party = True
        for m in dependencies.simple_dependencies.keys():
            if getattr(args, m):
                opts.include_specific_modules.append(m)
        if not args.top and not args.submodules and not args.first_party \
            and not args.second_party and not args.third_party \
            and len(opts.include_specific_modules) == 0:
            opts.include_top = True
    # Misc. options
    if args.verbose:
        opts.verbose = True
        opts.print_cmd = True

    if args.print:
        opts.print_cmd = True

        opts.print()
        print( module_info )
   
    # Can't select both num and name
    if args.test_num:
        opts.test_num = args.test_num
    elif args.test_name:
        opts.test_name = args.test_name

    if args.shell_executable:
        opts.shell_executable = args.shell_executable

    return opts, module_info

#####################################################################################

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

#####################################################################################

def clean( selected_modules_dirs, opts ):

    """Deletes the build and install directories"""
    print_color("Cleaning...", "yellow")

    for module_dir in selected_modules_dirs:

        print_if_verbose( "    " + module_dir.build_dir + ": ", opts )

        if( os.path.exists( module_dir.build_dir ) ):
            shutil.rmtree( module_dir.build_dir )
            print_if_verbose( "        ...Deleting directory: " + module_dir.build_dir, opts )

        if( os.path.exists( module_dir.install_dir ) ):
            shutil.rmtree( module_dir.install_dir )
            print_if_verbose( "        ...Deleting directory: " + module_dir.install_dir, opts )

        # Also delete compiled python files
        if (os.path.exists(os.path.join(module_dir.module_dir, '__pycache__'))):
            shutil.rmtree(os.path.join(module_dir.module_dir, '__pycache__'))
            print_if_verbose( "        ...Deleting __pycache__", opts )

        if (os.path.exists(os.path.join(module_dir.module_dir, 'make_helper.pyc'))):
            os.remove(os.path.join(module_dir.module_dir, 'make_helper.pyc'))
            print_if_verbose( "        ...Deleting make_helper.pyc", opts )

        if (os.path.exists(os.path.join(module_dir.module_dir, 'dependencies.pyc'))):
            os.remove(os.path.join(module_dir.module_dir, 'dependencies.pyc'))
            print_if_verbose( "        ...Deleting dependencies.pyc", opts )

#####################################################################################

def config( opts, module_dir, specialized_options = [] ):

    # Only check for proper configuration of 2nd and 3rd party when configuring at the top level.
    if( module_dir.module_type == ModuleType.TopLevel ) :

        # If this is a First Party module, verify that 2nd and 3rd party config have already taken place.
        if( module_dir.module_type == ModuleType.FirstParty ):
            # Assume that if the 2P and 3P build dirs do not exist, then they have not been configured.
            error_occurred = False
            if( not os.path.exists( opts.root_module_dir + "/modules/second_party/second_party_math/build" ) ):
                print( "Error: Second_Party modules have not been configured yet." )
                print( "       Add --second-party to make.py command line." )
                error_occurred = True
            if( not os.path.exists( opts.root_module_dir + "/modules/third_party/json_build" ) ):
                print( "Error: Third_Party modules have not been configured yet." )
                print( "       Add --third-party to make.py command line." )
                error_occurred = True
            if( error_occurred ):
                sys.exit()
        
        # If this is a Second Party module, verify that the 3rd party config has already taken place.
        if( module_dir.module_type == ModuleType.SecondParty ):
            if( not os.path.exists( opts.root_module_dir + "/modules/third_party/json_build" ) ):
                print( "Error: Third_Party modules have not been configured yet." )
                print( "       Add --third-party to make.py command line." )
                sys.exit()
        
    # Make sure the build and install directories exist
    if( not os.path.exists( module_dir.build_dir ) ):
        os.mkdir( module_dir.build_dir )

    if( not os.path.exists( module_dir.install_dir ) ):
        os.mkdir( module_dir.install_dir )

    if( module_dir.module_type == ModuleType.TopLevel ):
        # First, save options to be reloaded on next config
        opts.save()
        
    os.chdir( module_dir.build_dir )

    write_log = module_dir.module_dir != opts.root_module_dir

    if opts.verbose:
        log = " 2>&1 | tee cmake.log"
    else:
        log = " > cmake.log 2>&1"

    module_name_caps = module_dir.name.upper().replace('_', '')

    vflag = ""
    if( opts.verbose ):
        vflag = " --verbose "

    cmake_arch_flag = ''
    if opts.cmake_opts['architecture'] != '':
        cmake_arch_flag =  '-A"' + opts.cmake_opts['architecture'] + '" '

    cmake_flags = " -DCMAKE_BUILD_TYPE=" + opts.cmake_opts["build-type"] + " -j " + str( opts.n_jobs ) + vflag

    if module_dir.module_type != ModuleType.ThirdParty:
        cmake_flags += ' -D' + module_name_caps + '_BuildTests:BOOL=' + opts.cmake_opts["make-tests"]

    for special_opt in specialized_options:
        if special_opt.type != SpecializedOptionType.EnvVarBool and \
           special_opt.type != SpecializedOptionType.EnvVarString:
            
            if module_dir.name in special_opt.modules_to_vars_map.keys():
                for var in special_opt.modules_to_vars_map[module_dir.name]:
                    if special_opt.type == SpecializedOptionType.CMakeBool:
                        if special_opt.value is None:
                            cmake_flags += ' -D' + var + ':BOOL=' + opts.special_cmake_opts[special_opt.config_var]
                        else:
                            cmake_flags += ' -D' + var + ':BOOL=' + special_opt.value
                    elif special_opt.type == SpecializedOptionType.CMakeString:
                        if special_opt.value is None:
                            cmake_flags += ' -D' + var + '="' + opts.special_cmake_opts[special_opt.config_var] + '"'
                        else:
                            cmake_flags += ' -D' + var + '="' + special_opt.value + '"'
                    elif special_opt.type == SpecializedOptionType.CMakeWarningFlag:
                        cmake_flags += ' -W' + var

    print_color( "Configuring module: " + module_dir.name, "blue" )
    cmd = mod_for_os( opts.cmake_cmd + ' -G"' + opts.cmake_opts['generator'] + '" ' + cmake_arch_flag +
                      opts.cmake_opts['toolset'] + ' --no-warn-unused-cli ' +  
                      ' -DCMAKE_INSTALL_PREFIX:PATH="' + module_dir.install_dir + '" ' + 
                      cmake_flags + ' "' + module_dir.module_dir + '" ' + log,
                      opts.shell_executable )

    if opts.print_cmd:
        print_color( cmd, "cyan" )

    config_flag = subprocess.call(cmd, shell=True, executable=opts.shell_executable)

    if config_flag != 0:
        if write_log:
            with open( "cmake.log", 'r', encoding=get_write_encoding() ) as fin:
                    file_str = fin.read()
                    
                    # Note: the string "CMake Error is looked for because warnings will still generate
                    # the words containing the text "Error" in the description of the warnings (for some reason).
                    if "CMake Error" in file_str:
                        print( file_str )
                        print_color("   ERROR during configuration. (See cmake.log for details)", "red")
                        sys.exit()
                    else:
                        print_color("   CMake had warnings, see cmake.log for details.", "cyan")
        else:
            print_color("   ERROR during configuration. (See cmake.log for details)", "red")

#
# End config()
#####################################################################################

def build( opts, module_dir ):

    # toolchain is config-only
    if (module_dir.name == "toolchain"):
        return

    if os.path.exists( module_dir.build_dir):
        os.chdir( module_dir.build_dir )
    else:
        print("   Warning: Cannot build before config")
        return

    write_log = module_dir.module_dir != opts.root_module_dir
    #if write_log:
    #    log = " > build.log 2>&1"

    if( opts.verbose or opts.root_module_type != ModuleType.TopLevel ):
        log = " 2>&1 | tee build.log"
    else:
        log = " > build.log 2>&1"

    print_color("Building---------------" + module_dir.name, "blue")

    vflag = ""
    if( opts.verbose ):
        vflag = " --verbose "

    # Standard build command 
    cmd = mod_for_os( opts.cmake_cmd + ' --build . --config ' + opts.cmake_opts["build-type"] +
                      ' --target install -j ' + str( opts.n_jobs ) + vflag + log,
                      opts.shell_executable )

    # Alternatively, if a single test is requested, we modify this commmand for that test. On Linux, this uses GNU make
    # as the make program. On Windows, could use nmake, but for now we just assume linux...
    if opts.test_num:
        # In this case, check that we've already built so can get corresponding test name. Even if we haven't built
        # yet, the code below should return a blank target and thus build all 
        if os.path.exists( os.path.join(module_dir.build_dir,"build.log")):
            # Then get name from test list 
            get_test_list = opts.ctest_cmd + " -N -I "+str(opts.test_num)+","+str(opts.test_num)
            test_str = subprocess.check_output(get_test_list.split())
            test_str = test_str.decode( 'ascii' ).split('\n')[1]
            proj_name = (module_dir.name).replace('_', ' ').title().replace(' ', '')
            pos = test_str.find(proj_name)
            target=test_str[pos+len(proj_name)+1:]
            cmd = mod_for_os( 'make ' + target + log, opts.shell_executable )
    elif opts.test_name:
        # Otherwise, named targets still work, and will build dependencies
        cmd = mod_for_os( 'make ' + opts.test_name + log, opts.shell_executable )

    if( opts.print_cmd ):
        print_color( cmd, "cyan" )

    build_flag = subprocess.call(cmd, shell = True, executable=opts.shell_executable)

    if build_flag != 0:
        if write_log:
            with open("build.log", 'r', encoding=get_write_encoding()) as fin:
                print(fin.read()) 
        print_color( "   ERROR during build", "red" )

#####################################################################################

def test( opts, selected_modules_dirs ):

    vflag = ""
    if( opts.verbose ):
        vflag = " --verbose "

    for module_dir in selected_modules_dirs:

        print_color( "\n------------------------------------------------------", "blue" )
        print_color( "---- Testing: " + module_dir.name, "blue" )
        print_color( "", "blue" )

        if os.path.exists( os.path.join(module_dir.build_dir,"build.log") ):
            os.chdir(module_dir.build_dir)
        else:
            print("   Warning: Cannot test before build")
            continue

        cmd = mod_for_os( opts.ctest_cmd + ' ' + vflag + ' -C ' + opts.cmake_opts["build-type"] +
                          ' -j ' + str(opts.n_jobs), opts.shell_executable )
        if opts.test_num:
            cmd += " -I "+str(opts.test_num)+","+str(opts.test_num)
        elif opts.test_name:
            cmd += " -R "+opts.test_name

        test_flag = subprocess.call(cmd, shell = True, executable=opts.shell_executable)

        if( test_flag != 0 ):
            print_color("   ERROR during unit testing", "red")


#####################################################################################

def open_vs( opts, module_dir ):
    """Opens the solution project for the given module."""

    cmd = '"' + os.path.join(module_dir.build_dir,
                             module_dir.name.replace('_', '') + '.sln"')
    subprocess.Popen(cmd, shell = True, executable=opts.shell_executable)

#####################################################################################

def open_vscode( opts, module_dir ):
    """Opens the root directory of the given module in VS Code."""

    subprocess.Popen('code "' + module_dir.module_dir + '"',
                     shell = True, executable=opts.shell_executable)

#####################################################################################

def list_tests( opts, selected_modules_dirs ):

    vflag = ""
    if( opts.verbose ):
        vflag = " --verbose "

    for module_dir in selected_modules_dirs:
        print_color("Test List----------------" + module_dir.name, "blue")
        if os.path.exists( os.path.join(module_dir.build_dir,"build.log")):
            os.chdir(module_dir.build_dir)
        else:
            print("   Warning: Cannot list tests before build")
            continue

        cmd = mod_for_os( opts.ctest_cmd + ' -N', opts.shell_executable )

        test_flag = subprocess.call(cmd, shell = True, executable=opts.shell_executable)

        if (test_flag != 0):
            print_color("   ERROR during list testing", "red")

#####################################################################################

def copy_make(opts, selected_modules_dirs):
    if opts.root_module_type == ModuleType.TopLevel:
        for module_dir in selected_modules_dirs:
            if module_dir.module_type != ModuleType.TopLevel:
                print_color('Copying make_helper.py into ' + module_dir.name, "cyan")
                shutil.copy(src=os.path.join(opts.root_module_dir, 'make_helper.py'), \
                    dst=os.path.join(module_dir.module_dir, 'make_helper.py'))

#####################################################################################

def config_version_header( mod_info, shell_executable ):

    top_dir = os.getcwd()

    with open(os.path.join(top_dir, "include", "version.h.in"), 'r') as header_template:
        header_str = header_template.read()
   
    # Warn not to modify updated .h file 
    header_str = header_str.replace( '${OVERWRITE_WARNING}', 
      "DO NOT MODIFY. Automatically configured from version.h.in" )

    # Get top-level version from CMake
    with (open("CMakeLists.txt")) as cmake:
        cmake_str = cmake.read()

    version = re.search('VERSION (.*) LANGUAGES', cmake_str).group(1)
    header_str = header_str.replace('${VERSION}', version)

    # Get module hashes
    hash_str = "{\n"
    for i, mods in enumerate([{mod_info.root_module_name : mod_info.root_module_dir}, 
      mod_info.first_party_submodules, mod_info.second_party_submodules, mod_info.third_party_submodules]): 
        for k in mods:  
            os.chdir( mods[k] )

            # Popen/check_output doesn't seem to like set -o pipefail; 
            #cmd = mod_for_os( "git log --pretty=format:'%h' -1" )
            cmd = "git log --pretty=format:'%h' -1" 

            mod_hash = subprocess.check_output(cmd.split())
            mod_hash = mod_hash.decode( 'ascii' ).replace("'","") 
            hash_str += '      {%i, "%s", "%s"},\n' % (i, k, mod_hash)

    os.chdir( mod_info.root_module_dir )
    hash_str += "    }"
    header_str = header_str.replace( '${MODULE_INFO}', hash_str )

    # Get build (or config) time (assumes using make_helper to build...) 
    timestamp = datetime.datetime.now()
    header_str = header_str.replace('${TIMESTAMP}', timestamp.strftime("%m/%d/%Y %H:%M:%S"))

    # Finally write the full string to the header file
    with open(os.path.join(top_dir, "include", "version.h"), 'w') as header_file:
        header_file.write( header_str )


#####################################################################################
# Verify that python and cmake versions are new enough...

def validate_cmake_and_python():

    if sys.version_info[0] < 3:
        sys.tracebacklimit =  0
        print( "" ) # Print blank line first...
        raise Exception( "This script must use Python version 3+...  You are using version " + \
                                     str( sys.version_info[0] ) + "." + str( sys.version_info[1] ) + ".\n" )

    ##############################################################################
    # Verify cmake is new enough...

    # On some linux installs, the CMake version 3 executable is actually called cmake3, 
    # so we check for this first. 
    cmake_cmd = "cmake3"
    try:
        cmake_output = subprocess.check_output( "cmake3 --version", shell=True, stderr=subprocess.STDOUT ).decode( "utf-8" )
    except:
        cmake_cmd = "cmake"
        try:
            cmake_output = subprocess.check_output( "cmake --version", shell=True, stderr=subprocess.STDOUT ).decode( "utf-8" )
        except:
            print( "\nERROR: cmake command not found.  Please check your path.\n" )
            sys.exit(1)

    cmake_version_full = re.search( cmake_cmd + " version (\d+\.\d+\.\d+\.?\d?\d?\d?)", cmake_output )
    if( cmake_version_full == None ):
        print( "" )
        print( "Internal Error: could not find/parse the " + cmake_cmd + " version in the following string:\n\n", cmake_output )
        sys.exit( 1 )
    
    cmake_version = re.findall( "\d+", cmake_version_full.group(1) )

    if( len(cmake_version) < 3 ):
        print( "" )
        print( "Internal Error: could not parse " + cmake_cmd + " version (looking for at least 3 numbers).  Found: ", cmake_version, "\n" )
        sys.exit( 1 )

    # FIXME: What is the actual minimum version of cmake that we need?
    if( ( int(cmake_version[0]) < 3 ) or ( int(cmake_version[1]) < 12 ) ):
        print( "" )
        print( "ERROR: CMake is too old (Found " + cmake_version_full.group(0) + ").\n" )
        sys.exit( 1 )

    return cmake_cmd


#####################################################################################

def make( specialized_opts, build_order=[], root_repo_name='' ):
    """The kernel of the make process.  Call this with any specialized options that may
    be needed by the particular module(s).  If build_order is provided, then the modules
    will be sorted by that order for all operations."""

    cmake_cmd = validate_cmake_and_python()

    opts, module_info = parse_options( specialized_opts, root_repo_name )
    if (cmake_cmd == "cmake3"):
      opts.cmake_cmd = "cmake3"
      opts.ctest_cmd = "ctest3"

    selected_modules = opts.get_selected_module_dirs( module_info )
    print_color('Selected modules: ' + ', '.join([s.name for s in selected_modules]), 'blue')

    if len(build_order) != 0:
        selected_modules.sort(key=lambda x: build_order.index(x.name))

    if opts.clear_cache:
        clear_cache( selected_modules )

    if (opts.clean_toolchain and (os.name == "posix")):
        # Clean current toolchain dir. If toolchain was used to config before building third party,
        # this should be a symbolic link to the compiler-specific dir. Otherwise, it is just a dir.
        toolchain_dir = os.path.join(os.path.expanduser("~"), ".cmake", "packages") 
        if os.path.exists(toolchain_dir):
            if os.path.islink(toolchain_dir):
                link_path = os.readlink(toolchain_dir)
                os.remove(toolchain_dir)
                shutil.rmtree(link_path) 
            else:
                shutil.rmtree(toolchain_dir)

    if opts.clean:
        clean( selected_modules, opts )

    # If config, build, or doc: create the two needed headers from .h.in files
    if ( opts.config or opts.build or opts.doc ): 
   
        if ( opts.root_module_type == ModuleType.TopLevel ):
            # First update the version.h header
            config_version_header( module_info, opts.shell_executable )


    if opts.config:
        for module_dir in selected_modules:
            config( opts, module_dir, specialized_opts )
            if opts.build:
                build( opts, module_dir )

    if opts.build and not opts.config:
      for module_dir in selected_modules:
        build( opts, module_dir )

    if opts.run_tests:
        test( opts, selected_modules )

    if opts.open_vs:
        for module_dir in selected_modules:
            open_vs( opts, module_dir )

    if opts.open_vscode:
        for module_dir in selected_modules:
            open_vscode( opts, module_dir )

    if opts.list_tests:
        list_tests( opts, selected_modules )

    if module_info.root_module_type == ModuleType.TopLevel:
        if opts.copy_make:
            copy_make( opts, selected_modules )

#####################################################################################

# Example usage:
"""
import make_helper
from make_helper import SpecializedCMakeOption

specialized_opts = [
    SpecializedCMakeOption('make-paraview-reader', 'Build the ParaView reader (only applies to fea_io)', { 'fea_io': 'FEAIO_BuildParaViewReader' }),
    SpecializedCMakeOption('use-mpi', 'Build with MPI (requires MPI environment variables for CMake to find it)', { 'fea_io': 'FEAIO_USE_MPI' }),
    ]

make_helper.make( specialized_opts )
"""

# Execute main on initial import or script call:
if __name__=="__main__":
   print("Don't call this file directly, use make.py instead") 
