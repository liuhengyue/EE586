echo off
rem This file is called from top-level build.bat to build bsl examples
rem This script can be run stand-alone if you initialize the 'TIROOT', 'TCONF'
rem and TIMAKE variables.
echo on

set TIROOT=%TI_DIR%
set TCONF=%TIROOT%\bin\utilities\tconf\tconf
set TIMAKE=%TIROOT%\cc\bin\timake

%TIMAKE% video_loopback.pjt Debug

call cleanup.bat
