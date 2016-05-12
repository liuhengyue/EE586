echo off
rem Cleanup intermediate files generated from CCStudio
echo on

erase .\Debug\*.obj
erase *.paf2
erase *.sbl
erase *.lkf
erase *.lkv
erase cc_build*.log
