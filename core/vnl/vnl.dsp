# Microsoft Developer Studio Project File - Name="vnl" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=vnl - Win32 DebugSTLPort
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "vnl.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "vnl.mak" CFG="vnl - Win32 DebugSTLPort"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "vnl - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "vnl - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "vnl - Win32 StaticDebug" (based on "Win32 (x86) Static Library")
!MESSAGE "vnl - Win32 StaticRelease" (based on "Win32 (x86) Static Library")
!MESSAGE "vnl - Win32 DebugSTLPort" (based on "Win32 (x86) Static Library")
!MESSAGE "vnl - Win32 ReleaseSTLPort" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=xicl6.exe
RSC=rc.exe

!IF  "$(CFG)" == "vnl - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /Ob2 /I "$(IUEROOT)\vcl\config.win32-vc60" /I "$(IUEROOT)\vcl" /I "$(IUEROOT)\vxl" /D "WIN32" /D "NDEBUG" /D "_LIB" /YX /FD /c /Zl
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=xilink6.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\Release\vnl.lib"

!ELSEIF  "$(CFG)" == "vnl - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 2
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /Zi /Od /Ob2 /I "$(IUEROOT)\vcl\config.win32-vc60" /I "$(IUEROOT)\vcl" /I "$(IUEROOT)\vxl" /D "WIN32" /D "_DEBUG" /D "_LIB" /FD /GZ /c /Zl
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=xilink6.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\Debug\vnl.lib"

!ELSEIF  "$(CFG)" == "vnl - Win32 StaticDebug"

# PROP BASE Use_MFC 2
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "vnl___Win32_StaticDebug"
# PROP BASE Intermediate_Dir "vnl___Win32_StaticDebug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 2
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "StaticDebug"
# PROP Intermediate_Dir "StaticDebug"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /Gm /GX /Zi /Od /Ob2 /D "WIN32" /D "_DEBUG" /D "_LIB" /FD /GZ /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /Zi /Od /Ob2 /I "$(IUEROOT)\vcl\config.win32-vc60" /I "$(IUEROOT)\vcl" /I "$(IUEROOT)\vxl" /D "WIN32" /D "_DEBUG" /D "_LIB" /FD /GZ /c /Zl
# ADD BASE RSC /l 0x409 /d "_DEBUG" /d "_AFXDLL"
# ADD RSC /l 0x409 /d "_DEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=xilink6.exe -lib
# ADD BASE LIB32 /nologo /out:"..\Debug\vnl.lib"
# ADD LIB32 /nologo /out:"..\StaticDebug\vnl.lib"

!ELSEIF  "$(CFG)" == "vnl - Win32 StaticRelease"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "vnl___Win32_StaticRelease"
# PROP BASE Intermediate_Dir "vnl___Win32_StaticRelease"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "StaticRelease"
# PROP Intermediate_Dir "StaticRelease"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /Ob2 /D "WIN32" /D "NDEBUG" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MT /W3 /GX /O2 /Ob2 /I "$(IUEROOT)\vcl\config.win32-vc60" /I "$(IUEROOT)\vcl" /I "$(IUEROOT)\vxl" /D "WIN32" /D "NDEBUG" /D "_LIB" /YX /FD /c /Zl
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=xilink6.exe -lib
# ADD BASE LIB32 /nologo /out:"..\Release\vnl.lib"
# ADD LIB32 /nologo /out:"..\StaticRelease\vnl.lib"

!ELSEIF  "$(CFG)" == "vnl - Win32 DebugSTLPort"

# PROP BASE Use_MFC 2
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "vnl___Win32_DebugSTLPort"
# PROP BASE Intermediate_Dir "vnl___Win32_DebugSTLPort"
# PROP BASE Target_Dir ""
# PROP Use_MFC 2
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug-STLPort"
# PROP Intermediate_Dir "Debug-STLPort"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /Gm /GX /Zi /Od /Ob2 /D "WIN32" /D "_DEBUG" /D "_LIB" /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /Zi /Od /Ob2 /I "$(STLPORT)\stlport" /I "$(IUEROOT)\vcl\config.stlport.win32-vc60" /I "$(IUEROOT)\vcl" /I "$(IUEROOT)\vxl" /D "WIN32" /D "_DEBUG" /D "_LIB" /FD /GZ /c /Zl
# ADD BASE RSC /l 0x409 /d "_DEBUG" /d "_AFXDLL"
# ADD RSC /l 0x409 /d "_DEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=xilink6.exe -lib
# ADD BASE LIB32 /nologo /out:"..\Debug\vnl.lib"
# ADD LIB32 /nologo /out:"..\Debug-STLPort\vnl.lib"

!ELSEIF  "$(CFG)" == "vnl - Win32 ReleaseSTLPort"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "vnl___Win32_ReleaseSTLPort"
# PROP BASE Intermediate_Dir "vnl___Win32_ReleaseSTLPort"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release-STLPort"
# PROP Intermediate_Dir "Release-STLPort"
# PROP Target_Dir ""
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /Ob2 /D "WIN32" /D "NDEBUG" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /Ob2 /I "$(STLPORT)\stlport" /I "$(IUEROOT)\vcl\config.stlport.win32-vc60" /I "$(IUEROOT)\vcl" /I "$(IUEROOT)\vxl" /D "WIN32" /D "NDEBUG" /D "_LIB" /YX /FD /c /Zl
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=xilink6.exe -lib
# ADD BASE LIB32 /nologo /out:"..\Release\vnl.lib"
# ADD LIB32 /nologo /out:"..\Release-STLPort\vnl.lib"

# Begin Target

# Name "vnl - Win32 Release"
# Name "vnl - Win32 Debug"
# Name "vnl - Win32 StaticDebug"
# Name "vnl - Win32 StaticRelease"
# Name "vnl - Win32 DebugSTLPort"
# Name "vnl - Win32 ReleaseSTLPort"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=".\Templates\vcl_list+vnl_double_4-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_double_3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_double_3x3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_float_2-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_float_3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vcl_vector+vnl_double_3x4--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vcl_vector+vnl_float_2--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_double_2-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_double_4-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_matrix+double--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_matrix+double-~-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_matrix+float--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_matrix_fixed+double.3.4--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_real_npolynomial~-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_vector+double--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_vector+double-~-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_vector+vcl_complex+double---.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_vector+float--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vcl_vector+vnl_vector+float-~-.cxx"
# End Source File
# Begin Source File

SOURCE=.\vnl_alloc.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_binary.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_block.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+double_complex-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+float-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+float_complex-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+int-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+long-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+schar-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+uchar-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+uint-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_c_vector+ulong-.cxx"
# End Source File
# Begin Source File

SOURCE=.\vnl_complex.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_complex_ops+double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_complex_ops+float-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_complex_ops+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_complex_traits.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_copy.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_cost_function.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_cross_product_matrix.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_determinant+double-.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_diag_matrix+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_diag_matrix+vcl_complex+double--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_diag_matrix+float-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_diag_matrix+vcl_complex+float--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_diag_matrix+int-.cxx"
# End Source File
# Begin Source File

SOURCE=.\vnl_double_2.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_double_2x3.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_double_3.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_double_3x2.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_double_4.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_error.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_fastops.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_file_matrix+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_file_matrix+float-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_file_vector+double-.cxx"
# End Source File
# Begin Source File

SOURCE=.\vnl_float_2.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_float_3.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_float_4.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_fortran_copy+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_fortran_copy+vcl_complex+double--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_fortran_copy+float-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_fortran_copy+vcl_complex+float--.cxx"
# End Source File
# Begin Source File

SOURCE=.\vnl_identity_3x3.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_int_2.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_int_3.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_int_4.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_int_matrix.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_least_squares_cost_function.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_least_squares_function.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_linear_operators_3.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_linear_system.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_math.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_filewrite.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_header.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_print.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_print2.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_print_format.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_print_scalar.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_read.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_write.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_matops.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matlab_print+vcl_complex+double--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+float-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matlab_print+vcl_complex+float--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+int-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+long-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+schar-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+uchar-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+uint-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix+ulong-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_exp+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+double.2.2-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+double.2.3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+double.3.12-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+double.3.3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+double.3.4-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+double.4.3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+double.4.4-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed+float.3.3-.cxx"
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matrix_fixed_pairwise_ops.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed_ref+double.2.2-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed_ref+double.2.3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed_ref+double.3.12-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed_ref+double.3.3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed_ref+double.3.4-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed_ref+double.4.3-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_fixed_ref+double.4.4-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_ref+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_matrix_ref+float-.cxx"
# End Source File
# Begin Source File

SOURCE=.\vnl_nonlinear_minimizer.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_numeric_limits.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_numeric_traits.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_quaternion+double-.cxx"
# End Source File
# Begin Source File

SOURCE=.\vnl_real_npolynomial.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_real_polynomial.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_rotation_matrix.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_sample.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_sparse_matrix+double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_sparse_matrix+float-.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_sparse_matrix_linear_system.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_test.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_trace.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_transpose.cxx
# End Source File
# Begin Source File

SOURCE=.\vnl_unary_function.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+vcl_complex+double--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+float-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+vcl_complex+float--.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+int-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+long-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+schar-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+uchar-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+uint-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector+ulong-.cxx"
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+int.1-.cxx
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector_ref+double-.cxx"
# End Source File
# Begin Source File

SOURCE=".\Templates\vnl_vector_ref+int-.cxx"
# End Source File
# Begin Source File

SOURCE=.\Templates\vcl_vector+vcl_vector+vnl_sparse_matrix_pair+double---.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vcl_vector+vcl_vector+vnl_sparse_matrix_pair+float---.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vcl_vector+vnl_matrix_fixed+float.3.4--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vcl_vector+vnl_sparse_matrix_pair+double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vcl_vector+vnl_sparse_matrix_pair+float--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_c_vector+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_c_vector+long_double_complex-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_determinant+float-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_determinant+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_determinant+vcl_complex+double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_determinant+vcl_complex+float--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_determinant+vcl_complex+long_double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_diag_matrix+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_file_matrix+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_fortran_copy+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_fortran_copy+vcl_complex+long_double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matlab_print+double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matlab_print+float-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matlab_print+int-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matlab_print+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matlab_print+vcl_complex+long_double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matrix+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matrix+vcl_complex+double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matrix+vcl_complex+float--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matrix+vcl_complex+long_double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_matrix_fixed+double.2.6-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector+long_double-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector+vcl_complex+long_double--.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+double.1-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+double.2-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+double.3-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+double.4-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+double.6-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+float.1-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+float.2-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+float.3-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+float.4-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+int.2-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+int.3-.cxx
# End Source File
# Begin Source File

SOURCE=.\Templates\vnl_vector_fixed+int.4-.cxx
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\dll.h
# End Source File
# Begin Source File

SOURCE=.\vnl_alloc.h
# End Source File
# Begin Source File

SOURCE=.\vnl_block.h
# End Source File
# Begin Source File

SOURCE=.\vnl_c_vector.h
# End Source File
# Begin Source File

SOURCE=.\vnl_complex.h
# End Source File
# Begin Source File

SOURCE=.\vnl_complex_ops.h
# End Source File
# Begin Source File

SOURCE=.\vnl_complex_traits.h
# End Source File
# Begin Source File

SOURCE=.\vnl_copy.h
# End Source File
# Begin Source File

SOURCE=.\vnl_cost_function.h
# End Source File
# Begin Source File

SOURCE=.\vnl_cross_product_matrix.h
# End Source File
# Begin Source File

SOURCE=.\vnl_determinant.h
# End Source File
# Begin Source File

SOURCE=.\vnl_diag_matrix.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_2.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_2x2.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_2x3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_3x3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_3x4.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_4.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_4x3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_double_4x4.h
# End Source File
# Begin Source File

SOURCE=.\vnl_error.h
# End Source File
# Begin Source File

SOURCE=.\vnl_fastops.h
# End Source File
# Begin Source File

SOURCE=.\vnl_file_matrix.h
# End Source File
# Begin Source File

SOURCE=.\vnl_file_vector.h
# End Source File
# Begin Source File

SOURCE=.\vnl_float_2.h
# End Source File
# Begin Source File

SOURCE=.\vnl_float_3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_float_4.h
# End Source File
# Begin Source File

SOURCE=.\vnl_fortran_copy.h
# End Source File
# Begin Source File

SOURCE=.\vnl_fwd.h
# End Source File
# Begin Source File

SOURCE=.\vnl_identity.h
# End Source File
# Begin Source File

SOURCE=.\vnl_identity_3x3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_int_2.h
# End Source File
# Begin Source File

SOURCE=.\vnl_int_3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_int_4.h
# End Source File
# Begin Source File

SOURCE=.\vnl_int_matrix.h
# End Source File
# Begin Source File

SOURCE=.\vnl_least_squares_cost_function.h
# End Source File
# Begin Source File

SOURCE=.\vnl_least_squares_function.h
# End Source File
# Begin Source File

SOURCE=.\vnl_linear_operators_3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_linear_system.h
# End Source File
# Begin Source File

SOURCE=.\vnl_math.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_filewrite.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_header.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_print.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_print2.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_read.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matlab_write.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matops.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matrix.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matrix_exp.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matrix_fixed.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matrix_fixed_ref.h
# End Source File
# Begin Source File

SOURCE=.\vnl_matrix_ref.h
# End Source File
# Begin Source File

SOURCE=.\vnl_nonlinear_minimizer.h
# End Source File
# Begin Source File

SOURCE=.\vnl_numeric_limits.h
# End Source File
# Begin Source File

SOURCE=.\vnl_numeric_limits_double.h
# End Source File
# Begin Source File

SOURCE=.\vnl_numeric_limits_float.h
# End Source File
# Begin Source File

SOURCE=.\vnl_numeric_limits_int.h
# End Source File
# Begin Source File

SOURCE=.\vnl_numeric_traits.h
# End Source File
# Begin Source File

SOURCE=.\vnl_quaternion.h
# End Source File
# Begin Source File

SOURCE=.\vnl_real_npolynomial.h
# End Source File
# Begin Source File

SOURCE=.\vnl_real_polynomial.h
# End Source File
# Begin Source File

SOURCE=.\vnl_resize.h
# End Source File
# Begin Source File

SOURCE=.\vnl_rotation_matrix.h
# End Source File
# Begin Source File

SOURCE=.\vnl_sample.h
# End Source File
# Begin Source File

SOURCE=.\vnl_scalar_join_iterator.h
# End Source File
# Begin Source File

SOURCE=.\vnl_scatter_3x3.h
# End Source File
# Begin Source File

SOURCE=.\vnl_sparse_matrix.h
# End Source File
# Begin Source File

SOURCE=.\vnl_sparse_matrix_linear_system.h
# End Source File
# Begin Source File

SOURCE=.\vnl_T_n.h
# End Source File
# Begin Source File

SOURCE=.\vnl_test.h
# End Source File
# Begin Source File

SOURCE=.\vnl_trace.h
# End Source File
# Begin Source File

SOURCE=.\vnl_transpose.h
# End Source File
# Begin Source File

SOURCE=.\vnl_unary_function.h
# End Source File
# Begin Source File

SOURCE=.\vnl_vector.h
# End Source File
# Begin Source File

SOURCE=.\vnl_vector_dereference.h
# End Source File
# Begin Source File

SOURCE=.\vnl_vector_fixed.h
# End Source File
# Begin Source File

SOURCE=.\vnl_vector_ref.h
# End Source File
# End Group
# End Target
# End Project
