function binary=build(compiler)
%% Section 0: Header & Readme
% SYNTAX: binary=fastcone.build(); binary=fastcone.build(compiler)
% PURPOSE: Build the native analytical cone kernel from the bundled sources.
% INPUTS: Optional Windows MinGW g++ executable; otherwise use configured mex.
% OUTPUTS: Full binary path. Compiler failures raise an actionable error.
% UNITS: None. No downloads or runtime compilation occur during a solve.

%% Section 1: Locate The Pinned Sources And Compiler
package=fileparts(mfilename('fullpath'));
source=fullfile(package,'native','fastcone_core.cpp');
eigen=fullfile(package,'third_party','eigen');
binary=fullfile(package,['core.' mexext]);
assert(isfile(fullfile(eigen,'Eigen','Sparse')),'fastcone:MissingEigen', ...
    'The pinned Eigen headers are missing from %s.',eigen);
if nargin==0
    compiler='';
    if ispc && isempty(mex.getCompilerConfigurations('C++','Selected')) && ...
            isfile('C:\msys64\mingw64\bin\g++.exe')
        compiler='C:\msys64\mingw64\bin\g++.exe';
    end
end

%% Section 2: Compile Without Reassociating Floating-Point Expressions
if ~isempty(compiler)
    assert(ispc && isfile(compiler),'fastcone:InvalidCompiler', ...
        'The explicit compiler must be an installed Windows MinGW g++ executable.');
    library=fullfile(matlabroot,'extern','lib','win64','mingw64');
    parts={quote(compiler),'-shared','-O3','-std=c++17','-DNDEBUG', ...
        '-DMATLAB_MEX_FILE','-ffp-contract=off','-static-libgcc','-static-libstdc++', ...
        ['-I' quote(fullfile(matlabroot,'extern','include'))],['-I' quote(eigen)], ...
        quote(source),quote(fullfile(library,'libmex.lib')), ...
        quote(fullfile(library,'libmx.lib')),quote(fullfile(library,'mexFunction.def')), ...
        '-o',quote(binary)};
    [status,message]=system(strjoin(parts,' '));
    assert(status==0,'fastcone:BuildFailed','Native compilation failed: %s',message);
else
    config=mex.getCompilerConfigurations('C++','Selected');
    assert(~isempty(config),'fastcone:NoCompiler', ...
        'Configure a C++ compiler with mex -setup C++, then run fastcone.build.');
    flags='CXXFLAGS=$CXXFLAGS -O3 -std=c++17 -ffp-contract=off';
    if contains(config.Manufacturer,'Microsoft')
        flags='COMPFLAGS=$COMPFLAGS /O2 /std:c++17 /fp:precise';
    end
    mex('-R2017b',flags,'-DNDEBUG',['-I' eigen],source, ...
        '-outdir',package,'-output','core');
end
rehash;
fprintf('Built %s\n',binary);
end

function value=quote(value)
% Windows paths cannot contain literal double quotes.
assert(~contains(value,'"'),'fastcone:InvalidPath','A build path contains a quote.');
value=['"' char(value) '"'];
end
