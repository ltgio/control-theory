%
%    This file was auto-generated using the ACADO Toolkit.
%    
%    While ACADO Toolkit is free software released under the terms of
%    the GNU Lesser General Public License (LGPL), the generated code
%    as such remains the property of the user who used ACADO Toolkit
%    to generate this code. In particular, user dependent data of the code
%    do not inherit the GNU LGPL license. On the other hand, parts of the
%    generated code that are a direct copy of source code from the
%    ACADO Toolkit or the software tools it is based on, remain, as derived
%    work, automatically covered by the LGPL license.
%    
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%    


function make_acado_integrator( name, extern )

	% Output file name, and also function name
	if (nargin > 0)
		fileOut = name;
	else
		fileOut = 'acado_integrator';
	end;
		
	% Root folder of code generation
	CGRoot = '.';	
		
	% Auto-generated files
	CGSources = [ ...
		'acado_integrate.c ' ...
		'acado_integrator.c ' ...
		'acado_auxiliary_sim_functions.c ' ...
		];
	if (nargin > 1)
		CGSources = [CGSources extern];
	end
		
	% Adding additional linker flags for Linux
	ldFlags = '';
	if (isunix() && ~ismac())
		ldFlags = '-lrt';
    elseif (ispc)
        ldFlags = '-DWIN32';
	end;

	% Recipe for compilation
	CGRecipe = [ ...
		'mex -O' ...
		' -I. -I''CGRoot''' ...
		' ldFlags' ...
		' -D__MATLAB__ -O CGSources -output %s.%s' ...
		];

% Compilation
CGSources = regexprep(CGSources, 'CGRoot', CGRoot);

CGRecipe = regexprep(CGRecipe, 'CGRoot', CGRoot);
CGRecipe = regexprep(CGRecipe, 'CGSources', CGSources);
CGRecipe = regexprep(CGRecipe, 'ldFlags', ldFlags);

% disp( sprintf( CGRecipe, fileOut, mexext ) ); 
fprintf( 'compiling... ' );
eval( sprintf(CGRecipe, fileOut, mexext) );
fprintf( ['done! --> ' fileOut '.' mexext '\n'] );
