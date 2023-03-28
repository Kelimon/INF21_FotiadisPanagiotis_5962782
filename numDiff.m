function df = numDiff(method, func, x)
%NUMDIFF - Numerical differentiation using forward, backward, or central method
%
%   df = numDiff(method, func, x) differentiates the function func at
%   point x using the numerical differentiation method specified by method.
%   method can be 'forward' for forward difference, 'backward' for backward
%   difference, or 'central' for central difference. The output df is the
%   result derivative.
%
%   Inputs:
%       method - String that specifies the numerical differentiation method
%                to use. 'forward' will be used as default.
%       func   - Function handle that specifies the function to differentiate.
%       x      - Point at which to differentiate the function func.
%
%   Outputs:
%       DF     - Computed derivative of func at point x using method.
%
%   Example: 
%       df = numDiff('central', @sin, 0.5)
%       returns df = 0.87758, which is the computed derivative of sin(0.5)
%       using central difference with step size h=1e-6.
%
%   Other m-files required: none
%   Subfunctions: none
%   MAT-files required: none
%
%   See also: myNewton.m,  runmyNewton.m

%   Author: Panagiotis Fotiadis
%   email: panagiotis.fotiadis@hotmail.com
%   28.03.2023
%
%------------- BEGIN CODE --------------

% Implementation of numerical differentiation methods
if (strcmp(method, "forward") || strcmp(method, ''))
    h = 1e-8;
    df = (func(x+h) - func(x)) / h;

elseif strcmp(method, "backward")
    h = 1e-8;
    df = (func(x) - func(x-h)) / h;

elseif strcmp(method, "central")
    h = 1e-6;
    df = (func(x+h) - func(x-h)) / (2*h);
end

end
%------------- END OF CODE --------------
